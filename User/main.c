#include "gd32f4xx.h"
#include "rm68120_exmc.h"
#include "debug_uart.h"
#include "gfx.h"
#include "ui.h"
#include "waterfall.h"
#include "touch.h"
#include "aic3204.h"
#include "ms5351.h"
#include "rf_lpf.h"
#include "encoder.h"
#include "battery.h"
#include "backlight.h"
#include "arm_math.h" /* arm_fir_decimate_* - spectrum ZOOM, see spec_zoom_t's comment */
#include "demod_am.h"
#include "gd32_i2s.h"
#include "sdr_rx.h"
#include "fft.h"
#include "spectrum.h"
#include "splash_screen.h"
#include "splash_screen.h"

static void led_gpio_init(void);
static void speaker_pa_gpio_init(void);
static void speaker_pa_set_enabled(uint8_t on);
static void systick_delay_init(void);
static void radio_screen_draw(void);
static void sdr_spectrum_waterfall_tick(void);
static void demo_touch_poll(void);
static void freq_display_draw(void);
static void step_display_draw(void);
static void aux_row_display_draw(void);
static void mode_display_draw(void);
static void time_display_draw(void);
static void battery_display_draw(void);
static void badges_draw(void);
static void smeter_draw(uint8_t segs);
static uint8_t smeter_segments_from_peak(float peak);
static void tune_encoder_poll(void);
static void menu_screen_open(void);
static void menu_screen_close(void);
static void zoom_decimators_init(void);
static void menu_grid_show(void);
static void menu_bands_show(void);
static void menu_step_list_show(void);
static void menu_mode_list_show(void);
static void apply_lo_tune(uint32_t freq_hz);
static void menu_detail_value_redraw(void);
static void settings_value_redraw(void);

/* Set to 0 to go back to the normal demo once the real panel height is calibrated. */
#define CALIB_HEIGHT_TEST 0
#if CALIB_HEIGHT_TEST
static void calib_height_ruler_draw(void);
#endif

volatile uint32_t g_msticks = 0; /* incremented in SysTick_Handler, 1 tick = 1ms real time */
volatile uint16_t g_last_rddpm  = 0; /* last value read from RDDPM (0x0A00) */
volatile uint16_t g_last_rddsdr = 0; /* last value read from RDDSDR (0x0F00) */
volatile uint32_t g_fill_count  = 0; /* how many full fills have been done */
volatile uint16_t g_panel_id_check1 = 0; /* response to panel command 0x000A */
volatile uint16_t g_panel_id_check2 = 0; /* response to panel command 0x3A00 */
volatile uint32_t g_system_clock_snapshot = 0; /* copy of SystemCoreClock, to verify clock startup */

int main(void)
{
    /* Critical when chained after a bootloader: our vector table is no
     * longer at 0x08000000 (that's the bootloader's), but at
     * 0x08020000. Without this, any interrupt (including our own
     * SysTick) would look up its handler in the BOOTLOADER's vector
     * table, not ours - this must be the FIRST thing we do. */
    SCB->VTOR = 0x08020000;

    /*
     * Clear the NVIC state inherited from the bootloader BEFORE
     * re-enabling global interrupts. The bootloader may have left some
     * of its own interrupts enabled at the NVIC level (its own
     * SysTick, some DMA, etc.) which, with VTOR already pointing at
     * OUR vector table (unused entries = Default_Handler, a silent
     * infinite loop), would jump into a silent hang the moment they
     * got unmasked. Disabling everything and clearing pending flags
     * gives a clean slate; our own code then selectively re-enables
     * only what it configures (SysTick_Config below, EXTI2 later in
     * touch_init()).
     */
    {
        uint8_t i;
        for (i = 0; i < 8U; i++) {
            NVIC->ICER[i] = 0xFFFFFFFFU;
            NVIC->ICPR[i] = 0xFFFFFFFFU;
        }
    }

    /*
     * Also critical when chained after a bootloader: it's common for a
     * bootloader to leave interrupts globally disabled (PRIMASK)
     * before jumping to the application, to avoid handing off IRQ
     * state half-configured. Without this __enable_irq() here, NO
     * interrupt (SysTick included, touch.c's EXTI later on) would ever
     * fire, even with fully correct NVIC/EXTI configuration.
     */
    __enable_irq();

    /*
     * SystemInit() has already been called from the startup code
     * before main(): it configures PLL/HXTAL per system_gd32f4xx.c.
     * See that file for the active clock configuration.
     *
     * HXTAL_VALUE is set correctly via -D in the Makefile, matching
     * this board's real 12.288MHz crystal. SystemCoreClockUpdate()
     * re-reads the live PLL registers and recalculates
     * SystemCoreClock using that value - without calling it,
     * SystemCoreClock stays at the incorrect literal the vendor
     * startup file initializes it to.
     */
    SystemCoreClockUpdate();

    systick_delay_init();
    led_gpio_init();
    debug_uart_init();

    debug_print("\n\n=== STARTUP (chained after bootloader) ===\n");
    debug_print_hex32("VTOR read back", SCB->VTOR);

    /* Snapshot of SystemCoreClock as early as possible, to verify via
     * debugger whether the clock came up at the expected frequency or
     * whether the HXTAL crystal failed and silently settled elsewhere. */
    g_system_clock_snapshot = SystemCoreClock;
    debug_print_hex32("SystemCoreClock", g_system_clock_snapshot);

    debug_print_hex32("RCU_PLLI2S as early as possible in main()", RCU_PLLI2S);

    /*
     * REMOVED (30/07/2026): rm68120_init() itself - specifically its
     * rm68120_hw_reset() + fresh panel_init_sequence() - turned out to
     * be exactly what painted the panel red on boot, not a missing
     * clear. The bootloader already brings the panel up correctly
     * before handing off to this firmware (EXMC bus config included),
     * so re-running our own init just interrupts that known-good
     * state. Now relying entirely on the bootloader's init: no
     * rm68120_init() call here, and no gfx_fill_screen() either - the
     * panel is already black from the bootloader by the time we get
     * here, so there's nothing to blank.
     */
    debug_print_hex32("RCU_PLLI2S (rm68120_init skipped, using bootloader's panel init)",
                       RCU_PLLI2S);

    waterfall_init();
    touch_init();
    debug_print_hex32("RCU_PLLI2S after touch_init", RCU_PLLI2S);

    debug_print("\n--- AIC3204: phase 1 (I2C communication only) ---\n");
    aic3204_init(AIC3204_ADDR_DEFAULT);
    if (!aic3204_probe_and_reset()) {
        debug_print("aic3204: trying 0x19 (MODE pin tied to VDD)...\n");
        aic3204_init(0x19);
        if (!aic3204_probe_and_reset()) {
            aic3204_scan_bus();
        }
    }
    debug_print_hex32("RCU_PLLI2S after aic3204", RCU_PLLI2S);

    /*
     * MS5351 base configuration (PLLA 832MHz, 8pF load, CLK2 8MHz
     * prepared but off, OE = CLK0|CLK1). In the original firmware's
     * I2C capture this is the very FIRST traffic on the bus, before
     * any codec write; here it runs right after the codec probe only
     * because i2c_bitbang_init() lives inside aic3204_init(). The two
     * chips are independent, so the relative order between them does
     * not matter - what does match the capture is base-init BEFORE
     * codec config and the quadrature tune AFTER it (see below).
     */
    rf_lpf_init(); /* front-end LPF GPIOs ready before the first tune */
    encoder_init(); /* tuning knob: PD13(A)/PD12(B)/PC9(button, active high) */
    battery_init(); /* VBAT via ADC0 channel 18, see battery.h - independent
                      * of everything else in this sequence, so it just
                      * needs to run before the first battery_display_draw() */
    backlight_init(); /* PWM brightness on PA3 (TIMER1_CH3), see backlight.h -
                        * likewise independent; run before the panel is first
                        * drawn so it's never dark/at some undefined duty
                        * cycle for even one frame. */
    speaker_pa_gpio_init(); /* speaker PA enable/mute, see its own comment -
                              * independent GPIO, same reasoning as the other
                              * inits in this block; run before audio starts
                              * flowing so the speaker is never briefly at an
                              * undefined level for even one frame. */
    ms5351_init();

    debug_print("\n--- MCLK: TIMER2_CH0/PC6, 1.536MHz ---\n");
    gd32_i2s_mclk_timer_start();

    debug_print("\n--- I2S1: phase 3 (clocks + circular DMA, test tone) ---\n");
    gd32_i2s_init_slave_192k();

    /*
     * REORDERED (28/07/2026): sdr_rx_init() moved to run IMMEDIATELY
     * after gd32_i2s_init_slave_192k() enables I2S1_ADD's receive
     * path, instead of after aic3204_phase2_init(). The bit-banged I2C
     * codec configuration takes a non-trivial amount of time (many
     * register writes), and with real bits already arriving on PB14
     * every ~10us the moment I2S1_ADD is enabled, that whole gap was
     * long enough for SPI_STAT_RXORERR (receive overrun) to fire
     * continuously with nothing servicing it - by the time DMA was
     * finally armed, the overrun was mid-storm and never actually
     * cleared, which is a strong candidate for why captured samples
     * stayed pinned at a fixed value. Arming DMA first minimizes that
     * unserviced gap to just the few lines of DMA setup itself.
     */
    fft_init();
    spectrum_init(); /* palette LUT for spectrum + waterfall */
    zoom_decimators_init(); /* spectrum ZOOM cascaded decimators - see spec_zoom_t's comment */
    sdr_rx_init();

    debug_print("\n--- AIC3204: phase 2 (clock + single-ended ADC baseline + power-up) ---\n");
    aic3204_phase2_init();

    /*
     * Audio out: switch DMA0/CH4 from the bring-up test tone to the
     * ping-pong TX stream, then register the AM demodulator as the
     * per-block RX hook (runs in the RX DMA interrupt - see
     * demod_am.h for why it cannot live in this loop). Order matters:
     * the stream transport must exist before the hook can write into
     * it.
     */
    debug_print("\n--- Audio: TX stream + AM demodulator hook ---\n");
    gd32_i2s_dma_start_stream();
    demod_am_init();
    sdr_rx_set_block_hook(demod_am_process_raw);

    /*
     * Quadrature LO for the QSD, same point in the sequence as the
     * capture (right after the codec is fully configured). For
     * bring-up we replay the hardware-proven captured bytes
     * (90.800MHz, FM broadcast - whatever station is around should
     * show up in the spectrum, which makes for a great smoke test).
     * Once validated, switch to ms5351_set_lo_freq(hz): calling it
     * with MS5351_CAPTURED_LO_HZ emits these exact same bytes.
     *
     * (A low-IF offset scheme - LO tuned below the selected station
     * plus a digital down-mix in demod_am.c, to move the demodulated
     * signal off the QSD's DC-centered LO-leakage artifact - was
     * tried and fully reverted on 30/07/2026: it caused a broadband
     * noise floor / jumping spectrum on hardware. The person
     * confirmed the I2S signals and the MS5351's LO were both fine on
     * their own, so the actual cause wasn't the LO retuning itself
     * and is still unidentified - see demod_am.h for the full note if
     * picking this back up.)
     */
    debug_print("\n--- MS5351: quadrature LO tune ---\n");
    ms5351_tune_captured();

#if CALIB_HEIGHT_TEST
    calib_height_ruler_draw();
#else
    splash_screen_draw(); /* personalizable splash screen, see splash_screen.c */
    radio_screen_draw(); /* full radio UI, all readouts included */
#endif

    debug_print("main: entering the main loop\n");

    while (1) {
#if !CALIB_HEIGHT_TEST
        sdr_spectrum_waterfall_tick();
        demo_touch_poll();
        tune_encoder_poll();
#endif

        g_fill_count++;

        if ((g_fill_count % 50) == 0) {
            debug_print_dec("waterfall ticks", g_fill_count);
            /* ISR timing check (see demod_am.h's comment above
             * demod_am_get_last_cycles()): one block's real-time
             * budget is SDR_RX_BLOCK_SAMPLES samples at 192kHz. If
             * "demod ISR cycles" gets close to or over "block budget
             * cycles", the demod ISR doesn't fit in real time -
             * exactly the situation suspected in the USB/LSB hang
             * report. */
            debug_print_dec("demod ISR cycles (last block)", demod_am_get_last_cycles());
            debug_print_dec("block budget cycles (192kHz, for reference)",
                             (SystemCoreClock / 192000UL) * SDR_RX_BLOCK_SAMPLES);
            {
                /* Per-stage breakdown (31/07/2026, see
                 * demod_am_get_last_cycles_breakdown()'s comment) -
                 * pins down which stage a total-cycles jump actually
                 * comes from, instead of guessing. */
                demod_am_cycles_breakdown_t bd = demod_am_get_last_cycles_breakdown();
                debug_print_dec("  frontend (deinterleave/down-mix/CHF)", bd.frontend);
                debug_print_dec("  extract  (mode-specific: AM/WFM/SSB)", bd.extract);
                debug_print_dec("  audio    (DC block + audio LPF)", bd.audio);
                debug_print_dec("  agc_out  (AGC + I2S write)", bd.agc_out);
            }
        }
    }
}

/*
 * Calibration build: draws a horizontal ruler (tick + text label with
 * the Y value) every 40px from 0 to GFX_SCREEN_HEIGHT-1, plus an exact
 * border at (0,0,GFX_SCREEN_WIDTH-1,GFX_SCREEN_HEIGHT-1). Photograph
 * the panel and compare: the last label that reads COMPLETE (not cut
 * off) before the panel's real bottom edge indicates the true usable
 * height. If the drawn bottom border isn't visible at all,
 * GFX_SCREEN_HEIGHT is still larger than the real physical height.
 */
#if CALIB_HEIGHT_TEST
static void calib_height_ruler_draw(void)
{
    char label[8];
    uint16_t y;

    gfx_fill_screen(GFX_COLOR_BLACK);

    /* Exact border at the limits we currently assume */
    gfx_rect(0, 0, GFX_SCREEN_WIDTH, GFX_SCREEN_HEIGHT, GFX_COLOR_RED);

    for (y = 0; y < GFX_SCREEN_HEIGHT; y += 40) {
        uint8_t i = 0;
        uint16_t v = y;
        char tmp[8];
        uint8_t n = 0;

        /* Manual itoa (no sprintf, to avoid pulling in more of newlib) */
        if (v == 0) {
            tmp[n++] = '0';
        } else {
            while (v > 0 && n < sizeof(tmp)) {
                tmp[n++] = (char)('0' + (v % 10));
                v /= 10;
            }
        }
        while (n > 0) {
            label[i++] = tmp[--n];
        }
        label[i] = '\0';

        gfx_hline(0, y, 20, GFX_COLOR_YELLOW);
        gfx_text(24, (uint16_t)((y >= 3) ? (y - 3) : 0), label,
                  GFX_COLOR_CYAN, GFX_COLOR_BLACK, 1);
    }
}
#endif /* CALIB_HEIGHT_TEST */

/*
 * RADIO UI LAYOUT (30/07/2026 redesign - replaces the original 3-button
 * demo screen; still doubles as the gfx.c/ui.c validation surface).
 * Landscape 800x480 (confirmed on real hardware, see the
 * GFX_SCREEN_WIDTH/HEIGHT comment in gfx.h). Zones:
 *
 *   +--------------------------------------------------------------+
 *   | TOP BAR (h=64): freq (big) | mode | step+vol | time | batt   |
 *   +---------------------------------------------------+----------+
 *   | SPECTRUM (676 wide, 280 tall)                     | RIGHT    |
 *   +---------------------------------------------------+ COLUMN   |
 *   | WATERFALL (672 x 72 rows)                         | S-meter  |
 *   |                                                   | + badges |
 *   +---------------------------------------------------+----------+
 *   | BOTTOM BAR: 6 buttons (MODE VOL STEP NR BANDS MENU)          |
 *   +--------------------------------------------------------------+
 *
 * Every coordinate is an internally-linked constant (static const, not
 * a macro) so sdr_spectrum_waterfall_tick() uses exactly the same
 * values as radio_screen_draw() without duplicating arithmetic by
 * hand.
 */
static const uint16_t TOP_H        = 64;

/* Main (left) display column: spectrum over waterfall. */
static const uint16_t MAIN_W       = 676;             /* panel width, border included  */
static const uint16_t SPEC_Y       = 64;
static const uint16_t SPEC_H       = 280;
static const uint16_t SPEC_TRACE_X = 2;               /* inside the 1px panel border   */
static const uint16_t SPEC_TRACE_W = 672;             /* = WATERFALL_WIDTH; /4 exact for the SR/4 marker */
static const uint16_t WF_PANEL_Y   = 64 + 280 + 2;    /* = 346 */
static const uint16_t WF_Y         = 64 + 280 + 4;    /* first waterfall row           */

/* Right-hand status column: S-meter + up to 6 state badges. */
static const uint16_t RCOL_X       = 678;
static const uint16_t RCOL_W       = 122;             /* to x=799 inclusive            */
static const uint16_t RCOL_Y       = 64;
static const uint16_t RCOL_H       = 358;             /* down to the button bar        */

/* Bottom button bar: 6 buttons. */
static const uint16_t BTNBAR_Y     = 428;
static const uint16_t BTNBAR_BTN_W = 121;
static const uint16_t BTNBAR_BTN_H = 46;
static const uint16_t BTNBAR_GAP   = 10;              /* 6*121 + 7*10 = 796 <= 800     */

/*
 * IMPORTANT: these widgets are static (not local to
 * radio_screen_draw()) on purpose. ui_screen_t only stores POINTERS to
 * them (to avoid duplicating data or depending on malloc), so they
 * must stay alive for as long as the screen exists - if they were
 * stack variables of a function that already returned,
 * ui_screen_touch() would be reading stack memory already reused by
 * another call. This is exactly the kind of bug that doesn't produce
 * a compile error but silently corrupts memory at runtime.
 */
static ui_screen_t s_demo_screen;
static ui_panel_t  s_title_panel;
static ui_panel_t  s_spectrum_panel;
static ui_panel_t  s_waterfall_panel;
static ui_panel_t  s_rcol_panel;
static ui_button_t s_btn_mode;
static ui_button_t s_btn_vol;
static ui_button_t s_btn_step;
static ui_button_t s_btn_nr;
static ui_button_t s_btn_bands; /* bottom-bar shortcut to menu_bands_show() - repurposed 02/08/2026 from the SPT smoothing-cycle shortcut, see demo_button_callback()'s comment */
static ui_button_t s_btn_menu;
/* Added 31/07/2026: unlike the 6 bottom-bar buttons above, this one
 * lives in the badge grid (see badges_draw()) - a real, touchable
 * ui_button_t standing in for what used to be a plain badge_draw()
 * call, so tapping it can cycle the AGC profile directly instead of
 * needing yet another bottom-bar slot (all 6 are already spoken for -
 * see demo_button_callback()'s header comment) or another MENU cycle
 * position. ui_button_draw()'s rendering (fill+border+centered label)
 * already matches badge_draw()'s look, so no visual seam. */
static ui_button_t s_btn_agc_profile;
static ui_button_t s_btn_audio_bw; /* AM/SSB audio filter width - real touchable button (see badges_draw()'s comment), added 02/08/2026 replacing the BW slot's old plain badge_draw() call */

/*
 * encoder_target_t - hoisted up here (was originally declared further
 * down, right before s_encoder_target - see that declaration's full
 * comment for what this enum means and how each value is reached) so
 * it's available for s_menu_detail_target below, which needs the type
 * before its own declaration point. Fixes a real build error: the
 * settings-menu block was declared before this enum existed in the
 * file, even though C only cares about textual order, not "logical"
 * grouping.
 */
typedef enum {
    ENCODER_TARGET_TUNE = 0,
    ENCODER_TARGET_VOLUME,
    ENCODER_TARGET_BACKLIGHT,
    ENCODER_TARGET_SCALE,
    ENCODER_TARGET_SQUELCH,
    ENCODER_TARGET_SMOOTH,
    ENCODER_TARGET_PGA
} encoder_target_t;

/*
 * --- Settings menu screen (first pass, extended with real detail
 *     views 31/07/2026) -----------------------------------------------
 *
 * Added 31/07/2026: a SEPARATE ui_screen_t, not more widgets crammed
 * into s_demo_screen - the framework already supports this (ui_screen_t
 * is just a widget registry + dispatcher, see ui.h, no assumption
 * anywhere that only one screen exists). MENU now OPENS this screen
 * (menu_screen_open()) instead of cycling s_encoder_target through
 * BACKLIGHT/SCALE/SQUELCH - see demo_button_callback()'s MENU branch
 * for what that replaces.
 *
 * TWO LEVELS inside this one screen:
 *   - GRID (menu_grid_show()): the 4x2 tile overview - tapping
 *     AGC/SPT acts immediately (cycles/toggles in place, same as
 *     before); tapping SQUELCH/BACKLIGHT/SCALE/VOLUME/SMOOTH now
 *     opens...
 *   - DETAIL (menu_detail_show()): a single big value + a BACK tile,
 *     replacing what used to be "select the target and bounce back to
 *     the main screen to see it change" - per the project owner, that
 *     wasn't REAL interaction (you couldn't see the value change from
 *     inside the menu at all). Turning the knob now updates the big
 *     value live, right here - see settings_value_redraw()'s comment
 *     for how tune_encoder_poll() knows which view to repaint.
 *
 * CONFINED to the spectrum+waterfall panel (MENU_AREA_*, see
 * menu_grid_show()), not a full-screen takeover - changed 31/07/2026,
 * again per the project owner: the top bar, right column, and bottom
 * button bar now stay live and touchable the ENTIRE time the menu is
 * open, only the graph area gets replaced by the tile grid (or a
 * detail view). This means TWO screens are effectively active at
 * once, split by SCREEN REGION rather than by which one is "current":
 *   - demo_touch_poll() routes each touch sample to s_menu_screen if
 *     it lands inside MENU_AREA while s_menu_open, otherwise always to
 *     s_demo_screen (see its own comment for the accepted drag-across-
 *     the-boundary edge case).
 *   - sdr_spectrum_waterfall_tick() keeps the S-meter and time readout
 *     updating every frame regardless of s_menu_open (both live in the
 *     still-visible top bar/right column) but skips the FFT-averaging/
 *     smoothing/spectrum_draw()/waterfall work while the menu covers
 *     that panel - see its own comment for exactly where that split
 *     happens.
 *   - menu_screen_close() only needs to restore the spectrum+waterfall
 *     PANEL BORDERS (ui_panel_draw() on s_spectrum_panel/
 *     s_waterfall_panel) - everything else was never touched, so a
 *     full radio_screen_draw() would just be wasted EXMC bandwidth and
 *     a visible flash of things that never changed. The actual trace/
 *     waterfall CONTENT follows on the next tick's frame, ~33ms later
 *     at most - imperceptible.
 */
static ui_screen_t s_menu_screen;
static uint8_t s_menu_open = 0U;
/* s_menu_detail_active: 0 = grid showing, 1 = a detail view showing.
 * s_menu_detail_target: WHICH detail view, when active - reuses
 * encoder_target_t rather than inventing a parallel enum, since the
 * detail view IS "adjust whatever the encoder currently targets". */
static uint8_t s_menu_detail_active = 0U;
static encoder_target_t s_menu_detail_target = ENCODER_TARGET_TUNE;
/* s_menu_bands_active: same idea as s_menu_detail_active, one level
 * up - 0 = grid (or a detail view) showing, 1 = the BANDS preset list
 * showing. Mutually exclusive with s_menu_detail_active in practice
 * (menu_grid_show() clears both whenever it runs), but kept as its
 * own flag rather than folded into a 3-state enum - simplest thing
 * that reads clearly at each of the few call sites that check it. */
static uint8_t s_menu_bands_active = 0U;
/* s_menu_step_active / s_menu_mode_active: same bookkeeping idea as
 * s_menu_bands_active, for the two picker lists reachable directly
 * from the bottom bar (menu_step_list_show()/menu_mode_list_show()) -
 * see those functions' comments. Unlike BANDS these have no parent
 * grid to belong to (they're opened straight from STEP/MODE, not from
 * within the settings menu), but they still need SOME flag set while
 * open for the same "which sub-view of s_menu_screen is this" reason,
 * even though nothing branches on them today beyond set/reset. */
static uint8_t s_menu_step_active = 0U;
static uint8_t s_menu_mode_active = 0U;
/*
 * --- Settings grid PAGES (RADIO / UI / HW) -------------------------------
 *
 * Added 03/08/2026, per the project owner: the settings grid's left
 * column is now a fixed 3-tile PAGE SELECTOR (RADIO/UI/HW, one per
 * row), and the remaining 3x3 tiles on the right show whichever
 * page's options are currently selected. Only ONE flag is needed
 * (s_menu_page) - unlike s_menu_detail_active/s_menu_bands_active/etc.
 * above, the page selector doesn't leave the grid (it just changes
 * WHAT the grid shows), so there's no separate "is a page open"
 * boolean, just which page.
 *
 * menu_page_select_callback() is the page tiles' shared callback -
 * switches s_menu_page and re-runs menu_grid_show() to repaint both
 * the (now differently-highlighted) page column and the new page's
 * options in one go. See menu_grid_show()'s own comment for the slot
 * layout of the options side.
 */
typedef enum {
    MENU_PAGE_RADIO = 0,
    MENU_PAGE_UI,
    MENU_PAGE_HW,
    MENU_PAGE_COUNT
} menu_page_t;

static menu_page_t s_menu_page = MENU_PAGE_RADIO;
static ui_button_t s_menu_page_radio;
static ui_button_t s_menu_page_ui;
static ui_button_t s_menu_page_hw;
static ui_button_t s_menu_tile_agc;
static ui_button_t s_menu_tile_squelch;
static ui_button_t s_menu_tile_backlight;
static ui_button_t s_menu_tile_scale;
static ui_button_t s_menu_tile_volume;
static ui_button_t s_menu_tile_nb;
static ui_button_t s_menu_tile_smooth; /* was the reserved/empty slot - see menu_grid_show() */
static ui_button_t s_menu_tile_spec_style; /* spectrum trace style toggle, added 31/07/2026 alongside the region resize */
static ui_button_t s_menu_tile_zoom; /* spectrum/waterfall zoom, see spec_zoom_t below */
static ui_button_t s_menu_tile_bw; /* AM/SSB audio filter width selector (4K0/2K3/1K8) - repurposed 02/08/2026 from the grid's BANDS tile, see menu_tile_bw_callback()'s comment */
static ui_button_t s_menu_tile_pga; /* AIC3204 MIC_PGA analog input gain - fills the grid's last spare slot */
static ui_button_t s_menu_tile_speaker_pa; /* speaker PA enable/mute (PB7) - HW page, see its own comment */
/* s_speaker_pa_enabled: backs BOTH the tile's label (menu_tile_speaker_pa_refresh())
 * and the actual GPIO level (speaker_pa_set_enabled(), defined down
 * with the rest of the GPIO drivers near led_gpio_init() - declared
 * here instead, alongside the tile, since it's used well before that
 * point in the file). */
static uint8_t s_speaker_pa_enabled = 1U; /* speaker on by default at boot */
static ui_button_t s_menu_tile_exit;
static ui_button_t s_menu_detail_back; /* the DETAIL view's only widget besides the value text itself */
/* Backing buffers for the tiles whose label needs to show a live value
 * (AGC/SQUELCH/BACKLIGHT/VOLUME/SPT/SMOOTH/SPEC/ZOOM/PGA) - ui_button_t.label
 * is just a const char*, so whatever it points at must outlive the
 * button. SCALE and EXIT use plain string literals instead (SCALE
 * still shows no live value on its GRID tile - see menu_grid_show()'s
 * comment; its DETAIL view does show both LO and HI, see
 * menu_detail_value_redraw()). Sized generously; actual content is
 * always much shorter. */
static char s_menu_tile_agc_buf[16];
static char s_menu_tile_squelch_buf[16];
static char s_menu_tile_backlight_buf[16];
static char s_menu_tile_volume_buf[16];
static char s_menu_tile_pga_buf[16];
static char s_menu_tile_nb_buf[16];
static char s_menu_tile_smooth_buf[16];
static char s_menu_tile_spec_style_buf[16];
static char s_menu_tile_bw_buf[16];
static char s_menu_tile_zoom_buf[16];

/* s_nr_on is now VESTIGIAL - s_btn_nr was repurposed to cycle the AGC
 * profile instead (see agc_profile_cycle()'s comment and
 * demo_button_callback()'s NR branch), a quick stopgap ahead of a
 * proper settings-menu redesign. Nothing sets s_nr_on anymore, so its
 * row0 badge (see badges_draw()) just sits permanently off - left in
 * place rather than ripped out, since the whole badge grid is due for
 * a rework anyway and there's no point doing that cleanup twice. */
static uint8_t s_nr_on = 0U;

/* Spatial line-smoothing pass count (0-3), fed straight into
 * spectrum_set_line_smooth() - see its comment in spectrum.h for what
 * it actually does to the trace. REPURPOSED 01/08/2026 from the old
 * NB (noise blanker) flag: NB never drove any real DSP, it just
 * flipped a badge/button/tile with a debug_print() behind it (see the
 * git history if that stub is ever needed again), so its three UI
 * slots - bottom-bar button, right-column badge, and settings-menu
 * tile - were free real estate for a control that actually does
 * something. Reachable via the SPT badge (indicator only) and the
 * settings-menu grid tile (menu_tile_nb_callback() - cycles
 * 0->1->2->3->0 on tap); the bottom-bar button that used to be a third
 * shortcut for this was itself repurposed 02/08/2026 to open the
 * BANDS list instead (see s_btn_bands' comment in
 * demo_button_callback()) - the smoothing pass count is still fully
 * reachable, just one tap deeper now. Default 0 (matches NB's old
 * default-off state); the project owner settled on 2-3 as the sweet
 * spot after testing by hand. */
static uint8_t s_spec_smooth_passes = 0U;

/*
 * --- Encoder-driven tuning state -----------------------------------
 *
 * s_tune_hz starts at the captured LO (90.800MHz, set by
 * ms5351_tune_captured() during startup); the first encoder movement
 * switches over to ms5351_set_lo_freq(), which for that exact
 * frequency emits the same registers, so the transition is seamless.
 *
 * The button cycles the tuning step. Limits: 100kHz is
 * ms5351_set_lo_freq()'s own LOWF_FLOOR_HZ (see ms5351.c) - lowered
 * from 4.8MHz on 31/07/2026 when the low-band quadrature technique
 * was ported in; NOT bench-confirmed down at the very bottom of that
 * range yet, see ms5351_set_lo_freq_lowband()'s comment for why. 180MHz
 * is the top of the front-end LPF bank (rf_lpf.c).
 */
#define TUNE_MIN_HZ 100000UL
#define TUNE_MAX_HZ 180000000UL

static uint32_t s_tune_hz = MS5351_CAPTURED_LO_HZ;
/* Extended 31/07/2026 (100/1K/10K/100K/1M -> 8 steps) to cover the
 * channel spacings real bands actually use, needed for the BANDS
 * presets below (each preset picks an INDEX into this array, see
 * band_preset_t) - 5K for SW AM broadcast, 12K5/25K for VHF voice
 * channels (2m repeaters, airband). Order matters: BAND_STEP_5K etc.
 * below are literal indices into this array, so inserting/removing/
 * reordering an entry means updating those too. */
static const uint32_t k_tune_steps[] = {
    100UL, 1000UL, 5000UL, 10000UL, 12500UL, 25000UL, 100000UL, 1000000UL
};
#define BAND_STEP_100HZ 0U
#define BAND_STEP_1K    1U
#define BAND_STEP_5K    2U
#define BAND_STEP_10K   3U
#define BAND_STEP_12K5  4U
#define BAND_STEP_25K   5U
#define BAND_STEP_100K  6U
#define BAND_STEP_1M    7U
/* Uppercase K/M, not lowercase: gfx_font5x7 only covers 0x20-0x5A
 * (space, digits, UPPERCASE, punctuation - see gfx_font.h) - no
 * lowercase glyphs exist, and gfx_glyph_for() silently substitutes a
 * blank space for anything outside that range. A lowercase "1k" was
 * rendering as "1 " (the 'k' just missing, not garbled - easy to miss
 * at a glance), while "1M" happened to look fine because 'M' is
 * already uppercase. Found 31/07/2026 - see also the panadapter's
 * "+/-96K" span labels below, same root cause, same fix. "12K5" (not
 * "12.5K" - the font has no '.') is the closest 4-char fit; it's a
 * common enough ham-radio convention for 12.5kHz to read fine. */
static const char *k_tune_step_labels[] = {
    "100 ", "1K  ", "5K  ", "10K ", "12K5", "25K ", "100K", "1M  "
};
#define TUNE_STEP_COUNT (sizeof(k_tune_steps) / sizeof(k_tune_steps[0]))
static uint8_t s_tune_step_idx = 6; /* start at 100kHz (index shifted by the 31/07/2026 table extension - was index 3 in the old 5-entry table) - handy in the FM band the capture left us in */

/* STEP picker list - added 01/08/2026, replaces the old "STEP button
 * just cycles to the next entry" behavior with a real pick-from-a-list
 * screen (see menu_step_list_show()): TUNE_STEP_COUNT is exactly 8, so
 * it fills one 4x2 grid page with no leftover slots, same tile
 * geometry as the BANDS list below. Declared here (not up with
 * s_menu_screen's other widgets) since it needs TUNE_STEP_COUNT, same
 * reasoning as s_menu_band_tiles' comment. */
static ui_button_t s_menu_step_tiles[TUNE_STEP_COUNT];

/*
 * MODE picker list - added 01/08/2026, same treatment as STEP above:
 * replaces the old "MODE button cycles AM->USB->LSB->NFM->WFM->AM" with
 * a pick screen (see menu_mode_list_show()). label+demod_mode_t pairs
 * in that same order, purely so the grid fills left-to-right/top-to-
 * bottom in the order people expect from the old cycle.
 */
typedef struct {
    const char *label;
    demod_mode_t mode;
} demod_mode_entry_t;

static const demod_mode_entry_t k_demod_modes[] = {
    { "AM",  DEMOD_MODE_AM  },
    { "USB", DEMOD_MODE_USB },
    { "LSB", DEMOD_MODE_LSB },
    { "NFM", DEMOD_MODE_NFM },
    { "WFM", DEMOD_MODE_WFM }
};
#define DEMOD_MODE_ENTRY_COUNT (sizeof(k_demod_modes) / sizeof(k_demod_modes[0]))
static ui_button_t s_menu_mode_tiles[DEMOD_MODE_ENTRY_COUNT];

/*
 * --- BANDS presets -------------------------------------------------------
 *
 * Added 31/07/2026, per the project owner: quick-jump tiles for the
 * most commonly used bands within this board's 4.8-180MHz tuning
 * range (TUNE_MIN_HZ/MAX_HZ above), each bundling a starting
 * frequency + demod mode + tuning step into one tap. Frequencies are
 * reasonable STARTING POINTS inside each band, not band-EDGE
 * markers - picked to land on something likely to have activity, not
 * necessarily the technical bottom of the allocation.
 *
 * SW BROADCAST (49m/41m/31m/19m): AM, 5kHz step (standard SW
 * broadcast channel spacing). Only 4 of the many SW broadcast bands -
 * a reasonably representative spread across the HF spectrum, not an
 * exhaustive list; easy to add more entries the same way.
 * FM BCST (commercial FM broadcast): WFM, 100kHz step, 88.0MHz - the
 * bottom of the commercial FM band almost everywhere.
 * AIRBAND (civil aviation voice): AM - aviation voice is AM, not FM,
 * unlike everything else in this table - 25kHz step (the simpler,
 * widely-used channel spacing; some regions use 8.33kHz instead,
 * not offered here as a separate step yet).
 * 2M (amateur 2 meter band): NFM, 12.5kHz step (standard repeater/
 * simplex channel spacing in most of Europe).
 * VHF HI: NFM, 12.5kHz step, 150.0MHz - a general "above 2m, below
 * the 180MHz ceiling" catch-all (marine VHF, PMR, and similar narrow-
 * band VHF traffic live in this general region) rather than one
 * specific named allocation - the vaguest entry here, worth revisiting
 * once you know what you actually want to listen to up there.
 *
 * 80M/40M/20M (amateur HF, added 02/08/2026): LSB below 10MHz, USB
 * above - the standard amateur-radio sideband convention, not an
 * arbitrary choice per band. 1kHz step (BAND_STEP_1K) for SSB tuning
 * precision - the 5/25kHz broadcast/aviation steps above would be far
 * too coarse to sit on a voice QSO here. Frequencies are common phone-
 * segment calling/activity spots, not band edges: 80M 3.750MHz (IARU
 * R1 SSB calling area), 40M 7.150MHz (R1 voice segment, comfortably
 * inside 7.130-7.200), 20M 14.250MHz (a generally busy R1/R2 SSB
 * spot).
 * 11M (added 02/08/2026): technically CB, NOT an amateur allocation
 * anywhere today (it was, decades ago, in some countries - hence
 * still being lumped in with "ham bands" colloquially) - included
 * anyway per the project owner's request. 27.185MHz is CB channel 19,
 * historically the most active AM calling channel (truckers). AM,
 * 10kHz step (BAND_STEP_10K) - the actual 40-channel CB spacing.
 */
typedef struct {
    const char *label;
    uint32_t freq_hz;
    demod_mode_t mode;
    uint8_t step_idx; /* index into k_tune_steps[]/k_tune_step_labels[] - see BAND_STEP_* above */
} band_preset_t;

static const band_preset_t k_band_presets[] = {
    { "SW 49M",  6000000UL,   DEMOD_MODE_AM,  BAND_STEP_5K   },
    { "SW 41M",  7200000UL,   DEMOD_MODE_AM,  BAND_STEP_5K   },
    { "SW 31M",  9500000UL,   DEMOD_MODE_AM,  BAND_STEP_5K   },
    { "SW 19M",  15100000UL,  DEMOD_MODE_AM,  BAND_STEP_5K   },
    { "FM BCST", 88000000UL,  DEMOD_MODE_WFM, BAND_STEP_100K },
    { "AIRBAND", 118000000UL, DEMOD_MODE_AM,  BAND_STEP_25K  },
    { "2M",      144000000UL, DEMOD_MODE_NFM, BAND_STEP_12K5 },
    { "VHF HI",  150000000UL, DEMOD_MODE_NFM, BAND_STEP_12K5 },
    { "80M",     3750000UL,   DEMOD_MODE_LSB, BAND_STEP_1K   },
    { "40M",     7150000UL,   DEMOD_MODE_LSB, BAND_STEP_1K   },
    { "20M",     14250000UL,  DEMOD_MODE_USB, BAND_STEP_1K   },
    { "11M",     27185000UL,  DEMOD_MODE_AM,  BAND_STEP_10K  }
};
#define BAND_PRESET_COUNT (sizeof(k_band_presets) / sizeof(k_band_presets[0]))
static ui_button_t s_menu_band_tiles[BAND_PRESET_COUNT]; /* the BANDS preset-list tiles, see menu_bands_show() - declared here, not up with s_menu_screen's other widgets, since it needs BAND_PRESET_COUNT */


/*
 * --- Encoder-driven volume/backlight/scale state ------------------------
 *
 * s_encoder_target selects what the tuning encoder currently controls:
 * TUNE (default, existing behavior, untouched), VOLUME (DAC digital
 * volume via aic3204_set_volume_db()), BACKLIGHT (PWM brightness via
 * backlight_set_percent() - see backlight.h), or SCALE (the
 * spectrum/waterfall vertical dB range, added 31/07/2026 alongside
 * BACKLIGHT's own addition - see the SPECTRUM SCALE block below).
 *
 * Was a plain uint8_t boolean (s_volume_mode) before BACKLIGHT
 * existed - an enum instead of stacking independent flags keeps "what
 * does the knob do right now" a single, unambiguous piece of state
 * (no risk of two targets ending up "on" at once). The VOL button
 * jumps straight to TUNE<->VOLUME (unchanged). BACKLIGHT/SCALE/
 * SQUELCH are now selected by tapping their tile in the settings menu
 * screen (menu_screen_open(), reached via MENU) instead of cycling
 * MENU repeatedly - see s_menu_screen's declaration comment for why
 * that changed 31/07/2026. VOL still jumps straight to VOLUME/TUNE
 * regardless of what's selected in the menu, same as before.
 *
 * s_volume_db_x2 is kept directly in the hardware's native 0.5dB
 * units (same encoding aic3204_set_volume_db() converts to
 * internally) - avoids float rounding drift across repeated encoder
 * adjustments. Starts at 0 (0dB, unity - matches the byte-exact
 * captured baseline aic3204_phase2_init() leaves the chip at, so
 * turning the encoder for the first time doesn't jump the volume).
 *
 * (encoder_target_t itself is now declared earlier in this file -
 * see the comment right before the settings-menu block above - since
 * s_menu_detail_target needed the type before this point.)
 */

static void menu_detail_show(encoder_target_t target);


static encoder_target_t s_encoder_target = ENCODER_TARGET_TUNE;
static int16_t s_volume_db_x2 = 0;
#define VOLUME_STEP_X2 2 /* 2 * 0.5dB = 1.0dB per encoder detent */
#define VOLUME_MIN_X2  (-127)  /* -63.5dB */
#define VOLUME_MAX_X2  48      /* +24.0dB */
/* PGA (analog input gain, MIC_PGA_L/R - see aic3204_set_pga_gain_db())
 * - same 0.5dB-native-units reasoning as s_volume_db_x2 above, just
 * unsigned (0-95, matching the register's 0-47.5dB range, no cut
 * direction). Starts at 40 (20.0dB) - the byte-exact captured
 * baseline aic3204_phase2_init() leaves the chip at (0x28), so
 * turning the encoder for the first time doesn't jump the gain. */
static int16_t s_pga_gain_db_x2 = 40;
#define PGA_STEP_X2 2   /* 2 * 0.5dB = 1.0dB per encoder detent */
#define PGA_MIN_X2  0   /* 0.0dB */
#define PGA_MAX_X2  95  /* 47.5dB - see aic3204_set_pga_gain_db()'s field-range note */
/* Backlight step per encoder detent - 5% keeps the full 0-100% range
 * reachable in ~20 detents, coarse enough to actually SEE each step
 * change on the panel while turning the knob (unlike volume's finer
 * 1dB/detent, brightness differences under ~5% are hard to perceive
 * at all - no point spending detents on something invisible). */
#define BACKLIGHT_STEP 5U

/*
 * --- Spectrum/waterfall vertical scale (dB) -----------------------------
 *
 * s_db_min/s_db_max replace what used to be the compile-time
 * constants SDR_DB_MIN/SDR_DB_MAX (still UNCALIBRATED - the "dB"
 * value comes from a bit-manipulation log2 approximation in fft.c,
 * not a referenced measurement - that caveat still applies, only the
 * "compile-time constant" part changed) with live, encoder-adjustable
 * state - 31/07/2026, per the project owner: with the original
 * -10..90dB (100dB) default range, most of a typical signal's
 * dynamic range sits flat near the floor, wasting vertical screen
 * real estate that narrowing the range makes available to the part
 * that actually moves.
 *
 * Reached by tapping the SCALE tile in the settings menu screen
 * (menu_screen_open(), see s_menu_screen's declaration comment - this
 * used to be a MENU-button cycle position, replaced 31/07/2026). In
 * SCALE mode the encoder
 * adjusts EITHER db_min OR db_max - s_scale_adjust_max selects which,
 * TOGGLED BY THE ENCODER BUTTON itself (repurposed here exactly the
 * way it's already repurposed to cycle the tune step in VOLUME/
 * BACKLIGHT mode - see tune_encoder_poll()). SPECTRUM_DB_MIN_GAP
 * stops the two bounds from crossing or collapsing the visible range
 * to something degenerate.
 */
static float s_db_min = 30.0f; /* same starting point as the old SDR_DB_MIN */
static float s_db_max = 90.0f;  /* same starting point as the old SDR_DB_MAX */
static uint8_t s_scale_adjust_max = 0U; /* 0 = knob moves db_min, 1 = moves db_max */
#define SPECTRUM_DB_STEP     2.0f   /* dB per encoder detent */
#define SPECTRUM_DB_FLOOR  (30.0f) /* db_min can't go below this */
#define SPECTRUM_DB_CEIL    120.0f  /* db_max can't go above this */
#define SPECTRUM_DB_MIN_GAP  10.0f  /* db_max - db_min never allowed below this -
                                      * keeps spectrum_draw()'s scale_t = 1/(max-min)
                                      * from blowing up into a useless few-pixel
                                      * sliver of range. */

/*
 * --- Squelch (AM + NFM, encoder target) -------------------------------
 *
 * Reached by tapping the SQUELCH tile in the settings menu screen
 * (menu_screen_open() - this used to be a 4th MENU-button cycle
 * position, replaced 31/07/2026, same as SCALE's - see
 * s_menu_screen's declaration comment). The encoder just adjusts
 * demod_am's threshold directly via
 * demod_am_set_squelch_db()/get - no separate mirrored state
 * needed here, unlike s_db_min/s_db_max (which don't have a demod_am
 * equivalent to read from). The encoder BUTTON does the same "still
 * cycles the tune step" courtesy as VOLUME/BACKLIGHT (see
 * tune_encoder_poll()) - there's no second sub-value to toggle here
 * the way SCALE has LO/HI.
 */
#define SQUELCH_DB_STEP 2.0f /* dB per encoder detent - same granularity as SCALE */
#define SQUELCH_DB_FLOOR (-10.0f) /* matches demod_am.h's "OFF" default exactly - turning
                                     * all the way down returns to today's un-squelched
                                     * NFM behavior, not some arbitrary very-quiet floor. */
#define SQUELCH_DB_CEIL 60.0f /* generous headroom above any realistic in-channel signal
                                * level this metric could read - see demod_am.h's
                                * UNCALIBRATED note; this is a display/encoder-range
                                * bound, not a claim about what's physically meaningful. */

/*
 * --- Spectrum temporal smoothing (encoder target) -----------------------
 *
 * s_spectrum_smooth_alpha used to be a #define local to
 * sdr_spectrum_waterfall_tick() - promoted to file-scope state
 * 31/07/2026 per the project owner, so it can be adjusted live via
 * ENCODER_TARGET_SMOOTH (reached through the settings menu screen's
 * SMOOTH tile, see menu_screen_open()) instead of only at compile
 * time. See sdr_spectrum_waterfall_tick()'s TEMPORAL smoothing comment
 * for the full explanation of what this value does.
 *
 * Presented to the user as a 0-95% "history weight" (see
 * menu_detail_value_redraw()) rather than the raw 0.0-0.95 float -
 * more intuitive than a bare decimal, and matches how the other
 * percent-style controls (BACKLIGHT) already read. SPECTRUM_SMOOTH_MAX
 * stops short of 1.0 deliberately - true 1.0 would freeze the display
 * forever, never blending in a new frame at all.
 */
static float s_spectrum_smooth_alpha = 0.75f; /* same default the #define always used */
#define SPECTRUM_SMOOTH_STEP 0.05f /* 5 percentage points per encoder detent */
#define SPECTRUM_SMOOTH_MIN 0.0f
#define SPECTRUM_SMOOTH_MAX 0.95f

/* TOP BAR readout placement (see the RADIO UI LAYOUT block). The
 * frequency is the star: scale 5 (30px wide x 35px tall per char),
 * 11-char field "XXX.XXX.XXX" = 330px, left-anchored. Mode sits to
 * its right at scale 3; step and volume stack in two scale-2 rows
 * next; time (scale 3) and the battery gauge live over the right
 * status column. */
#define FREQ_TEXT_SCALE 5
#define FREQ_FIELD_CHARS 11
#define FREQ_X 8
#define FREQ_Y 14
#define MODE_X 348
#define MODE_Y 21
#define STEP_X 430
#define STEP_Y 8
#define VOL_X  430
#define VOL_Y  38
#define TIME_X 690
#define TIME_Y 8
#define BATT_X 690
#define BATT_Y 40
#define BATT_W 70 /* narrowed from 80 on 31/07/2026 to make room for the
                    * voltage readout to its right - see
                    * battery_display_draw()'s comment. Screen is 800px
                    * wide (GFX_SCREEN_WIDTH, RCOL_X+RCOL_W=800) and the
                    * icon's right edge sits at BATT_X+BATT_W-1, so this
                    * leaves 799-(690+70-1)=40px for the "XX.XV" text
                    * (30px at scale 1) plus a safety margin. */
#define BATT_H 16

/*
 * Renders `hz` as a fixed 11-char field "XXX.XXX.XXX" with thousands
 * separators, right-aligned, space-padded (e.g. " 90.800.000",
 * "  7.100.000"). Fixed width keeps gfx_text() repainting the whole
 * field every time, so no ghost digits survive a change in magnitude.
 * Manual formatting - same policy as the itoa above, no sprintf.
 * `buf` must hold FREQ_FIELD_CHARS + 1 bytes.
 */
static void tune_freq_format(uint32_t hz, char *buf)
{
    int8_t pos = FREQ_FIELD_CHARS;
    uint8_t digits = 0;

    buf[pos] = '\0';
    do {
        if ((digits > 0U) && ((digits % 3U) == 0U)) {
            buf[--pos] = '.';
        }
        buf[--pos] = (char)('0' + (hz % 10U));
        hz /= 10U;
        digits++;
    } while (hz > 0U && pos > 0);
    while (pos > 0) {
        buf[--pos] = ' ';
    }
}

/*
 * Renders db_x2 (native 0.5dB units) as a fixed 11-char field,
 * right-aligned, same geometry/width as tune_freq_format() so it can
 * reuse the frequency readout's position: e.g. "   +12.0dB",
 * "   -6.5dB", "    +0.0dB". Manual formatting, no sprintf, same
 * convention as tune_freq_format(). `buf` must hold
 * FREQ_FIELD_CHARS + 1 bytes.
 */
static void volume_format(int16_t db_x2, char *buf)
{
    int8_t pos = FREQ_FIELD_CHARS;
    uint16_t whole, tenth;
    uint8_t negative = (db_x2 < 0) ? 1U : 0U;
    uint16_t mag = negative ? (uint16_t)(-(int32_t)db_x2) : (uint16_t)db_x2;

    whole = mag / 2U;
    tenth = (mag % 2U) * 5U; /* 0 or 5 - 0.5dB steps only need one decimal */

    buf[pos] = '\0';
    buf[--pos] = 'B';
    buf[--pos] = 'D'; /* uppercase, not lowercase 'd' - same font-coverage
                        * bug as the "1K"/"96K" fix above (gfx_font5x7 is
                        * 0x20-0x5A only, no lowercase - see gfx_font.h).
                        * This one was hiding in a char-by-char buffer
                        * build, not a string literal, so the earlier grep
                        * for it missed this one. Found 31/07/2026. */
    buf[--pos] = (char)('0' + tenth);
    buf[--pos] = '.';
    do {
        if (pos > 0) {
            buf[--pos] = (char)('0' + (whole % 10U));
        }
        whole /= 10U;
    } while (whole > 0U && pos > 0);
    if (pos > 0) {
        buf[--pos] = negative ? '-' : '+';
    }
    while (pos > 0) {
        buf[--pos] = ' ';
    }
}

/* Same fixed-7-char-field, sign-then-digits-then-unit style as
 * volume_format() above, but for the spectrum SCALE readout: whole
 * dB only (SPECTRUM_DB_STEP is a whole number, no decimal needed),
 * range -40..120 so up to 3 digits. `buf` must hold FREQ_FIELD_CHARS+1
 * bytes, same as volume_format(). */
static void spectrum_db_format(int16_t db_i, char *buf)
{
    int8_t pos = FREQ_FIELD_CHARS;
    uint16_t mag;
    uint8_t negative = (db_i < 0) ? 1U : 0U;

    mag = negative ? (uint16_t)(-(int32_t)db_i) : (uint16_t)db_i;

    buf[pos] = '\0';
    buf[--pos] = 'B';
    buf[--pos] = 'D';
    do {
        if (pos > 0) {
            buf[--pos] = (char)('0' + (mag % 10U));
        }
        mag /= 10U;
    } while (mag > 0U && pos > 0);
    if (pos > 0) {
        buf[--pos] = negative ? '-' : '+';
    }
    while (pos > 0) {
        buf[--pos] = ' ';
    }
}

/*
 * --- Top-bar / right-column readout drawing ---------------------------
 *
 * Each readout has its own draw function, repainting a fixed-width
 * field at a fixed position (no ghost chars). All draw over the
 * DARKGRAY top bar / BLACK right column.
 */
static void mode_display_draw(void)
{
    const char *label;
    uint16_t color;

    switch (demod_am_get_mode()) {
    case DEMOD_MODE_USB: label = "USB"; color = GFX_COLOR_GREEN; break;
    case DEMOD_MODE_LSB: label = "LSB"; color = GFX_COLOR_GREEN; break;
    case DEMOD_MODE_NFM: label = "NFM"; color = GFX_COLOR_ORANGE; break;
    case DEMOD_MODE_WFM: label = "WFM"; color = GFX_COLOR_CYAN; break;
    case DEMOD_MODE_AM:
    default:             label = "AM "; color = GFX_COLOR_YELLOW; break;
    }
    gfx_text((uint16_t)MODE_X, MODE_Y, label, color, GFX_COLOR_DARKGRAY, 3);
}

static void freq_display_draw(void)
{
    char buf[FREQ_FIELD_CHARS + 1];

    tune_freq_format(s_tune_hz, buf);
    gfx_text((uint16_t)FREQ_X, FREQ_Y, buf,
             GFX_COLOR_CYAN, GFX_COLOR_DARKGRAY, FREQ_TEXT_SCALE);
}

static void step_display_draw(void)
{
    gfx_text((uint16_t)STEP_X, STEP_Y, "STEP ",
             GFX_COLOR_GRAY, GFX_COLOR_DARKGRAY, 2);
    gfx_text((uint16_t)(STEP_X + 5 * 6 * 2), STEP_Y, k_tune_step_labels[s_tune_step_idx],
             GFX_COLOR_YELLOW, GFX_COLOR_DARKGRAY, 2);
}

/*
 * Shared "what does the knob control right now" readout at
 * VOL_X/VOL_Y - shows either the volume or the backlight percent
 * depending on s_encoder_target, inverted colors while that target is
 * the active one (same visual language as step_display_draw()'s
 * highlight, just per-target instead of per-button). Renamed
 * 31/07/2026 from volume_display_draw() now that this row shows more
 * than just volume - same call sites, same row.
 *
 * Both branches format into a FIXED 7-char value field (same
 * reasoning as tune_freq_format()'s comment: fixed width means
 * gfx_text() repaints the whole field every time, so switching
 * VOL<->BACKLIGHT never leaves a ghost digit from whichever one was
 * showing before - a 4-char "100%" over a stale 7-char "+00.0DB"
 * would otherwise leave 3 uncleared pixels' worth of the old text).
 */
static void aux_row_display_draw(void)
{
    uint16_t fg, bg;

    if (s_encoder_target == ENCODER_TARGET_BACKLIGHT) {
        char buf[8]; /* 7-char field + NUL, e.g. "   100%" or "    50%" */
        uint8_t pos = 7U;
        uint8_t v = backlight_get_percent();

        fg = GFX_COLOR_BLACK;
        bg = GFX_COLOR_CYAN;

        buf[pos] = '\0';
        buf[--pos] = '%';
        do {
            buf[--pos] = (char)('0' + (v % 10U));
            v /= 10U;
        } while (v > 0U && pos > 0U);
        while (pos > 0U) {
            buf[--pos] = ' ';
        }

        gfx_text((uint16_t)VOL_X, VOL_Y, "BL  ", fg, bg, 2);
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, buf, fg, bg, 2);
    } else if (s_encoder_target == ENCODER_TARGET_SCALE) {
        char buf[FREQ_FIELD_CHARS + 1];
        uint8_t i;

        fg = GFX_COLOR_BLACK;
        bg = GFX_COLOR_CYAN;

        /* Label says which bound the knob currently moves - toggled
         * by the encoder BUTTON, see tune_encoder_poll()'s SCALE
         * branch - "LO " when adjusting db_min, "HI " when adjusting
         * db_max. Both exactly 4 chars, same width as "VOL "/"BL  "
         * above, so no ghosting on the label side either. */
        spectrum_db_format((int16_t)(s_scale_adjust_max ? s_db_max : s_db_min), buf);
        gfx_text((uint16_t)VOL_X, VOL_Y, s_scale_adjust_max ? "HI  " : "LO  ", fg, bg, 2);
        /* Same "last 7 of the fixed 11-char field" trim as the VOL
         * branch below, and for the same reason: a consistent value
         * field width across all four targets. */
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, &buf[i], fg, bg, 2);
    } else if (s_encoder_target == ENCODER_TARGET_SQUELCH) {
        char buf[FREQ_FIELD_CHARS + 1];
        uint8_t i;

        fg = GFX_COLOR_BLACK;
        bg = GFX_COLOR_CYAN;

        /* Same fixed-7-char-field formatter as SCALE above
         * (spectrum_db_format() is generic - any signed integer dB
         * value) - reused rather than duplicated. */
        spectrum_db_format((int16_t)demod_am_get_squelch_db(), buf);
        gfx_text((uint16_t)VOL_X, VOL_Y, "SQL ", fg, bg, 2);
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, &buf[i], fg, bg, 2);
    } else if (s_encoder_target == ENCODER_TARGET_SMOOTH) {
        char buf[8]; /* same 7-char percent field as BACKLIGHT above */
        uint8_t pos = 7U;
        uint8_t v = (uint8_t)(s_spectrum_smooth_alpha * 100.0f + 0.5f);

        fg = GFX_COLOR_BLACK;
        bg = GFX_COLOR_CYAN;

        buf[pos] = '\0';
        buf[--pos] = '%';
        do {
            buf[--pos] = (char)('0' + (v % 10U));
            v /= 10U;
        } while (v > 0U && pos > 0U);
        while (pos > 0U) {
            buf[--pos] = ' ';
        }

        gfx_text((uint16_t)VOL_X, VOL_Y, "SMH ", fg, bg, 2);
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, buf, fg, bg, 2);
    } else if (s_encoder_target == ENCODER_TARGET_PGA) {
        char buf[FREQ_FIELD_CHARS + 1];
        uint8_t i;

        fg = GFX_COLOR_BLACK;
        bg = GFX_COLOR_CYAN;

        /* volume_format() works fine here too - PGA is stored in the
         * same 0.5dB-native-units encoding, just always non-negative
         * in practice, so it'll always show a "+" - that's accurate
         * (PGA gain has no cut direction, see aic3204_set_pga_gain_db()'s
         * comment), not a formatting bug. */
        volume_format(s_pga_gain_db_x2, buf);
        gfx_text((uint16_t)VOL_X, VOL_Y, "PGA ", fg, bg, 2);
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, &buf[i], fg, bg, 2);
    } else {
        char buf[FREQ_FIELD_CHARS + 1];
        uint8_t i;

        fg = (s_encoder_target == ENCODER_TARGET_VOLUME) ? GFX_COLOR_BLACK : GFX_COLOR_GRAY;
        bg = (s_encoder_target == ENCODER_TARGET_VOLUME) ? GFX_COLOR_CYAN : GFX_COLOR_DARKGRAY;

        volume_format(s_volume_db_x2, buf);
        gfx_text((uint16_t)VOL_X, VOL_Y, "VOL ", fg, bg, 2);
        /* volume_format right-aligns into 11 chars; show only the last
         * 7 ("+00.0DB") to keep the row compact and the SAME fixed
         * width as the BACKLIGHT branch above. */
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        gfx_text((uint16_t)(VOL_X + 4 * 6 * 2), VOL_Y, &buf[i], fg, bg, 2);
    }
}

/*
 * Time-of-day slot. There is NO RTC configured in this project (and
 * no battery-backed clock domain has been brought up), so this shows
 * UPTIME since power-on (from g_msticks, the 1ms SysTick counter) in
 * HH:MM form - honest and still useful on a bench. When/if the
 * GD32F450's RTC gets configured (needs LXTAL bring-up + calendar
 * init), swap the source here and nothing else changes.
 */
static void time_display_draw(void)
{
    extern volatile uint32_t g_msticks;
    uint32_t total_min = g_msticks / 60000UL;
    uint32_t hh = (total_min / 60UL) % 100UL;
    uint32_t mm = total_min % 60UL;
    char buf[6];

    buf[0] = (char)('0' + (hh / 10UL));
    buf[1] = (char)('0' + (hh % 10UL));
    buf[2] = ':';
    buf[3] = (char)('0' + (mm / 10UL));
    buf[4] = (char)('0' + (mm % 10UL));
    buf[5] = '\0';
    gfx_text((uint16_t)TIME_X, TIME_Y, buf,
             GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, 3);
}

/*
 * Battery gauge, backed by VBAT/ADC0 channel 18 (see battery.h/.c) -
 * 31/07/2026, replacing the earlier "no ADC channel identified"
 * placeholder now that the VBAT pin + diode-drop compensation is
 * known. battery_get_percent() never returns >100, so the "--"
 * unknown-placeholder path below is now dead in practice - left in
 * place rather than deleted, in case battery_init() ever needs a
 * "not wired / read failed" escape hatch again (e.g. a future board
 * revision that removes the divider).
 */
static uint8_t radio_battery_percent(void)
{
    return battery_get_percent();
}

/* Formats millivolts as a fixed 5-char "XX.XV" field (e.g. " 3.9V",
 * "12.4V") - always the same width regardless of magnitude, so
 * gfx_text() fully repaints it every refresh with no ghost digits
 * (same fixed-width reasoning as tune_freq_format()/volume_format()).
 * Deliberately simpler than those two: no sign, and the whole-volt
 * part is capped at 2 digits (99V) - plenty for any battery pack this
 * board is realistically going to have - so straight positional
 * indexing reads clearer here than their right-to-left digit-peeling
 * loops. `buf` must hold 6 bytes. */
static void battery_voltage_format(uint16_t mv, char *buf)
{
    uint16_t whole = mv / 1000U;
    uint16_t tenth = (mv % 1000U) / 100U; /* one decimal digit */

    if (whole > 99U) {
        whole = 99U; /* clamp - a display cap, not a real limit */
    }

    buf[0] = (whole >= 10U) ? (char)('0' + (whole / 10U)) : ' ';
    buf[1] = (char)('0' + (whole % 10U));
    buf[2] = '.';
    buf[3] = (char)('0' + tenth);
    buf[4] = 'V';
    buf[5] = '\0';
}

/*
 * Battery gauge + voltage readout. The voltage text sits just right
 * of the icon (BATT_W was narrowed to make room - see its comment)
 * and shares the SAME green/orange/red threshold color as the fill
 * bar, so a glance at the color alone still tells the story even
 * before reading the number. Uses battery_get_millivolts() directly
 * (not the already-floor/ceiling-clamped percent) since the raw
 * voltage is the more useful number to actually read off, and it's
 * a second, independent ADC conversion each redraw rather than a
 * derived value - harmless, this is a housekeeping readout polled a
 * few times a second at most, not the demod ISR.
 */
static void battery_display_draw(void)
{
    uint8_t pct = radio_battery_percent();
    uint16_t color = GFX_COLOR_GRAY;

    /* body + nub */
    gfx_rect(BATT_X, BATT_Y, (uint16_t)(BATT_W - 4), BATT_H, GFX_COLOR_WHITE);
    gfx_fill_rect((uint16_t)(BATT_X + BATT_W - 4), (uint16_t)(BATT_Y + 4),
                   3, (uint16_t)(BATT_H - 8), GFX_COLOR_WHITE);

    if (pct <= 100U) {
        uint16_t fill_w = (uint16_t)(((uint32_t)(BATT_W - 8) * pct) / 100UL);

        color = (pct > 50U) ? GFX_COLOR_GREEN
              : (pct > 20U) ? GFX_COLOR_ORANGE : GFX_COLOR_RED;
        gfx_fill_rect((uint16_t)(BATT_X + 2), (uint16_t)(BATT_Y + 2),
                       (uint16_t)(BATT_W - 8), (uint16_t)(BATT_H - 4), GFX_COLOR_DARKGRAY);
        gfx_fill_rect((uint16_t)(BATT_X + 2), (uint16_t)(BATT_Y + 2),
                       fill_w, (uint16_t)(BATT_H - 4), color);
    } else {
        gfx_text((uint16_t)(BATT_X + BATT_W / 2 - 12), (uint16_t)(BATT_Y + 1), "--",
                 GFX_COLOR_GRAY, GFX_COLOR_DARKGRAY, 2);
    }

    {
        char vbuf[6];

        battery_voltage_format(battery_get_millivolts(), vbuf);
        /* Vertically centered against the 16px-tall icon: scale-1 text
         * is 7px tall, (16-7)/2 = 4 (rounds down, close enough). */
        gfx_text((uint16_t)(BATT_X + BATT_W + 2), (uint16_t)(BATT_Y + 4), vbuf,
                 color, GFX_COLOR_DARKGRAY, 1);
    }
}

/*
 * --- Right column: S-meter + status badges ----------------------------
 *
 * S-METER: 12 segments driven by the demodulator's pre-AGC envelope
 * peak (demod_am_get_signal_peak() - instant attack / slow release,
 * proper meter ballistics for free). Mapped from dBFS, NOT calibrated
 * S-units: without a known antenna/frontend gain figure, painting
 * "S9" on it would be an invented number. Segment thresholds span
 * -90..-6dBFS (7dB/segment); the last 3 segments draw red like the
 * classic "over S9" zone. Good relative meter now; calibration (an
 * offset + label change here) can come once a signal generator has
 * been on the antenna jack.
 */
#define SMETER_X    (RCOL_X + 6)
#define SMETER_Y    (RCOL_Y + 22)
#define SMETER_SEGS 12
#define SMETER_SEG_W 8
#define SMETER_SEG_H 20

static uint8_t s_smeter_segs_last = 0xFFU; /* force first draw */

static void smeter_draw(uint8_t segs)
{
    uint8_t i;

    if (segs == s_smeter_segs_last) {
        return; /* nothing changed - skip the blits entirely */
    }
    s_smeter_segs_last = segs;

    for (i = 0; i < SMETER_SEGS; i++) {
        uint16_t x = (uint16_t)(SMETER_X + i * (SMETER_SEG_W + 1));
        uint16_t on_color = (i >= 9U) ? GFX_COLOR_RED : GFX_COLOR_GREEN;
        gfx_fill_rect(x, SMETER_Y, SMETER_SEG_W, SMETER_SEG_H,
                       (i < segs) ? on_color : GFX_COLOR_DARKGRAY);
    }
}

/* Convert the demod's peak (int16 scale) to lit segments. Uses the
 * same IEEE754 bit-trick log2 approximation as fft.c instead of
 * libm's log10f: this project links without syscall stubs and
 * newlib's log10f drags in __errno (link error) - and a few percent
 * of log error is invisible on a 12-segment meter anyway. Main loop
 * only - never the ISR. */
static float smeter_log2_approx(float x)
{
    union { float f; uint32_t u; } v;
    float y;

    v.f = x;
    y = (float)v.u;
    y *= 1.1920928955078125e-7f; /* 1/2^23 */
    return y - 126.94269504f;
}

static uint8_t smeter_segments_from_peak(float peak)
{
    float dbfs;
    int32_t segs;

    if (peak < 1.0f) {
        peak = 1.0f;
    }
    /* 20*log10(x) = 6.0206*log2(x); dBFS relative to int16 full scale. */
    dbfs = 6.0206f * (smeter_log2_approx(peak) - smeter_log2_approx(32767.0f));
    /* -90dBFS -> 0 segs, 7dB per segment, saturating at 12. */
    segs = (int32_t)((dbfs + 90.0f) / 7.0f);
    if (segs < 0)  { segs = 0; }
    if (segs > SMETER_SEGS) { segs = SMETER_SEGS; }
    return (uint8_t)segs;
}

/*
 * STATUS BADGES: up to 6, in a 2x3 grid under the S-meter. Each shows
 * a radio state at a glance:
 *   NR / SPT  - NR is now VESTIGIAL (permanently off, see s_nr_on's
 *               comment - its button was repurposed to the AGC
 *               profile, see agc_profile_cycle()). SPT lights up
 *               whenever the spectrum's spatial line smoothing is
 *               active (passes > 0) - see s_spec_smooth_passes'
 *               comment; it used to be the NB (noise blanker) badge,
 *               which never drove any real DSP.
 *   AGC       - lit: the demod AGC really is always active (even in
 *               MANUAL profile the loop still runs, it just skips the
 *               peak-tracking math - see agc_profile_t's MANUAL note
 *               in demod_am.h).
 *   [profile] - s_btn_agc_profile, a REAL touchable button (not a
 *               plain badge_draw() call) showing MAN/SLW/MED/FST -
 *               tap it to cycle, see agc_profile_button_callback()
 *               below. Moved here from where BW used to live
 *               31/07/2026, per the project owner: this slot gets
 *               more use as an interactive control than BW did as a
 *               static readout.
 *   BW        - now a REAL touchable button too (s_btn_audio_bw, added
 *               02/08/2026, same "not a plain badge_draw() call"
 *               treatment as [profile] above): in AM/USB/LSB it shows
 *               the active audio-filter width and tapping it cycles
 *               4K0 -> 2K3 -> 1K8 -> 4K0 (see audio_bw_cycle() and
 *               demod_am_set_audio_bw()'s comment in demod_am.h). In
 *               NFM/WFM it goes back to being purely informative
 *               (channel-filter -3dB width, "6K3"/"96K") and tapping
 *               it does nothing - those modes have their own fixed
 *               filters, unrelated to this selector, see
 *               audio_bw_button_callback()'s comment.
 *   PRE       - preamp: no such hardware control has been identified
 *               on this board, shown dark (off) as a placeholder for
 *               the layout. ATT (attenuator, same story) was dropped
 *               from this grid to make room for BW's move - if a real
 *               attenuator control ever gets added, it'll need a new
 *               home rather than reclaiming this exact slot.
 */
#define BADGE_W 55
#define BADGE_H 26
#define BADGE_X0 (RCOL_X + 4)
#define BADGE_X1 (RCOL_X + 4 + BADGE_W + 4)
#define BADGE_Y0 (RCOL_Y + 60)
#define BADGE_ROW_STEP (BADGE_H + 6)

/* Indexed directly by agc_profile_t (demod_am.h) - MANUAL, SLOW,
 * MEDIUM, FAST in that order. Matches what the project owner asked
 * for verbatim: "MAN, SLW, MED, FST". */
static const char *k_agc_profile_labels[4] = { "MAN", "SLW", "MED", "FST" };

/* Indexed directly by audio_bw_t (demod_am.h) - AUDIO_BW_4K0,
 * AUDIO_BW_2K3, AUDIO_BW_1K8 in that order. */
static const char *k_audio_bw_labels[3] = { "4K0", "2K3", "1K8" };
/* Same indexing, in Hz - the nominal -3dB corner each ALPF_*_COEFFS
 * table was designed for (see their comments in demod_am.c). Added
 * 03/08/2026 for the spectrum panadapter's demodulated-bandwidth tint
 * (see sdr_spectrum_waterfall_tick()'s call to spectrum_draw()) - the
 * labels above are for display only and aren't parseable back into a
 * number, so this is a second small table rather than deriving one
 * from the other. */
static const uint32_t k_audio_bw_hz[3] = { 4000UL, 2300UL, 1800UL };

/*
 * Cycles MANUAL -> SLOW -> MEDIUM -> FAST -> MANUAL and redraws
 * s_btn_agc_profile - shared by BOTH ways to trigger this now:
 * tapping the badge/button itself (agc_profile_button_callback()
 * below) and the NR bottom-bar button (see demo_button_callback()'s
 * NR branch, repurposed 31/07/2026 - a quick, deliberately temporary
 * stopgap since NR's noise-reduction toggle was never wired to real
 * DSP anyway, ahead of a proper settings-menu redesign the project
 * owner is planning for later the same day, once the badge grid ran
 * out of comfortable room for more controls). Factored out so both
 * entry points can't drift out of sync with each other.
 */
static void agc_profile_cycle(void)
{
    agc_profile_t p = demod_am_get_agc_profile();

    switch (p) {
    case AGC_PROFILE_MANUAL: p = AGC_PROFILE_SLOW;   break;
    case AGC_PROFILE_SLOW:   p = AGC_PROFILE_MEDIUM; break;
    case AGC_PROFILE_MEDIUM: p = AGC_PROFILE_FAST;   break;
    case AGC_PROFILE_FAST:
    default:                  p = AGC_PROFILE_MANUAL; break;
    }
    demod_am_set_agc_profile(p);
    debug_print("agc: profile now ");
    debug_print(k_agc_profile_labels[(uint8_t)p]);
    debug_print("\n");

    s_btn_agc_profile.label = k_agc_profile_labels[(uint8_t)p];
    ui_button_draw(&s_btn_agc_profile);
}

/* Unlike the bottom-bar buttons (which toggle or jump straight to a
 * target), this only ever needs to STEP forward one at a time - four
 * short taps to get anywhere, and there's no "off" state worth
 * jumping straight to the way VOL/MENU jump straight to their
 * targets. */
static void agc_profile_button_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;

    if (event == UI_EVENT_RELEASE) {
        agc_profile_cycle();
    }
}

static void badge_draw(uint16_t x, uint16_t y, const char *label,
                        uint8_t on, uint16_t on_bg)
{
    uint16_t bg = on ? on_bg : GFX_COLOR_DARKGRAY;
    uint16_t fg = on ? GFX_COLOR_BLACK : GFX_COLOR_GRAY;
    uint16_t tw = gfx_text_width(label, 2);

    gfx_fill_rect(x, y, BADGE_W, BADGE_H, bg);
    gfx_rect(x, y, BADGE_W, BADGE_H, GFX_COLOR_GRAY);
    gfx_text((uint16_t)(x + (BADGE_W - tw) / 2), (uint16_t)(y + 6), label, fg, bg, 2);
}

static void badges_draw(void)
{
    /* BW's label: NFM/WFM show their own ACTUAL channel-filter -3dB
     * corner (see demod_am.c's NFM_CHF_COEFFS comment; WFM has no
     * channel filter at all - see demod_am.h's WFM note - "96K" here
     * means the full +/-96kHz complex Nyquist bandwidth it uses
     * instead, not a -3dB corner, so it's not quite the same kind of
     * number as the others, but it's the closest honest one-badge
     * answer to "how wide is this mode listening"). AM/USB/LSB instead
     * show the currently SELECTED audio filter width (see
     * demod_am_set_audio_bw()'s comment in demod_am.h) - this is the
     * one case where the badge is genuinely live/user-controlled, not
     * just informative. */
    demod_mode_t mode = demod_am_get_mode();
    const char *bw_label;
    uint8_t bw_interactive;

    switch (mode) {
    case DEMOD_MODE_NFM: bw_label = "6K3"; bw_interactive = 0U; break; /* NFM_CHF_COEFFS, ~6.25kHz */
    case DEMOD_MODE_WFM: bw_label = "96K"; bw_interactive = 0U; break; /* no channel filter - full Nyquist */
    default:              bw_label = k_audio_bw_labels[(uint8_t)demod_am_get_audio_bw()];
                           bw_interactive = 1U; break;
    }

    badge_draw(BADGE_X0, BADGE_Y0,                      "NR",  s_nr_on, GFX_COLOR_GREEN);
    /* SPT badge: lit whenever line smoothing is active (passes > 0),
     * same "on = colored" convention as the other badges - see
     * s_spec_smooth_passes' comment for the repurposing story. The
     * actual pass count only shows on the menu tile (badge_draw() has
     * no room for "SPT 3" at this width). */
    badge_draw(BADGE_X1, BADGE_Y0,                      "SPT", (uint8_t)(s_spec_smooth_passes > 0U), GFX_COLOR_GREEN);
    badge_draw(BADGE_X0, (uint16_t)(BADGE_Y0 + BADGE_ROW_STEP),     "AGC", 1U, GFX_COLOR_GREEN);
    /* s_btn_agc_profile is a real ui_button_t (see its declaration),
     * not a badge_draw() call - keep its label in sync with the
     * current profile and redraw it here too, so it stays correct
     * even if badges_draw() is called from somewhere other than the
     * button's own callback (e.g. after a MODE change). */
    s_btn_agc_profile.label = k_agc_profile_labels[(uint8_t)demod_am_get_agc_profile()];
    ui_button_draw(&s_btn_agc_profile);
    /* s_btn_audio_bw: same "real ui_button_t, kept in sync here" deal
     * as s_btn_agc_profile above - see its declaration and
     * audio_bw_button_callback()'s comment. bw_interactive only
     * changes the VISUAL "is this live" cue (cyan when it does
     * something, dimmer gray otherwise, mirroring badge_draw()'s own
     * on/off convention) - the button stays touchable either way,
     * audio_bw_button_callback() itself is what actually gates whether
     * tapping it does anything in NFM/WFM. */
    s_btn_audio_bw.label = bw_label;
    s_btn_audio_bw.bg = bw_interactive ? GFX_COLOR_CYAN : GFX_COLOR_DARKGRAY;
    s_btn_audio_bw.fg = bw_interactive ? GFX_COLOR_BLACK : GFX_COLOR_GRAY;
    ui_button_draw(&s_btn_audio_bw);
    badge_draw(BADGE_X1, (uint16_t)(BADGE_Y0 + 2 * BADGE_ROW_STEP), "PRE", 0U, GFX_COLOR_GREEN);
}

/*
 * --- Settings menu screen: tile geometry + callbacks --------------------
 *
 * CONFINED to the spectrum+waterfall area (MENU_AREA_*) rather than
 * the whole 800x480 screen - changed 31/07/2026 per the project
 * owner: the top bar (frequency/mode/time/battery) and the right
 * column (S-meter/badges) and the bottom button bar all stay live and
 * visible while the menu is open, only the graph itself gets replaced
 * by the tile grid. See s_menu_screen's declaration comment for how
 * touch routing and the still-running S-meter/time updates work with
 * two screens effectively active in different SCREEN REGIONS at once.
 *
 * Grid: 4 columns x 3 rows, 159x108px tiles, 8px gaps, inside
 * MENU_AREA (676x358, starting at (0,SPEC_Y)).
 *
 * REDESIGNED 03/08/2026, per the project owner, from one flat 4x3
 * grid of 12 interchangeable tiles into a PAGED layout:
 *
 *   - Column 0 (all 3 rows): a fixed PAGE SELECTOR - RADIO / UI / HW,
 *     one tile per row, always present and never changing. These
 *     tiles look visually DISTINCT from the option tiles beside them
 *     (ORANGE-accented instead of the CYAN/DARKGRAY/YELLOW palette
 *     the option tiles use - see menu_page_select_callback()'s
 *     styling) precisely so "this switches pages" reads differently
 *     at a glance from "this is a page's option". The currently
 *     selected page is solid ORANGE-on-BLACK; the other two are
 *     outlined (BLACK-on-BLACK fill, ORANGE text/border).
 *   - Columns 1-3 (all 3 rows = 9 slots): that page's OPTIONS, laid
 *     out row-major (slot 0 = row0/col1, slot 1 = row0/col2, ...,
 *     slot 7 = row2/col2). Slot 8 (row2/col3, bottom-right) is
 *     ALWAYS EXIT, regardless of which page is selected - it closes
 *     the whole menu (menu_screen_close()), same as the
 *     long-press-the-knob gesture does from anywhere (see
 *     tune_encoder_poll()'s comment). A page that doesn't fill all 8
 *     option slots just leaves the rest empty (black) - HW starts
 *     with zero tiles today, deliberately, for future additions.
 *
 * Per-page option assignment (all pre-existing tiles, just
 * relocated - no settings were dropped):
 *   RADIO (slots 0-4): AGC, SQL (squelch), VOL, BW, PGA.
 *   UI    (slots 0-5): BL (backlight), SCALE, SPT, SMH (smooth),
 *                       SPC (spectrum trace style, HEATMAP<->LINE),
 *                       ZOOM.
 *   HW    (slot 0):    SPK - speaker PA enable/mute (PB7, see
 *                       speaker_pa_set_enabled()'s comment - pin/
 *                       polarity UNCONFIRMED as of 03/08/2026).
 *                       Slots 1-7 reserved for future hardware
 *                       settings.
 *
 * Tile behavior is unchanged from before the redesign, just
 * regrouped by page:
 *   - AGC, SPT, SPC, BW, and ZOOM CYCLE/TOGGLE DIRECTLY on tap (same
 *     as their existing bottom-bar-button/badge equivalents where
 *     they have one) and stay on this screen - you can tap several of
 *     these in a row.
 *   - SQL/BL/SCALE/VOL/SMH/PGA instead open a DETAIL view
 *     (menu_detail_show()) for that target - see its own comment.
 *
 * menu_grid_show() itself always redraws the WHOLE MENU_AREA (page
 * column + options + EXIT) in one pass, including on a page switch -
 * simplest correct thing (no separate "clear just the options area"
 * path to keep in sync), and cheap enough for a menu tap.
 */
#define MENU_AREA_X 0
#define MENU_AREA_W MAIN_W
#define MENU_AREA_Y SPEC_Y
#define MENU_AREA_H (uint16_t)(WF_PANEL_Y + WATERFALL_ROWS + 4U - SPEC_Y) /* 422-64=358 */

#define MENU_TILE_W 159
#define MENU_TILE_H 108
#define MENU_TILE_GAP 8
#define MENU_TILE_X0 (uint16_t)(MENU_AREA_X + 8)
#define MENU_TILE_Y0 (uint16_t)(MENU_AREA_Y + 8)
#define MENU_TILE_COL(i) (uint16_t)(MENU_TILE_X0 + (i) * (MENU_TILE_W + MENU_TILE_GAP))
#define MENU_TILE_ROW(i) (uint16_t)(MENU_TILE_Y0 + (i) * (MENU_TILE_H + MENU_TILE_GAP))

/*
 * Option-slot geometry helpers: slot 0-8 -> (row, col) within the
 * RIGHT-hand 3x3 area (columns 1-3, since column 0 is the page
 * selector). Row-major: slot / 3 = row, slot % 3 = column-within-3,
 * offset by +1 to skip the page-selector column. Slot 8 always lands
 * on row 2 / column 3 (bottom-right) - the fixed EXIT position - see
 * this block's comment above for why that's deliberate, not a
 * coincidence of the arithmetic.
 */
#define MENU_OPT_COL(slot) MENU_TILE_COL(1 + ((slot) % 3))
#define MENU_OPT_ROW(slot) MENU_TILE_ROW((slot) / 3)
#define MENU_OPT_EXIT_SLOT 8

/*
 * --- Spectrum/waterfall ZOOM --------------------------------------------
 *
 * Added 31/07/2026, per the project owner - NOT a codec/sample-rate
 * change (see the earlier discussion on why that's both risky, with
 * no reference capture to replay, and not actually what was needed).
 * This is a purely DIGITAL zoom: the codec/ADC/I2S keep running at
 * 192kHz exactly as always, untouched.
 *
 * How it works: cascade 1-3 stages of a generic decimate-by-2 FIR
 * (ZOOM_DECIM2_COEFFS, see its own comment) on a COPY of the raw I/Q,
 * re-centered on the actual tuned frequency first (same delay-free
 * sign-flip rotation demod_am.c's low-IF down-mix uses, applied here
 * only when demod_am_get_if_offset_active() is set - see
 * zoom_process_block()). Enough decimated samples accumulate across
 * however many raw 192kHz blocks it takes to fill one FFT_SIZE window,
 * then that window feeds the SAME fft_compute_db_iq() the unzoomed
 * view already uses - no change to the FFT itself, just what feeds it.
 *
 *   SPEC_ZOOM_1X - unchanged existing behavior: FFT runs directly on
 *                  the raw 192kHz block, every block, +/-96kHz span.
 *                  ZERO extra cost - the whole zoom pipeline below is
 *                  skipped entirely at this setting.
 *   SPEC_ZOOM_2X - one decimate-by-2 stage, +/-48kHz span. Needs 2 raw
 *                  blocks (~5.3ms) per FFT window.
 *   SPEC_ZOOM_4X - two cascaded stages, +/-24kHz span, 4 raw blocks
 *                  (~10.7ms) per window.
 *   SPEC_ZOOM_8X - three cascaded stages, +/-12kHz span, 8 raw blocks
 *                  (~21.3ms) per window.
 *
 * REFRESH RATE TRADEOFF, inherent to any zoom-FFT (not a bug to fix):
 * higher zoom means each window takes longer to fill, so the spectrum/
 * waterfall update less often - see sdr_spectrum_waterfall_tick()'s
 * comment for exactly how that's handled (skip drawing, don't block,
 * when a window isn't ready yet).
 */
typedef enum {
    SPEC_ZOOM_1X = 0,
    SPEC_ZOOM_2X,
    SPEC_ZOOM_4X,
    SPEC_ZOOM_8X
} spec_zoom_t;

static spec_zoom_t s_spec_zoom = SPEC_ZOOM_1X;

/*
 * Panadapter frequency scale under the spectrum trace - 5 reference
 * points (both edges, both quarter-points, and center), each showing
 * the ACTUAL absolute frequency at that point, TRUNCATED to kHz (last
 * 3 digits dropped - "quitando los 3 ultimos digitos", per the
 * project owner, once 3 points at full Hz precision didn't leave room
 * for more of them). Same grouped-thousands look as
 * freq_display_draw()'s title-bar readout (via tune_freq_format() -
 * already defined above, reused here on the truncated kHz value
 * rather than the raw Hz one - e.g. 180096345 Hz -> 180096 -> shown as
 * "180.096", not "180.096.345").
 *
 * PANEL-CENTER FREQUENCY - the one thing this function has to get
 * right that a naive "s_tune_hz +/- half_span" wouldn't: the pixel
 * grid's true center is NOT always s_tune_hz. Whenever the low-IF
 * down-mix is active (demod_am_get_if_offset_active(), AM/USB/LSB/NFM
 * at 1X zoom - see demod_am.h's LOW-IF TUNING note), the LO itself
 * sits DEMOD_IF_OFFSET_HZ BELOW the selected/displayed station, and
 * the FFT (hence this whole panel) is centered on the LO, not the
 * station - s_tune_hz actually shows up center_mark_offset_px pixels
 * to the right of center (see sdr_spectrum_waterfall_tick()'s comment
 * and the red marker it draws there). Under ZOOM, on the other hand,
 * zoom_process_block() re-centers on s_tune_hz BEFORE decimating, so
 * the panel center genuinely IS s_tune_hz there. panel_center_hz
 * below picks the right one of those two, the EXACT same condition
 * center_mark_offset_px uses - so this scale, the red center marker,
 * and the demodulated-bandwidth tint all agree on which frequency
 * sits at which pixel. Getting this wrong would silently mislabel
 * every non-WFM reading by 48kHz at 1X zoom - worth the extra
 * conditional to avoid.
 *
 * The two edges are the exact +/-half_span_hz boundary of whatever
 * the CURRENT zoom shows (96/48/24/12kHz - see the switch below), so
 * they track ZOOM automatically, same as the marker/tint do.
 * panel_center_hz +/- half_span_hz can, in principle, run outside
 * TUNE_MIN_HZ/MAX_HZ (e.g. tuned near the 100kHz floor, minus 96kHz
 * of span) - int64_t math + a floor-at-0 clamp keeps that from
 * wrapping a uint32_t negative into a huge bogus frequency; showing
 * an edge label below the tunable floor is harmless (there's no
 * corresponding "real" edge case to get wrong here, it's just a
 * label), unlike clamping the LO itself would be.
 *
 * MUST be redrawn on every actual frequency change, not just at
 * init/menu-close - "esta escala tiene que variar con la frecuencia".
 * Call sites: radio_screen_draw() (initial paint), menu_screen_close()
 * (also covers band-preset/mode changes applied from the menu, which
 * both close it on their way out - see menu_band_preset_callback()'s
 * and menu_mode_preset_callback()'s comments), tune_encoder_poll()'s
 * TUNE branch (knob retune) and spec_drag_tune_apply() (touch drag-
 * to-tune) - the two places s_tune_hz changes WITHOUT going through
 * the menu. Cheap enough (a handful of small draws, same cost class
 * as freq_display_draw() it's always paired with) to just redraw on
 * every change rather than diffing against the last-drawn value.
 */
#define SPEC_SCALE_TEXT_SIZE 2 /* was 1 - bumped 03/08/2026, per the
                                 * project owner: "casi no se ve" */

static void spec_span_labels_draw(void)
{
    uint32_t full_span_hz;
    int32_t half_span_hz;
    uint32_t panel_center_hz;
    uint8_t i;

    switch (s_spec_zoom) {
    case SPEC_ZOOM_2X: full_span_hz = 96000UL;  break;
    case SPEC_ZOOM_4X: full_span_hz = 48000UL;  break;
    case SPEC_ZOOM_8X: full_span_hz = 24000UL;  break;
    case SPEC_ZOOM_1X:
    default:           full_span_hz = 192000UL; break;
    }
    half_span_hz = (int32_t)(full_span_hz / 2U);

    /* See this function's PANEL-CENTER FREQUENCY comment above - same
     * condition sdr_spectrum_waterfall_tick() uses for
     * center_mark_offset_px. s_tune_hz > DEMOD_IF_OFFSET_HZ always
     * holds here (TUNE_MIN_HZ=100kHz > DEMOD_IF_OFFSET_HZ=48kHz), so
     * the subtraction below never underflows. */
    panel_center_hz = s_tune_hz;
    if (s_spec_zoom == SPEC_ZOOM_1X && demod_am_get_if_offset_active()) {
        panel_center_hz = s_tune_hz - DEMOD_IF_OFFSET_HZ;
    }

    /* Clear the whole scale strip (ticks + labels) in one go, then
     * the horizontal baseline the ticks hang from. Taller than the
     * old single-height-1 row to fit SPEC_SCALE_TEXT_SIZE's bigger
     * glyphs. */
    gfx_fill_rect(0, (uint16_t)(SPEC_Y + SPEC_H - 24), MAIN_W, 24, GFX_COLOR_BLACK);
    gfx_line(1, (uint16_t)(SPEC_Y + SPEC_H - 20),
              (uint16_t)(MAIN_W - 2), (uint16_t)(SPEC_Y + SPEC_H - 20),
              GFX_COLOR_DARKGRAY);

    for (i = 0; i < 5U; i++) {
        /* i=0..4 -> k=-2..+2 -> off_hz = k * half_span_hz/2, i.e. left
         * edge, left-quarter, center, right-quarter, right edge -
         * evenly spaced in Hz (and therefore in pixels too, since the
         * Hz->px mapping is linear). half_span_hz/2 is always exact
         * for every full_span_hz above (96000/48000/24000/12000 all
         * divide cleanly by 4 total), so no rounding to worry about. */
        int32_t k = (int32_t)i - 2;
        int32_t off_hz = k * (half_span_hz / 2);
        int32_t px = (int32_t)(((int64_t)SPEC_TRACE_W * off_hz) / (int64_t)full_span_hz);
        uint16_t cx = (uint16_t)((int32_t)(MAIN_W / 2) + px);
        int64_t freq_i64 = (int64_t)panel_center_hz + (int64_t)off_hz;
        int64_t khz_i64;
        char buf[FREQ_FIELD_CHARS + 1];
        const char *label;
        uint16_t label_color = (k == 0) ? GFX_COLOR_RED : GFX_COLOR_WHITE;
        uint16_t tw;
        int32_t tx;
        uint8_t s;

        gfx_line(cx, (uint16_t)(SPEC_Y + SPEC_H - 24),
                  cx, (uint16_t)(SPEC_Y + SPEC_H - 20), GFX_COLOR_DARKGRAY);

        if (freq_i64 < 0) { freq_i64 = 0; } /* see this function's
                                              * comment - a label-only
                                              * clamp, not a tuning
                                              * limit */
        khz_i64 = freq_i64 / 1000; /* drop the last 3 digits */
        tune_freq_format((uint32_t)khz_i64, buf);
        /* Skip the leading space-padding tune_freq_format() adds for
         * its fixed-width title-bar use - here each label is
         * individually centered, so the field doesn't need to stay a
         * constant width the way the live readout does. */
        for (s = 0; buf[s] == ' ' && s < FREQ_FIELD_CHARS; s++) { }
        label = &buf[s];

        tw = gfx_text_width(label, SPEC_SCALE_TEXT_SIZE);
        tx = (int32_t)cx - (int32_t)(tw / 2U);
        if (tx < 0) { tx = 0; }
        if (tx > (int32_t)(MAIN_W - tw)) { tx = (int32_t)(MAIN_W - tw); }
        gfx_text((uint16_t)tx, (uint16_t)(SPEC_Y + SPEC_H - 18), label,
                  label_color, GFX_COLOR_BLACK, SPEC_SCALE_TEXT_SIZE);
    }
}

static void menu_tile_agc_refresh(void)
{
    const char *p = k_agc_profile_labels[(uint8_t)demod_am_get_agc_profile()];

    s_menu_tile_agc_buf[0] = 'A'; s_menu_tile_agc_buf[1] = 'G';
    s_menu_tile_agc_buf[2] = 'C'; s_menu_tile_agc_buf[3] = ' ';
    s_menu_tile_agc_buf[4] = p[0]; s_menu_tile_agc_buf[5] = p[1];
    s_menu_tile_agc_buf[6] = p[2]; s_menu_tile_agc_buf[7] = '\0';
    s_menu_tile_agc.label = s_menu_tile_agc_buf;
    ui_button_draw(&s_menu_tile_agc);
}

static void menu_tile_squelch_refresh(void)
{
    char db[FREQ_FIELD_CHARS + 1];
    uint8_t i;

    spectrum_db_format((int16_t)demod_am_get_squelch_db(), db);
    for (i = 0; db[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
    s_menu_tile_squelch_buf[0] = 'S'; s_menu_tile_squelch_buf[1] = 'Q';
    s_menu_tile_squelch_buf[2] = 'L'; s_menu_tile_squelch_buf[3] = ' ';
    {
        uint8_t j = 4U;
        uint8_t k;
        for (k = i; db[k] != '\0' && j < 15U; k++, j++) {
            s_menu_tile_squelch_buf[j] = db[k];
        }
        s_menu_tile_squelch_buf[j] = '\0';
    }
    s_menu_tile_squelch.label = s_menu_tile_squelch_buf;
    ui_button_draw(&s_menu_tile_squelch);
}

static void menu_tile_backlight_refresh(void)
{
    uint8_t pct = backlight_get_percent();
    char digits[4];
    uint8_t dpos = 3U;
    uint8_t i, j;

    digits[dpos] = '\0';
    do {
        digits[--dpos] = (char)('0' + (pct % 10U));
        pct /= 10U;
    } while (pct > 0U && dpos > 0U);

    s_menu_tile_backlight_buf[0] = 'B';
    s_menu_tile_backlight_buf[1] = 'L';
    s_menu_tile_backlight_buf[2] = ' ';
    j = 3U;
    for (i = dpos; digits[i] != '\0'; i++) {
        s_menu_tile_backlight_buf[j++] = digits[i];
    }
    s_menu_tile_backlight_buf[j++] = '%';
    s_menu_tile_backlight_buf[j] = '\0';

    s_menu_tile_backlight.label = s_menu_tile_backlight_buf;
    ui_button_draw(&s_menu_tile_backlight);
}

static void menu_tile_volume_refresh(void)
{
    char db[FREQ_FIELD_CHARS + 1];
    uint8_t i;

    volume_format(s_volume_db_x2, db);
    for (i = 0; db[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
    s_menu_tile_volume_buf[0] = 'V'; s_menu_tile_volume_buf[1] = 'O';
    s_menu_tile_volume_buf[2] = 'L'; s_menu_tile_volume_buf[3] = ' ';
    {
        uint8_t j = 4U;
        uint8_t k;
        for (k = i; db[k] != '\0' && j < 15U; k++, j++) {
            s_menu_tile_volume_buf[j] = db[k];
        }
        s_menu_tile_volume_buf[j] = '\0';
    }
    s_menu_tile_volume.label = s_menu_tile_volume_buf;
    ui_button_draw(&s_menu_tile_volume);
}

static void menu_tile_pga_refresh(void)
{
    char db[FREQ_FIELD_CHARS + 1];
    uint8_t i;

    volume_format(s_pga_gain_db_x2, db);
    for (i = 0; db[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
    s_menu_tile_pga_buf[0] = 'P'; s_menu_tile_pga_buf[1] = 'G';
    s_menu_tile_pga_buf[2] = 'A'; s_menu_tile_pga_buf[3] = ' ';
    {
        uint8_t j = 4U;
        uint8_t k;
        for (k = i; db[k] != '\0' && j < 15U; k++, j++) {
            s_menu_tile_pga_buf[j] = db[k];
        }
        s_menu_tile_pga_buf[j] = '\0';
    }
    s_menu_tile_pga.label = s_menu_tile_pga_buf;
    ui_button_draw(&s_menu_tile_pga);
}

static void menu_tile_nb_refresh(void)
{
    /* "SPT n" - n is s_spec_smooth_passes, always a single digit
     * (0-3, see SPECTRUM_LINE_SMOOTH_MAX), so no digit-extraction loop
     * needed here unlike menu_tile_smooth_refresh()'s percentage. */
    s_menu_tile_nb_buf[0] = 'S'; s_menu_tile_nb_buf[1] = 'P'; s_menu_tile_nb_buf[2] = 'T';
    s_menu_tile_nb_buf[3] = ' ';
    s_menu_tile_nb_buf[4] = (char)('0' + s_spec_smooth_passes);
    s_menu_tile_nb_buf[5] = '\0';
    s_menu_tile_nb.label = s_menu_tile_nb_buf;
    ui_button_draw(&s_menu_tile_nb);
}

static void menu_tile_smooth_refresh(void)
{
    uint8_t v = (uint8_t)(s_spectrum_smooth_alpha * 100.0f + 0.5f);
    char digits[4];
    uint8_t dpos = 3U;
    uint8_t i, j;

    digits[dpos] = '\0';
    do {
        digits[--dpos] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v > 0U && dpos > 0U);

    s_menu_tile_smooth_buf[0] = 'S'; s_menu_tile_smooth_buf[1] = 'M';
    s_menu_tile_smooth_buf[2] = 'H'; s_menu_tile_smooth_buf[3] = ' ';
    j = 4U;
    for (i = dpos; digits[i] != '\0'; i++) {
        s_menu_tile_smooth_buf[j++] = digits[i];
    }
    s_menu_tile_smooth_buf[j++] = '%';
    s_menu_tile_smooth_buf[j] = '\0';

    s_menu_tile_smooth.label = s_menu_tile_smooth_buf;
    ui_button_draw(&s_menu_tile_smooth);
}

static void menu_tile_spec_style_refresh(void)
{
    const char *v = (spectrum_get_style() == SPECTRUM_STYLE_LINE) ? "LINE" : "HEAT";
    uint8_t j = 4U;
    uint8_t i;

    s_menu_tile_spec_style_buf[0] = 'S'; s_menu_tile_spec_style_buf[1] = 'P';
    s_menu_tile_spec_style_buf[2] = 'C'; s_menu_tile_spec_style_buf[3] = ' ';
    for (i = 0; v[i] != '\0'; i++) {
        s_menu_tile_spec_style_buf[j++] = v[i];
    }
    s_menu_tile_spec_style_buf[j] = '\0';

    s_menu_tile_spec_style.label = s_menu_tile_spec_style_buf;
    ui_button_draw(&s_menu_tile_spec_style);
}

static void menu_tile_agc_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        agc_profile_cycle(); /* updates s_btn_agc_profile too - harmless, it's just not visible right now */
        menu_tile_agc_refresh();
    }
}

static void menu_tile_nb_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        s_spec_smooth_passes = (uint8_t)((s_spec_smooth_passes + 1U) % (SPECTRUM_LINE_SMOOTH_MAX + 1U));
        spectrum_set_line_smooth(s_spec_smooth_passes); /* live - no re-init needed */
        debug_print_dec("spectrum line smooth passes", s_spec_smooth_passes);
        menu_tile_nb_refresh();
    }
}

/* SPEC (trace style) behaves like AGC/SPT above, not like the DETAIL-
 * view group below - it's a 2-way toggle (HEATMAP<->LINE, see
 * spectrum_set_style()'s comment in spectrum.h), so cycling it
 * directly on tap and staying on the grid is simpler and just as
 * clear as a dedicated detail view would be for only two states. */
static void menu_tile_spec_style_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        spectrum_set_style((spectrum_get_style() == SPECTRUM_STYLE_LINE)
                            ? SPECTRUM_STYLE_HEATMAP : SPECTRUM_STYLE_LINE);
        debug_print("spectrum: style now ");
        debug_print((spectrum_get_style() == SPECTRUM_STYLE_LINE) ? "LINE\n" : "HEATMAP\n");
        menu_tile_spec_style_refresh();
    }
}

/* BW (AM/SSB audio filter width) - added 02/08/2026, replacing the
 * grid's BANDS tile (see s_menu_tile_bw's declaration comment).
 * Extended from a plain WIDE/NARROW toggle to a 3-way cycle
 * (4K0->2K3->1K8->4K0) the same day per the project owner, once the
 * main-screen BW badge itself became a second, real entry point (see
 * s_btn_audio_bw's declaration and audio_bw_button_callback() below) -
 * a cycle handles 3+ states as cleanly as a toggle handles 2, so this
 * keeps the same "cycle directly on tap, stay on the grid" treatment
 * as SPEC's HEATMAP<->LINE toggle rather than a DETAIL view - see
 * menu_tile_spec_style_callback()'s comment for that reasoning.
 * Applies live, same as SPEC - demod_am_process_raw() picks up the new
 * setting on its very next block.
 *
 * audio_bw_cycle() is the single shared step - both this grid tile's
 * callback AND s_btn_audio_bw's callback go through it, so the two
 * entry points can't drift out of sync with each other (same reasoning
 * agc_profile_cycle() already established for AGC's two entry points).
 */
static void menu_tile_bw_refresh(void); /* forward decl - audio_bw_cycle() below calls it before its own definition */
static void audio_bw_cycle(void)
{
    audio_bw_t bw = demod_am_get_audio_bw();

    switch (bw) {
    case AUDIO_BW_4K0: bw = AUDIO_BW_2K3; break;
    case AUDIO_BW_2K3: bw = AUDIO_BW_1K8; break;
    case AUDIO_BW_1K8:
    default:            bw = AUDIO_BW_4K0; break;
    }
    demod_am_set_audio_bw(bw);
    debug_print("audio filter: now ");
    debug_print(k_audio_bw_labels[(uint8_t)bw]);
    debug_print("\n");

    /* badges_draw() (not just a direct s_btn_audio_bw update) because
     * it's the function that already knows how to compute bw_label
     * correctly for the CURRENT mode - keeps this in one place rather
     * than duplicating that switch here too. Cheap: a handful of small
     * rect+text draws, same cost class as the mode-change call sites
     * that already call the whole thing (e.g.
     * menu_band_preset_callback()). */
    badges_draw();
    /* s_menu_tile_bw only actually exists on screen while the grid is
     * showing AND the RADIO page (BW's home page - see the "Settings
     * grid PAGES" comment) is the one currently selected - redrawing
     * it any other time would paint tile graphics at s_menu_tile_bw's
     * STALE coordinates, straight into whatever is actually on screen
     * there right now (the live spectrum/waterfall panel if the menu
     * is closed - MENU_AREA overlaps that region, see its comment -
     * or the UI/HW page's own tiles otherwise). Same menu-only-repaint
     * guard settings_value_redraw() and friends already use, extended
     * with the page check the paged-grid redesign added. */
    if (s_menu_open && s_menu_page == MENU_PAGE_RADIO) {
        menu_tile_bw_refresh();
    }
}

static void menu_tile_bw_refresh(void)
{
    const char *v = k_audio_bw_labels[(uint8_t)demod_am_get_audio_bw()];
    uint8_t j = 3U;
    uint8_t i;

    s_menu_tile_bw_buf[0] = 'B'; s_menu_tile_bw_buf[1] = 'W'; s_menu_tile_bw_buf[2] = ' ';
    for (i = 0; v[i] != '\0'; i++) {
        s_menu_tile_bw_buf[j++] = v[i];
    }
    s_menu_tile_bw_buf[j] = '\0';

    s_menu_tile_bw.label = s_menu_tile_bw_buf;
    ui_button_draw(&s_menu_tile_bw);
}

static void menu_tile_bw_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        /* Unconditional, unlike s_btn_audio_bw's callback below - this
         * is a deliberate menu action, not a live mode-dependent
         * readout, so cycling it while in NFM/WFM to get AM/SSB's
         * filter ready ahead of time is fine (see
         * demod_am_set_audio_bw()'s "harmless in NFM/WFM" comment in
         * demod_am.h). menu_tile_bw_refresh() (called inside
         * audio_bw_cycle() via s_menu_open) repaints THIS tile
         * regardless, since the menu is obviously open right now if
         * this callback fired at all. */
        audio_bw_cycle();
    }
}

/* s_btn_audio_bw's callback - unlike menu_tile_bw_callback() above,
 * this one is MODE-GATED: it's a live, always-visible readout (see
 * badges_draw()'s comment), so cycling it while its own label is
 * showing NFM/WFM's unrelated fixed "6K3"/"96K" would silently change
 * AM/SSB's filter with zero visible feedback right now - confusing,
 * not "harmless". Tapping it outside AM/USB/LSB is simply a no-op
 * instead. */
static void audio_bw_button_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        demod_mode_t mode = demod_am_get_mode();

        if (mode == DEMOD_MODE_AM || mode == DEMOD_MODE_USB || mode == DEMOD_MODE_LSB) {
            audio_bw_cycle();
        } else {
            debug_print("audio filter: BW badge tap ignored - not in AM/USB/LSB\n");
        }
    }
}

static void menu_tile_zoom_refresh(void)
{
    const char *v = (s_spec_zoom == SPEC_ZOOM_8X) ? "8X" :
                     (s_spec_zoom == SPEC_ZOOM_4X) ? "4X" :
                     (s_spec_zoom == SPEC_ZOOM_2X) ? "2X" : "1X";
    uint8_t j = 5U;
    uint8_t i;

    s_menu_tile_zoom_buf[0] = 'Z'; s_menu_tile_zoom_buf[1] = 'O';
    s_menu_tile_zoom_buf[2] = 'O'; s_menu_tile_zoom_buf[3] = 'M';
    s_menu_tile_zoom_buf[4] = ' ';
    for (i = 0; v[i] != '\0'; i++) {
        s_menu_tile_zoom_buf[j++] = v[i];
    }
    s_menu_tile_zoom_buf[j] = '\0';

    s_menu_tile_zoom.label = s_menu_tile_zoom_buf;
    ui_button_draw(&s_menu_tile_zoom);
}

/* ZOOM cycles 1X -> 2X -> 4X -> 8X -> 1X, same "direct cycle, stay on
 * the grid" behavior as AGC/SPT/SPEC - see spec_zoom_t's comment for
 * what each level actually does. */
static void menu_tile_zoom_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        switch (s_spec_zoom) {
        case SPEC_ZOOM_1X: s_spec_zoom = SPEC_ZOOM_2X; break;
        case SPEC_ZOOM_2X: s_spec_zoom = SPEC_ZOOM_4X; break;
        case SPEC_ZOOM_4X: s_spec_zoom = SPEC_ZOOM_8X; break;
        case SPEC_ZOOM_8X:
        default:            s_spec_zoom = SPEC_ZOOM_1X; break;
        }
        debug_print_dec("spectrum: zoom now", (uint32_t)1U << (uint8_t)s_spec_zoom);
        menu_tile_zoom_refresh();
    }
}

/*
 * Shared by all BAND_PRESET_COUNT preset tiles - which preset arrives
 * via user_data (see menu_bands_show(), where each tile's user_data is
 * set to its own index into k_band_presets[], cast through a
 * uintptr_t round-trip). Applies the whole preset in one go (frequency
 * + mode + step) and returns straight to the main radio screen - same
 * "close the menu so the result is immediately visible" reasoning the
 * SQUELCH/BACKLIGHT/SCALE/VOLUME/SMOOTH tiles already use, just with
 * nothing left to adjust afterward (unlike those, a band preset is a
 * one-shot jump, not an ongoing knob target).
 */
static void menu_band_preset_callback(void *widget, ui_event_t event, void *user_data)
{
    uintptr_t idx = (uintptr_t)user_data;

    (void)widget;
    if (event == UI_EVENT_RELEASE && idx < BAND_PRESET_COUNT) {
        const band_preset_t *p = &k_band_presets[idx];

        s_tune_hz = p->freq_hz;
        s_tune_step_idx = p->step_idx;
        demod_am_set_mode(p->mode);
        apply_lo_tune(s_tune_hz);

        debug_print("bands: preset applied -> ");
        debug_print(p->label);
        debug_print("\n");

        /* menu_screen_close() only restores the spectrum/waterfall
         * panel + span labels (see its own comment) - it doesn't know
         * the frequency/mode/step actually CHANGED (as opposed to just
         * having been hidden behind the menu, unchanged, the way it is
         * for every other tile) - so those three readouts (and the
         * BW badge, which is mode-dependent) need an explicit repaint
         * here, not just a restore. */
        freq_display_draw();
        mode_display_draw();
        step_display_draw();
        badges_draw();
        menu_screen_close();
    }
}

/*
 * SQUELCH/BACKLIGHT/SCALE/VOLUME/SMOOTH tiles all do the SAME thing on
 * tap now (31/07/2026): open a DETAIL view for that target instead of
 * just selecting it and bouncing back to the main screen - see
 * s_menu_screen's declaration comment on why the old behavior wasn't
 * REAL interaction. One tiny callback per tile (rather than one
 * generic callback reading which ui_button_t* fired) because
 * ui_button_t's on_event already gets `widget` for exactly that kind
 * of dispatch, but menu_detail_show() takes an encoder_target_t, not a
 * widget pointer - a switch-on-widget-pointer indirection here would
 * be MORE code than five one-liners, not less.
 */
static void menu_tile_squelch_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_SQUELCH); }
}

static void menu_tile_backlight_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_BACKLIGHT); }
}

static void menu_tile_scale_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_SCALE); }
}

static void menu_tile_volume_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_VOLUME); }
}

static void menu_tile_pga_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_PGA); }
}

/*
 * SPK (HW page, see speaker_pa_set_enabled()'s comment) - a direct
 * toggle, same "cycle right here, stay on this screen" behavior as
 * AGC/BW/ZOOM (see menu_grid_show()'s comment on tile behaviors), not
 * a DETAIL view: there's only one bit to flip, a whole separate
 * screen for it would be overkill. No live-value char buffer needed
 * (unlike AGC/SQUELCH/etc.) - only two possible strings, so the label
 * just points straight at one of the two literals below.
 */
static void menu_tile_speaker_pa_refresh(void)
{
    s_menu_tile_speaker_pa.label = s_speaker_pa_enabled ? "SPK ON" : "SPK OFF";
    ui_button_draw(&s_menu_tile_speaker_pa);
}

static void menu_tile_speaker_pa_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        speaker_pa_set_enabled(!s_speaker_pa_enabled);
        debug_print("speaker PA: now ");
        debug_print(s_speaker_pa_enabled ? "ON\n" : "OFF (headphones only)\n");
        menu_tile_speaker_pa_refresh();
    }
}

static void menu_tile_smooth_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) { menu_detail_show(ENCODER_TARGET_SMOOTH); }
}

static void menu_tile_exit_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        menu_screen_close();
    }
}

/*
 * Shared callback for all 3 page-selector tiles (RADIO/UI/HW) - see
 * the "Settings grid PAGES" comment on s_menu_page's declaration and
 * menu_grid_show()'s comment for the layout this switches between.
 * user_data carries the target menu_page_t (same (void*)(uintptr_t)
 * pattern menu_band_preset_callback()/menu_step_preset_callback() use
 * for their own index). Tapping the ALREADY-selected page still just
 * re-runs menu_grid_show() - harmless, and simpler than special-casing
 * a no-op.
 */
static void menu_page_select_callback(void *widget, ui_event_t event, void *user_data)
{
    menu_page_t page = (menu_page_t)(uintptr_t)user_data;

    (void)widget;
    if (event == UI_EVENT_RELEASE && page < MENU_PAGE_COUNT) {
        s_menu_page = page;
        menu_grid_show();
    }
}

static void menu_detail_back_callback(void *widget, ui_event_t event, void *user_data)
{
    (void)widget;
    (void)user_data;
    if (event == UI_EVENT_RELEASE) {
        menu_grid_show(); /* back to the tile grid, menu stays open */
    }
}

/*
 * BANDS preset list - reachable from the bottom bar (s_btn_bands,
 * added 02/08/2026, replacing the old SPT smoothing-cycle shortcut -
 * see its comment in demo_button_callback()). It USED to also have a
 * tile inside the settings grid (menu_tile_bands_callback()), but
 * that tile was repurposed 02/08/2026 into the BW audio-filter toggle
 * (see s_menu_tile_bw's declaration comment) once the bottom-bar
 * shortcut made the grid entry point redundant - the grid was already
 * completely full (12/12 slots), so freeing this one was the only way
 * to fit BW in without a bigger layout change. All BAND_PRESET_COUNT
 * preset tiles fill the WHOLE 4x3 grid (12 slots) - no BACK tile: every
 * preset tap already calls menu_screen_close() itself (see
 * menu_band_preset_callback()'s comment), so a separate "never mind,
 * go back" tile had nothing left to do once picking ANY tile here
 * closes the screen anyway. If you open this by mistake and don't
 * want to change bands, the long-press-the-knob gesture (see
 * tune_encoder_poll()'s comment) still gets you out without picking
 * anything.
 */
static void menu_bands_show(void)
{
    uint8_t i;

    gfx_fill_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_BLACK);
    gfx_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_GRAY);
    ui_screen_init(&s_menu_screen);

    for (i = 0; i < (uint8_t)BAND_PRESET_COUNT; i++) {
        uint16_t col = i % 4U;
        uint16_t row = i / 4U;

        s_menu_band_tiles[i] = (ui_button_t){
            MENU_TILE_COL(col), MENU_TILE_ROW(row), MENU_TILE_W, MENU_TILE_H,
            k_band_presets[i].label, GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_band_preset_callback, (void *)(uintptr_t)i};
        ui_screen_add_button(&s_menu_screen, &s_menu_band_tiles[i]);
    }

    ui_screen_draw(&s_menu_screen);

    s_menu_detail_active = 0U;
    s_menu_bands_active = 1U;
    s_menu_step_active = 0U;
    s_menu_mode_active = 0U;
    s_menu_open = 1U; /* harmless if already 1 (opened from the grid); required when opened straight from s_btn_bands, same reasoning as menu_step_list_show()'s comment */
}

/*
 * STEP / MODE picker lists - added 01/08/2026 per the project owner:
 * the bottom-bar STEP and MODE buttons used to just cycle to the next
 * entry on tap (see the old s_btn_step/s_btn_mode branches in
 * demo_button_callback()'s history); now they open a real "pick from
 * all N at once" screen instead, same tile-grid mechanism as BANDS
 * above, just entered DIRECTLY from the bottom bar rather than from
 * within the settings menu grid. Unlike BANDS (no BACK tile at all -
 * see its comment), these DO get a BACK/CANCEL tile, reusing
 * menu_tile_exit_callback(): with only 8 (STEP) or 5 (MODE) entries
 * there's always a free slot for one, and a "never mind" escape hatch
 * costs nothing when there's room for it.
 * Applying a selection closes the same way BANDS does - see
 * menu_band_preset_callback()'s comment for that reasoning, which
 * applies here unchanged.
 */
static void menu_step_preset_callback(void *widget, ui_event_t event, void *user_data)
{
    uintptr_t idx = (uintptr_t)user_data;

    (void)widget;
    if (event == UI_EVENT_RELEASE && idx < TUNE_STEP_COUNT) {
        s_tune_step_idx = (uint8_t)idx;
        debug_print_dec("tune: step now Hz", k_tune_steps[s_tune_step_idx]);
        /* Same "menu_screen_close() doesn't know this readout actually
         * CHANGED" gap as menu_band_preset_callback()'s comment - needs
         * an explicit repaint, not just the panel-border restore. */
        step_display_draw();
        menu_screen_close();
    }
}

static void menu_mode_preset_callback(void *widget, ui_event_t event, void *user_data)
{
    uintptr_t idx = (uintptr_t)user_data;

    (void)widget;
    if (event == UI_EVENT_RELEASE && idx < DEMOD_MODE_ENTRY_COUNT) {
        demod_mode_t mode = k_demod_modes[idx].mode;

        demod_am_set_mode(mode);
        /* Re-tune at the (unchanged) selected frequency so the LO
         * offset behavior matches the NEW mode immediately - same
         * WFM/NFM reasoning as the old cycling MODE button, see
         * apply_lo_tune()'s comment. */
        apply_lo_tune(s_tune_hz);
        debug_print("mode: demodulator now ");
        debug_print(k_demod_modes[idx].label);
        debug_print("\n");
        mode_display_draw();
        badges_draw(); /* BW badge is mode-dependent, see its comment */
        menu_screen_close();
    }
}

static void menu_step_list_show(void)
{
    uint8_t i;

    gfx_fill_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_BLACK);
    gfx_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_GRAY);
    ui_screen_init(&s_menu_screen);

    for (i = 0; i < (uint8_t)TUNE_STEP_COUNT; i++) {
        uint16_t col = i % 4U;
        uint16_t row = i / 4U;
        /* Highlight the currently-active step so it's obvious at a
         * glance which one turning the knob would have cycled to
         * next under the old behavior. */
        uint8_t cur = (i == s_tune_step_idx);

        s_menu_step_tiles[i] = (ui_button_t){
            MENU_TILE_COL(col), MENU_TILE_ROW(row), MENU_TILE_W, MENU_TILE_H,
            k_tune_step_labels[i],
            cur ? GFX_COLOR_BLACK : GFX_COLOR_WHITE,
            cur ? GFX_COLOR_CYAN  : GFX_COLOR_DARKGRAY,
            GFX_COLOR_GRAY,
            2, 0, 1, menu_step_preset_callback, (void *)(uintptr_t)i};
        ui_screen_add_button(&s_menu_screen, &s_menu_step_tiles[i]);
    }

    s_menu_detail_back = (ui_button_t){
        MENU_TILE_COL(0), MENU_TILE_ROW(2), MENU_TILE_W, MENU_TILE_H,
        "BACK", GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_WHITE,
        3, 0, 1, menu_tile_exit_callback, NULL};
    ui_screen_add_button(&s_menu_screen, &s_menu_detail_back);

    ui_screen_draw(&s_menu_screen);

    s_menu_detail_active = 0U;
    s_menu_bands_active = 0U;
    s_menu_mode_active = 0U;
    s_menu_step_active = 1U;
    s_menu_open = 1U; /* opened straight from the bottom bar, not the grid - unlike BANDS, nothing else sets this first */
    debug_print("menu: step picker opened\n");
}

static void menu_mode_list_show(void)
{
    uint8_t i;
    demod_mode_t cur_mode = demod_am_get_mode();

    gfx_fill_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_BLACK);
    gfx_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_GRAY);
    ui_screen_init(&s_menu_screen);

    for (i = 0; i < (uint8_t)DEMOD_MODE_ENTRY_COUNT; i++) {
        uint16_t col = i % 4U;
        uint16_t row = i / 4U;
        uint8_t cur = (k_demod_modes[i].mode == cur_mode);

        s_menu_mode_tiles[i] = (ui_button_t){
            MENU_TILE_COL(col), MENU_TILE_ROW(row), MENU_TILE_W, MENU_TILE_H,
            k_demod_modes[i].label,
            cur ? GFX_COLOR_BLACK : GFX_COLOR_WHITE,
            cur ? GFX_COLOR_CYAN  : GFX_COLOR_DARKGRAY,
            GFX_COLOR_GRAY,
            2, 0, 1, menu_mode_preset_callback, (void *)(uintptr_t)i};
        ui_screen_add_button(&s_menu_screen, &s_menu_mode_tiles[i]);
    }

    s_menu_detail_back = (ui_button_t){
        MENU_TILE_COL(0), MENU_TILE_ROW(2), MENU_TILE_W, MENU_TILE_H,
        "BACK", GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_WHITE,
        3, 0, 1, menu_tile_exit_callback, NULL};
    ui_screen_add_button(&s_menu_screen, &s_menu_detail_back);

    ui_screen_draw(&s_menu_screen);

    s_menu_detail_active = 0U;
    s_menu_bands_active = 0U;
    s_menu_step_active = 0U;
    s_menu_mode_active = 1U;
    s_menu_open = 1U;
    debug_print("menu: mode picker opened\n");
}

/*
 * Repaints ONLY the value area of the currently-open detail view -
 * called both by menu_detail_show() (initial paint) and by
 * tune_encoder_poll() through settings_value_redraw() every time the
 * knob actually changes something. Fixed clear rect up front so
 * switching targets (via BACK -> another tile) or SCALE's LO/HI
 * toggle never leaves a ghost of the previous content, same fixed-
 * width-repaint reasoning used everywhere else in this file.
 *
 * SQUELCH/BACKLIGHT/VOLUME/SMOOTH: one big centered value. SCALE is
 * the odd one out - two values (LO and HI) side by side, the active
 * one (s_scale_adjust_max) highlighted cyan, matching the same visual
 * language aux_row_display_draw() already uses for it on the main
 * screen.
 */
/*
 * DETAIL VIEW geometry - confined to MENU_AREA (676x358 @ (0,SPEC_Y)),
 * same reasoning as the GRID's MENU_TILE_* macros above. Vertical
 * zones, non-overlapping: title (72-100) / hint (104-118) / value
 * area, redrawn on every encoder tick (126-350) / BACK button
 * (358-414) - all comfortably inside MENU_AREA_Y..MENU_AREA_Y+
 * MENU_AREA_H (64-422).
 */
#define MENU_DETAIL_TITLE_Y  72
#define MENU_DETAIL_HINT_Y   104
#define MENU_DETAIL_VALUE_CLEAR_Y 126
#define MENU_DETAIL_VALUE_CLEAR_H 224
#define MENU_DETAIL_VALUE_Y  217 /* centers a scale-6 (42px tall) line in the clear band above */
#define MENU_DETAIL_SCALE_LABEL_Y 140
#define MENU_DETAIL_SCALE_VALUE_Y 190
#define MENU_DETAIL_BACK_X 228
#define MENU_DETAIL_BACK_Y 358
#define MENU_DETAIL_BACK_W 220
#define MENU_DETAIL_BACK_H 56

static void menu_detail_value_redraw(void)
{
    gfx_fill_rect(MENU_AREA_X, MENU_DETAIL_VALUE_CLEAR_Y, MENU_AREA_W, MENU_DETAIL_VALUE_CLEAR_H, GFX_COLOR_BLACK);

    switch (s_menu_detail_target) {
    case ENCODER_TARGET_SQUELCH: {
        char buf[FREQ_FIELD_CHARS + 1];
        uint16_t vw;
        uint8_t i;

        spectrum_db_format((int16_t)demod_am_get_squelch_db(), buf);
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        vw = gfx_text_width(&buf[i], 6);
        gfx_text((uint16_t)((MENU_AREA_W - vw) / 2), MENU_DETAIL_VALUE_Y, &buf[i], GFX_COLOR_CYAN, GFX_COLOR_BLACK, 6);
        break;
    }
    case ENCODER_TARGET_BACKLIGHT: {
        char buf[8];
        uint8_t pos = 7U;
        uint8_t v = backlight_get_percent();
        uint16_t vw;

        buf[pos] = '\0';
        buf[--pos] = '%';
        do { buf[--pos] = (char)('0' + (v % 10U)); v /= 10U; } while (v > 0U && pos > 0U);
        while (pos > 0U) { buf[--pos] = ' '; }
        vw = gfx_text_width(buf, 6);
        gfx_text((uint16_t)((MENU_AREA_W - vw) / 2), MENU_DETAIL_VALUE_Y, buf, GFX_COLOR_CYAN, GFX_COLOR_BLACK, 6);
        break;
    }
    case ENCODER_TARGET_VOLUME: {
        char buf[FREQ_FIELD_CHARS + 1];
        uint16_t vw;
        uint8_t i;

        volume_format(s_volume_db_x2, buf);
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        vw = gfx_text_width(&buf[i], 6);
        gfx_text((uint16_t)((MENU_AREA_W - vw) / 2), MENU_DETAIL_VALUE_Y, &buf[i], GFX_COLOR_CYAN, GFX_COLOR_BLACK, 6);
        break;
    }
    case ENCODER_TARGET_PGA: {
        /* Same volume_format() reuse as aux_row_display_draw()'s PGA
         * branch - see its comment on why the always-non-negative
         * "+" sign is correct here, not a formatting bug. */
        char buf[FREQ_FIELD_CHARS + 1];
        uint16_t vw;
        uint8_t i;

        volume_format(s_pga_gain_db_x2, buf);
        for (i = 0; buf[i] == ' ' && i < (FREQ_FIELD_CHARS - 7U); i++) { }
        vw = gfx_text_width(&buf[i], 6);
        gfx_text((uint16_t)((MENU_AREA_W - vw) / 2), MENU_DETAIL_VALUE_Y, &buf[i], GFX_COLOR_CYAN, GFX_COLOR_BLACK, 6);
        break;
    }
    case ENCODER_TARGET_SMOOTH: {
        char buf[8];
        uint8_t pos = 7U;
        uint8_t v = (uint8_t)(s_spectrum_smooth_alpha * 100.0f + 0.5f);
        uint16_t vw;

        buf[pos] = '\0';
        buf[--pos] = '%';
        do { buf[--pos] = (char)('0' + (v % 10U)); v /= 10U; } while (v > 0U && pos > 0U);
        while (pos > 0U) { buf[--pos] = ' '; }
        vw = gfx_text_width(buf, 6);
        gfx_text((uint16_t)((MENU_AREA_W - vw) / 2), MENU_DETAIL_VALUE_Y, buf, GFX_COLOR_CYAN, GFX_COLOR_BLACK, 6);
        break;
    }
    case ENCODER_TARGET_SCALE: {
        char lo[FREQ_FIELD_CHARS + 1];
        char hi[FREQ_FIELD_CHARS + 1];
        uint8_t li, hi_i;
        uint16_t lo_fg = s_scale_adjust_max ? GFX_COLOR_GRAY  : GFX_COLOR_BLACK;
        uint16_t lo_bg = s_scale_adjust_max ? GFX_COLOR_BLACK : GFX_COLOR_CYAN;
        uint16_t hi_fg = s_scale_adjust_max ? GFX_COLOR_BLACK : GFX_COLOR_GRAY;
        uint16_t hi_bg = s_scale_adjust_max ? GFX_COLOR_CYAN  : GFX_COLOR_BLACK;

        spectrum_db_format((int16_t)s_db_min, lo);
        spectrum_db_format((int16_t)s_db_max, hi);
        for (li = 0; lo[li] == ' ' && li < (FREQ_FIELD_CHARS - 7U); li++) { }
        for (hi_i = 0; hi[hi_i] == ' ' && hi_i < (FREQ_FIELD_CHARS - 7U); hi_i++) { }

        gfx_text(90, MENU_DETAIL_SCALE_LABEL_Y, "LO", GFX_COLOR_GRAY, GFX_COLOR_BLACK, 3);
        gfx_text(60, MENU_DETAIL_SCALE_VALUE_Y, &lo[li], lo_fg, lo_bg, 4);
        gfx_text(460, MENU_DETAIL_SCALE_LABEL_Y, "HI", GFX_COLOR_GRAY, GFX_COLOR_BLACK, 3);
        gfx_text(430, MENU_DETAIL_SCALE_VALUE_Y, &hi[hi_i], hi_fg, hi_bg, 4);
        break;
    }
    default:
        break;
    }
}

static void menu_detail_show(encoder_target_t target)
{
    const char *title;
    const char *hint;

    s_encoder_target = target;
    s_menu_detail_target = target;
    s_menu_detail_active = 1U;

    switch (target) {
    case ENCODER_TARGET_SQUELCH:   title = "SQUELCH";   hint = "TURN KNOB TO ADJUST";  break;
    case ENCODER_TARGET_BACKLIGHT: title = "BACKLIGHT"; hint = "TURN KNOB TO ADJUST";  break;
    case ENCODER_TARGET_VOLUME:    title = "VOLUME";    hint = "TURN KNOB TO ADJUST";  break;
    case ENCODER_TARGET_PGA:       title = "PGA GAIN";  hint = "TURN KNOB TO ADJUST";  break;
    case ENCODER_TARGET_SMOOTH:    title = "SMOOTH";    hint = "TURN KNOB TO ADJUST";  break;
    case ENCODER_TARGET_SCALE:     title = "SCALE";     hint = "PRESS KNOB: LO OR HI"; break;
    default:                        title = "";          hint = "";                     break;
    }

    /* Only clear/redraw MENU_AREA (see menu_grid_show()'s comment) -
     * same border treatment as the grid, for visual continuity
     * between the two levels of this one screen. */
    gfx_fill_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_BLACK);
    gfx_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_GRAY);
    ui_screen_init(&s_menu_screen);

    {
        uint16_t tw = gfx_text_width(title, 4);
        gfx_text((uint16_t)((MENU_AREA_W - tw) / 2), MENU_DETAIL_TITLE_Y, title, GFX_COLOR_WHITE, GFX_COLOR_BLACK, 4);
    }
    {
        uint16_t hw = gfx_text_width(hint, 2);
        gfx_text((uint16_t)((MENU_AREA_W - hw) / 2), MENU_DETAIL_HINT_Y, hint, GFX_COLOR_GRAY, GFX_COLOR_BLACK, 2);
    }

    s_menu_detail_back = (ui_button_t){
        MENU_DETAIL_BACK_X, MENU_DETAIL_BACK_Y, MENU_DETAIL_BACK_W, MENU_DETAIL_BACK_H,
        "BACK", GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_WHITE,
        3, 0, 1, menu_detail_back_callback, NULL};
    ui_screen_add_button(&s_menu_screen, &s_menu_detail_back);
    ui_button_draw(&s_menu_detail_back);

    menu_detail_value_redraw();

    s_menu_open = 1U;
    debug_print("menu: detail view opened for ");
    debug_print(title);
    debug_print("\n");
}

/*
 * settings_value_redraw(): the dispatcher tune_encoder_poll() calls
 * instead of aux_row_display_draw() directly, for every target that
 * can now be reached from EITHER the main screen's aux row OR the
 * menu's detail view - it decides which (if either) is actually
 * visible right now and repaints that one, so the live value always
 * updates wherever the user can actually see it.
 */
static void settings_value_redraw(void)
{
    if (s_menu_open) {
        if (s_menu_detail_active) {
            menu_detail_value_redraw();
        }
        /* else: grid is showing, not a detail view - nothing visible
         * needs repainting here (shouldn't normally happen, since
         * s_encoder_target only becomes one of these targets via
         * menu_detail_show(), which also sets s_menu_detail_active). */
    } else {
        aux_row_display_draw();
    }
}

static void menu_grid_show(void)
{
    /* Only clear/redraw the confined area (MENU_AREA_*) - the top
     * bar, right column, and bottom button bar stay exactly as they
     * were, never touched. A bordered rect (same bg/border colors
     * s_spectrum_panel already uses) makes it visually read as "this
     * panel, replaced" rather than a random black hole. Redrawn in
     * full on EVERY call, including a page switch (see this file's
     * "Settings grid PAGES" comment) - simplest way to guarantee a
     * page that just lost a tile (e.g. switching RADIO -> HW) never
     * leaves a stale tile behind from the previous page. */
    gfx_fill_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_BLACK);
    gfx_rect(MENU_AREA_X, MENU_AREA_Y, MENU_AREA_W, MENU_AREA_H, GFX_COLOR_GRAY);
    ui_screen_init(&s_menu_screen);

    /*
     * --- Page selector column (col 0, all 3 rows) -----------------
     * Visually distinct from every option tile: ORANGE instead of
     * the CYAN(cycle)/DARKGRAY(opens detail)/YELLOW(exit) palette the
     * right-hand side uses. Selected page: solid ORANGE fill,
     * BLACK text. Unselected: BLACK fill, ORANGE text/border - reads
     * as "outlined, tap to switch" next to the filled active one.
     */
    s_menu_page_radio = (ui_button_t){
        MENU_TILE_COL(0), MENU_TILE_ROW(0), MENU_TILE_W, MENU_TILE_H,
        "RADIO",
        (s_menu_page == MENU_PAGE_RADIO) ? GFX_COLOR_BLACK  : GFX_COLOR_ORANGE,
        (s_menu_page == MENU_PAGE_RADIO) ? GFX_COLOR_ORANGE : GFX_COLOR_BLACK,
        GFX_COLOR_ORANGE,
        2, 0, 1, menu_page_select_callback, (void *)(uintptr_t)MENU_PAGE_RADIO};
    s_menu_page_ui = (ui_button_t){
        MENU_TILE_COL(0), MENU_TILE_ROW(1), MENU_TILE_W, MENU_TILE_H,
        "UI",
        (s_menu_page == MENU_PAGE_UI) ? GFX_COLOR_BLACK  : GFX_COLOR_ORANGE,
        (s_menu_page == MENU_PAGE_UI) ? GFX_COLOR_ORANGE : GFX_COLOR_BLACK,
        GFX_COLOR_ORANGE,
        2, 0, 1, menu_page_select_callback, (void *)(uintptr_t)MENU_PAGE_UI};
    s_menu_page_hw = (ui_button_t){
        MENU_TILE_COL(0), MENU_TILE_ROW(2), MENU_TILE_W, MENU_TILE_H,
        "HW",
        (s_menu_page == MENU_PAGE_HW) ? GFX_COLOR_BLACK  : GFX_COLOR_ORANGE,
        (s_menu_page == MENU_PAGE_HW) ? GFX_COLOR_ORANGE : GFX_COLOR_BLACK,
        GFX_COLOR_ORANGE,
        2, 0, 1, menu_page_select_callback, (void *)(uintptr_t)MENU_PAGE_HW};
    ui_screen_add_button(&s_menu_screen, &s_menu_page_radio);
    ui_screen_add_button(&s_menu_screen, &s_menu_page_ui);
    ui_screen_add_button(&s_menu_screen, &s_menu_page_hw);

    /*
     * --- EXIT (slot 8, row2/col3) - fixed on every page ------------
     * Neither a cycler nor a detail-opener - just closes the whole
     * menu (menu_screen_close()), same as before the redesign.
     */
    s_menu_tile_exit = (ui_button_t){
        MENU_OPT_COL(MENU_OPT_EXIT_SLOT), MENU_OPT_ROW(MENU_OPT_EXIT_SLOT), MENU_TILE_W, MENU_TILE_H,
        "EXIT", GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_WHITE,
        2, 0, 1, menu_tile_exit_callback, NULL};
    ui_screen_add_button(&s_menu_screen, &s_menu_tile_exit);

    /*
     * --- Page options (slots 0-7) -----------------------------------
     * See this file's "Settings grid PAGES" comment for the full
     * RADIO/UI/HW slot assignment. Only the tiles belonging to the
     * CURRENTLY selected page get positioned/added/refreshed here -
     * every s_menu_tile_* struct is shared across pages (same static
     * storage it always had), so leaving an inactive page's tiles
     * untouched this call is what keeps their stale coordinates from
     * ever being drawn onto the wrong page.
     */
    switch (s_menu_page) {
    case MENU_PAGE_RADIO:
        s_menu_tile_agc = (ui_button_t){
            MENU_OPT_COL(0), MENU_OPT_ROW(0), MENU_TILE_W, MENU_TILE_H,
            "AGC", GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_agc_callback, NULL};
        s_menu_tile_squelch = (ui_button_t){
            MENU_OPT_COL(1), MENU_OPT_ROW(1), MENU_TILE_W, MENU_TILE_H,
            "SQUELCH", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_squelch_callback, NULL};
        s_menu_tile_volume = (ui_button_t){
            MENU_OPT_COL(2), MENU_OPT_ROW(2), MENU_TILE_W, MENU_TILE_H,
            "VOL", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_volume_callback, NULL};
        /* BW: cycles the AM/SSB audio filter width 4K0/2K3/1K8 (see
         * menu_tile_bw_callback()'s comment). */
        s_menu_tile_bw = (ui_button_t){
            MENU_OPT_COL(3), MENU_OPT_ROW(3), MENU_TILE_W, MENU_TILE_H,
            "BW", GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_bw_callback, NULL};
        /* PGA: AIC3204 MIC_PGA analog input gain (0-47.5dB) - see
         * aic3204_set_pga_gain_db()'s comment. */
        s_menu_tile_pga = (ui_button_t){
            MENU_OPT_COL(4), MENU_OPT_ROW(4), MENU_TILE_W, MENU_TILE_H,
            "PGA", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_pga_callback, NULL};
        /* Slots 5-7 intentionally empty - room to grow RADIO further. */

        ui_screen_add_button(&s_menu_screen, &s_menu_tile_agc);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_squelch);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_volume);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_bw);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_pga);

        ui_screen_draw(&s_menu_screen);
        /* ui_screen_draw() just painted each tile with its STATIC
         * label ("AGC", "SQUELCH", ...) via ui_button_draw()'s normal
         * single-line render - immediately overpaint the ones that
         * need a live value with the real content. */
        menu_tile_agc_refresh();
        menu_tile_squelch_refresh();
        menu_tile_volume_refresh();
        menu_tile_bw_refresh();
        menu_tile_pga_refresh();
        break;

    case MENU_PAGE_UI:
        s_menu_tile_backlight = (ui_button_t){
            MENU_OPT_COL(0), MENU_OPT_ROW(0), MENU_TILE_W, MENU_TILE_H,
            "BL", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_backlight_callback, NULL};
        /* SCALE's GRID tile shows no live value (two numbers, LO+HI,
         * don't fit this single-string tile format cleanly) - just a
         * static label. Its DETAIL view (menu_detail_value_redraw())
         * DOES show both, side by side. */
        s_menu_tile_scale = (ui_button_t){
            MENU_OPT_COL(1), MENU_OPT_ROW(1), MENU_TILE_W, MENU_TILE_H,
            "SCALE", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_scale_callback, NULL};
        s_menu_tile_nb = (ui_button_t){
            MENU_OPT_COL(2), MENU_OPT_ROW(2), MENU_TILE_W, MENU_TILE_H,
            "SPT", GFX_COLOR_BLACK, GFX_COLOR_GREEN, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_nb_callback, NULL};
        s_menu_tile_smooth = (ui_button_t){
            MENU_OPT_COL(3), MENU_OPT_ROW(3), MENU_TILE_W, MENU_TILE_H,
            "SMOOTH", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_smooth_callback, NULL};
        /* SPC: the spectrum trace style toggle (HEATMAP<->LINE) - see
         * spectrum_set_style()'s comment in spectrum.h. */
        s_menu_tile_spec_style = (ui_button_t){
            MENU_OPT_COL(4), MENU_OPT_ROW(4), MENU_TILE_W, MENU_TILE_H,
            "SPC", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_spec_style_callback, NULL};
        /* ZOOM: spectrum/waterfall zoom (1x/2x/4x/8x) - see
         * spec_zoom_t's comment. */
        s_menu_tile_zoom = (ui_button_t){
            MENU_OPT_COL(5), MENU_OPT_ROW(5), MENU_TILE_W, MENU_TILE_H,
            "ZOOM", GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_zoom_callback, NULL};
        /* Slots 6-7 intentionally empty - room to grow UI further. */

        ui_screen_add_button(&s_menu_screen, &s_menu_tile_backlight);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_scale);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_nb);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_smooth);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_spec_style);
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_zoom);

        ui_screen_draw(&s_menu_screen);
        menu_tile_backlight_refresh();
        menu_tile_nb_refresh();
        menu_tile_smooth_refresh();
        menu_tile_spec_style_refresh();
        menu_tile_zoom_refresh();
        break;

    case MENU_PAGE_HW:
        /* SPK: speaker PA enable/mute (PB7 - UNCONFIRMED, see
         * speaker_pa_set_enabled()'s comment). First (and so far
         * only) HW-page option; slots 1-7 still intentionally empty,
         * reserved for future hardware-related settings. */
        s_menu_tile_speaker_pa = (ui_button_t){
            MENU_OPT_COL(0), MENU_OPT_ROW(0), MENU_TILE_W, MENU_TILE_H,
            "SPK", GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
            2, 0, 1, menu_tile_speaker_pa_callback, NULL};
        ui_screen_add_button(&s_menu_screen, &s_menu_tile_speaker_pa);

        ui_screen_draw(&s_menu_screen);
        menu_tile_speaker_pa_refresh();
        break;

    default:
        /* Should never actually be reached (s_menu_page only ever
         * takes one of the MENU_PAGE_COUNT values above) - safety net
         * so an out-of-range value still paints SOMETHING (the page
         * selector column + EXIT tile already added above) rather
         * than leaving the options area in an inconsistent state. */
        ui_screen_draw(&s_menu_screen);
        break;
    }

    s_menu_detail_active = 0U;
    s_menu_bands_active = 0U;
    s_menu_step_active = 0U;
    s_menu_mode_active = 0U;
    s_menu_open = 1U;
}

static void menu_screen_open(void)
{
    menu_grid_show();
    debug_print("menu: settings screen opened\n");
}

static void menu_screen_close(void)
{
    s_menu_open = 0U;
    s_menu_detail_active = 0U;
    s_menu_bands_active = 0U;
    s_menu_step_active = 0U;
    s_menu_mode_active = 0U;
    /* Only the spectrum+waterfall panels need restoring - the top
     * bar, right column, and bottom bar were never hidden or
     * touch-disabled while the menu was open (see s_menu_screen's
     * declaration comment), so redrawing the WHOLE screen here (the
     * old behavior, back when the menu covered everything) would just
     * be wasted EXMC bandwidth and a visible flash of things that
     * never changed. ui_panel_draw() restores each panel's
     * background+border immediately; the actual spectrum TRACE and
     * waterfall content follow naturally on the next
     * sdr_spectrum_waterfall_tick() frame (within ~33ms - imperceptible). */
    ui_panel_draw(&s_spectrum_panel);
    ui_panel_draw(&s_waterfall_panel);
    /* The span-label row (+/- edges, "LO" marker, divider) lives
     * INSIDE the spectrum panel and gets wiped by the menu's black
     * fill same as the border does - restore it too, zoom-aware in
     * case the ZOOM tile was used while the menu was open. */
    spec_span_labels_draw();
    debug_print("menu: settings screen closed\n");
}

/*
 * Called from the main loop: drains the encoder and applies tuning.
 * Deltas accumulate inside the encoder driver while the loop is busy
 * with the FFT/waterfall, so a fast spin during a slow loop pass
 * coalesces into ONE retune of (detents * step) instead of queueing
 * stale intermediate retunes - the radio always jumps straight to
 * where the knob actually is.
 */
/*
 * Programs the LO for s_tune_hz according to the CURRENT demod mode,
 * and keeps demod_am's if-offset flag in sync with it. Added
 * 31/07/2026 alongside WFM and factored out of tune_encoder_poll()
 * because now TWO things can trigger a re-tune at the same
 * frequency: moving the encoder, and toggling the MODE button into or
 * out of WFM (see demo_button_callback()) - before WFM, only the
 * encoder ever changed s_tune_hz, so this logic lived inline there.
 *
 * WFM tunes the LO DIRECTLY on the selected frequency, no offset -
 * see demod_am.h's WFM note for why (it needs the full +/-96kHz of
 * complex bandwidth centered on the station, not shifted 48kHz off
 * it). AM/USB/LSB keep the existing low-IF behavior unchanged.
 */
static void apply_lo_tune(uint32_t freq_hz)
{
    if (demod_am_get_mode() == DEMOD_MODE_WFM) {
        if (!ms5351_set_lo_freq(freq_hz)) {
            debug_print_dec("tune: ms5351_set_lo_freq FAILED (WFM, no offset) at Hz", freq_hz);
        } else {
            demod_am_set_if_offset_active(0U);
        }
    } else {
        /* Low-IF tuning (2nd attempt, Fs/4 sign-flip rotation - see
         * demod_am.h's LOW-IF TUNING note for the full history): the
         * LO sits DEMOD_IF_OFFSET_HZ below the selected station so
         * demod_am_process_raw()'s down-mix can pull the wanted
         * signal off the LO-leakage artifact at 0Hz. s_tune_hz itself
         * stays the selected/displayed frequency. The
         * demod_am_set_if_offset_active(1) call keeps the digital
         * down-mix in sync with the LO actually being offset from
         * here on (idempotent - fine to call on every retune, not
         * just the first). */
        if (!ms5351_set_lo_freq(freq_hz - DEMOD_IF_OFFSET_HZ)) {
            debug_print_dec("tune: ms5351_set_lo_freq FAILED at Hz", freq_hz);
        } else {
            demod_am_set_if_offset_active(1U);
        }
    }
}

static void tune_encoder_poll(void)
{
    int32_t detents    = encoder_take_delta();
    uint8_t press       = encoder_take_press();
    uint8_t long_press  = encoder_take_long_press();
    uint8_t changed = 0;

    /* Long-press: unconditionally hands the knob back to TUNE and, if
     * the settings menu is open, closes it too - one gesture that
     * always gets you back to "turning the knob retunes the radio",
     * from anywhere. Added 01/08/2026 to fix a real bug: closing the
     * menu via the EXIT tile (menu_screen_close()) never reset
     * s_encoder_target, so after e.g. picking SCALE from the menu and
     * exiting, the knob kept adjusting the dB scale instead of the
     * frequency, with no way back short of reopening the menu and
     * picking VOL/etc to bounce through. A SHORT press couldn't be
     * reused for this - it already means something different in
     * every non-TUNE target (cycles the tune step, or toggles SCALE's
     * LO/HI bound - see the per-target branches below), so this
     * needed a gesture none of them use. Checked first, before
     * touching detents/press against whatever the OLD target was, and
     * returns immediately - a long-press is a deliberate "get me out"
     * and shouldn't also be evaluated as a turn against a target
     * we're leaving in the same poll. */
    if (long_press) {
        if (s_encoder_target != ENCODER_TARGET_TUNE) {
            s_encoder_target = ENCODER_TARGET_TUNE;
            debug_print("encoder: long-press - knob back to TUNE\n");
            aux_row_display_draw(); /* clear the stale LO/HI/SQL/BL/... row - always visible, menu or not, see s_btn_vol's callback for the same unconditional call */
        }
        if (s_menu_open) {
            menu_screen_close();
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_VOLUME) {

        /* Volume mode: the encoder button still cycles the tune step
         * (ready for when you flip back) - but only draw it if the
         * main screen is actually showing; STEP's position doesn't
         * exist on the menu detail view (see s_menu_open's checks
         * throughout this function). */
        if (press) {
            s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
            if (!s_menu_open) { step_display_draw(); }
        }

        if (detents != 0) {
            int32_t v = (int32_t)s_volume_db_x2 + detents * VOLUME_STEP_X2;

            if (v < VOLUME_MIN_X2) { v = VOLUME_MIN_X2; }
            if (v > VOLUME_MAX_X2) { v = VOLUME_MAX_X2; }

            if ((int16_t)v != s_volume_db_x2) {
                s_volume_db_x2 = (int16_t)v;
                aic3204_set_volume_db((float)s_volume_db_x2 * 0.5f);
                settings_value_redraw();
            }
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_PGA) {

        /* Same "button still cycles tune step" courtesy as VOLUME
         * above. */
        if (press) {
            s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
            if (!s_menu_open) { step_display_draw(); }
        }

        if (detents != 0) {
            int32_t v = (int32_t)s_pga_gain_db_x2 + detents * PGA_STEP_X2;

            if (v < PGA_MIN_X2) { v = PGA_MIN_X2; }
            if (v > PGA_MAX_X2) { v = PGA_MAX_X2; }

            if ((int16_t)v != s_pga_gain_db_x2) {
                s_pga_gain_db_x2 = (int16_t)v;
                aic3204_set_pga_gain_db((float)s_pga_gain_db_x2 * 0.5f);
                settings_value_redraw();
            }
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_BACKLIGHT) {

        /* Same "button still cycles tune step" courtesy as VOLUME
         * mode above. */
        if (press) {
            s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
            if (!s_menu_open) { step_display_draw(); }
        }

        if (detents != 0) {
            int32_t pct = (int32_t)backlight_get_percent() + detents * (int32_t)BACKLIGHT_STEP;

            if (pct < 20)   { pct = 20; }
            if (pct > 100) { pct = 100; }

            if ((uint8_t)pct != backlight_get_percent()) {
                backlight_set_percent((uint8_t)pct);
                settings_value_redraw();
            }
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_SCALE) {

        /* Here the encoder BUTTON does NOT cycle the tune step like
         * the other two non-TUNE targets - it toggles which bound
         * (db_min/"LO" vs db_max/"HI") the knob's rotation moves. See
         * s_scale_adjust_max's comment. */
        if (press) {
            s_scale_adjust_max = (uint8_t)(s_scale_adjust_max ? 0U : 1U);
            settings_value_redraw();
        }

        if (detents != 0) {
            float step = (float)detents * SPECTRUM_DB_STEP;

            if (s_scale_adjust_max) {
                float v = s_db_max + step;

                if (v > SPECTRUM_DB_CEIL) { v = SPECTRUM_DB_CEIL; }
                /* Don't let HI get pulled down past LO+GAP - clamp
                 * against the OTHER bound, not just the ceiling. */
                if (v < s_db_min + SPECTRUM_DB_MIN_GAP) { v = s_db_min + SPECTRUM_DB_MIN_GAP; }
                if (v != s_db_max) {
                    s_db_max = v;
                    settings_value_redraw();
                }
            } else {
                float v = s_db_min + step;

                if (v < SPECTRUM_DB_FLOOR) { v = SPECTRUM_DB_FLOOR; }
                if (v > s_db_max - SPECTRUM_DB_MIN_GAP) { v = s_db_max - SPECTRUM_DB_MIN_GAP; }
                if (v != s_db_min) {
                    s_db_min = v;
                    settings_value_redraw();
                }
            }
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_SQUELCH) {
        

        /* Button just cycles the tune step, same courtesy as VOLUME/
         * BACKLIGHT - no second sub-value to toggle here. */
        if (press) {
            s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
            if (!s_menu_open) { step_display_draw(); }
        }

        if (detents != 0) {
            float v = demod_am_get_squelch_db() + (float)detents * SQUELCH_DB_STEP;

            if (v < SQUELCH_DB_FLOOR) { v = SQUELCH_DB_FLOOR; }
            if (v > SQUELCH_DB_CEIL)  { v = SQUELCH_DB_CEIL; }
            if (v != demod_am_get_squelch_db()) {
                demod_am_set_squelch_db(v);
                settings_value_redraw();
            }
        }
        return;
    }

    if (s_encoder_target == ENCODER_TARGET_SMOOTH) {

        /* Same "button still cycles tune step" courtesy as VOLUME/
         * BACKLIGHT/SQUELCH above. */
        if (press) {
            s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
            if (!s_menu_open) { step_display_draw(); }
        }

        if (detents != 0) {
            float v = s_spectrum_smooth_alpha + (float)detents * SPECTRUM_SMOOTH_STEP;

            if (v < SPECTRUM_SMOOTH_MIN) { v = SPECTRUM_SMOOTH_MIN; }
            if (v > SPECTRUM_SMOOTH_MAX) { v = SPECTRUM_SMOOTH_MAX; }
            if (v != s_spectrum_smooth_alpha) {
                s_spectrum_smooth_alpha = v;
                settings_value_redraw();
            }
        }
        return;
    }

    if (press) {
        s_tune_step_idx = (uint8_t)((s_tune_step_idx + 1U) % TUNE_STEP_COUNT);
        debug_print_dec("tune: step now Hz", k_tune_steps[s_tune_step_idx]);
        step_display_draw();
    }

    if (detents != 0) {
        int64_t f = (int64_t)s_tune_hz + (int64_t)detents * (int64_t)k_tune_steps[s_tune_step_idx];

        if (f < (int64_t)TUNE_MIN_HZ) {
            f = (int64_t)TUNE_MIN_HZ;
        } else if (f > (int64_t)TUNE_MAX_HZ) {
            f = (int64_t)TUNE_MAX_HZ;
        }

        if ((uint32_t)f != s_tune_hz) {
            s_tune_hz = (uint32_t)f;
            apply_lo_tune(s_tune_hz);
            changed = 1;
        }
    }

    if (changed) {
        freq_display_draw();
        spec_span_labels_draw(); /* "esta escala tiene que variar con
                                   * la frecuencia" - see its comment */
    }
}

/*
 * Shared by the 6 bottom-bar buttons (fires on RELEASE, the actual
 * "click"):
 *   MODE - opens the mode picker list (menu_mode_list_show()) instead
 *           of cycling directly - added 01/08/2026, see its comment.
 *   VOL  - jumps the encoder target straight to VOLUME (or back to
 *           TUNE if already there) - see s_encoder_target's comment.
 *   STEP - opens the tune-step picker list (menu_step_list_show())
 *           instead of cycling directly - added 01/08/2026, see its
 *           comment alongside menu_mode_list_show()'s above. Pressing
 *           the encoder (not this button) still cycles the step the
 *           old way - see tune_encoder_poll()'s per-target branches.
 *   NR   - cycles the AGC profile (MAN/SLW/MED/FST) - repurposed
 *           31/07/2026, see agc_profile_cycle()'s comment and
 *           s_nr_on's comment for why.
 *   BANDS - opens the BANDS preset list (menu_bands_show()) directly,
 *           without detouring through the settings grid - repurposed
 *           02/08/2026 from the SPT spatial-line-smoothing shortcut
 *           (still reachable via its badge + the settings-menu grid
 *           tile - see s_spec_smooth_passes' comment).
 *   MENU - opens the settings menu screen (menu_screen_open()) - see
 *           s_menu_screen's declaration comment. Replaced the old
 *           encoder-target cycle 31/07/2026.
 */
static void demo_button_callback(void *widget, ui_event_t event, void *user_data)
{
    ui_button_t *btn = (ui_button_t *)widget;
    (void)user_data;

    if (event == UI_EVENT_RELEASE) {
        debug_print("button pressed: ");
        debug_print(btn->label);
        debug_print("\n");

        if (widget == &s_btn_vol) {
            s_encoder_target = (s_encoder_target == ENCODER_TARGET_VOLUME)
                                ? ENCODER_TARGET_TUNE : ENCODER_TARGET_VOLUME;
            debug_print((s_encoder_target == ENCODER_TARGET_VOLUME)
                        ? "vol: encoder now controls VOLUME\n"
                        : "vol: encoder now controls TUNE\n");
            aux_row_display_draw();
        } else if (widget == &s_btn_step) {
            /* Opens the picker list (menu_step_list_show()) instead of
             * cycling directly - see its comment. Applying a choice
             * there closes the menu itself; nothing else to do here. */
            menu_step_list_show();
        } else if (widget == &s_btn_nr) {
            /* Repurposed 31/07/2026 - see agc_profile_cycle()'s
             * comment: NR's noise-reduction toggle was never wired to
             * real DSP anyway (s_nr_on is now dead - nothing sets it
             * anymore, and its row0 badge just stays permanently off,
             * see badges_draw()), so the button does something real
             * instead: a second, physically-separate way to cycle the
             * AGC profile, alongside tapping s_btn_agc_profile
             * directly. Deliberately a quick stopgap, not a redesign -
             * a proper settings menu is coming later today once the
             * badge grid's out of room for more controls. */
            agc_profile_cycle();
        } else if (widget == &s_btn_bands) {
            /* Repurposed 02/08/2026 from the SPT smoothing-cycle
             * shortcut (see s_spec_smooth_passes' comment) to open the
             * BANDS list (menu_bands_show()) directly from the main
             * screen, without detouring through the settings grid -
             * same direct-entry treatment s_btn_step/s_btn_mode
             * already got 01/08/2026. menu_bands_show() sets
             * s_menu_open itself (see its comment), so there's nothing
             * else to do here - applying a preset closes the menu on
             * its own too (menu_band_preset_callback()). */
            menu_bands_show();
        } else if (widget == &s_btn_menu) {
            /* Opens the settings menu screen (menu_screen_open()) -
             * replaces the old TUNE -> BACKLIGHT -> SCALE -> SQUELCH
             * -> TUNE cycle 31/07/2026: tapping the tile you actually
             * want directly is strictly better than stepping through
             * a cycle to find it, now that there's somewhere for those
             * tiles to live - see s_menu_screen's declaration comment.
             * s_encoder_target itself is untouched here - each tile's
             * own callback sets it (or leaves it alone, for AGC/SPT
             * which act immediately instead). */
            menu_screen_open();
        } else if (widget == &s_btn_mode) {
            /* Opens the picker list (menu_mode_list_show()) instead of
             * cycling directly - see its comment alongside
             * menu_step_list_show(). Applying a choice there closes
             * the menu itself; nothing else to do here. */
            menu_mode_list_show();
        }
    }
}

static void radio_screen_draw(void)
{
    uint8_t i;
    static const char *k_btn_labels[6] = { "MODE", "VOL", "STEP", "NR", "BANDS", "MENU" };
    ui_button_t *k_btns[6];

    k_btns[0] = &s_btn_mode;
    k_btns[1] = &s_btn_vol;
    k_btns[2] = &s_btn_step;
    k_btns[3] = &s_btn_nr;
    k_btns[4] = &s_btn_bands;
    k_btns[5] = &s_btn_menu;

    gfx_fill_screen(GFX_COLOR_BLACK);
    ui_screen_init(&s_demo_screen);

    /* Top bar: freq/mode/step/vol/time/battery live here, drawn by
     * their own readout functions after the panels. */
    s_title_panel = (ui_panel_t){0, 0, GFX_SCREEN_WIDTH, TOP_H,
                                  GFX_COLOR_DARKGRAY, GFX_COLOR_DARKGRAY};
    ui_screen_add_panel(&s_demo_screen, &s_title_panel);

    /* Main display column: spectrum over waterfall. */
    s_spectrum_panel = (ui_panel_t){0, SPEC_Y, MAIN_W, SPEC_H,
                                     GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_spectrum_panel);

    s_waterfall_panel = (ui_panel_t){0, WF_PANEL_Y, MAIN_W,
                                      (uint16_t)(WATERFALL_ROWS + 4), GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_waterfall_panel);

    /* Right status column (S-meter + badges drawn on top afterwards). */
    s_rcol_panel = (ui_panel_t){RCOL_X, RCOL_Y, RCOL_W, RCOL_H,
                                 GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_rcol_panel);

    /* Bottom bar: 6 buttons. enabled=1 set explicitly - if omitted, a
     * freshly declared ui_button_t defaults to enabled=0 (C zero-
     * initialization) and ui_screen_touch() would ignore it even
     * though it draws fine. MODE gets the yellow "primary" styling. */
    for (i = 0; i < 6U; i++) {
        uint16_t x = (uint16_t)(BTNBAR_GAP + i * (BTNBAR_BTN_W + BTNBAR_GAP));
        uint16_t fg = (i == 0U) ? GFX_COLOR_BLACK : GFX_COLOR_WHITE;
        uint16_t bg = (i == 0U) ? GFX_COLOR_YELLOW : GFX_COLOR_DARKGRAY;
        uint16_t border = (i == 0U) ? GFX_COLOR_BLACK : GFX_COLOR_WHITE;

        *k_btns[i] = (ui_button_t){x, BTNBAR_Y, BTNBAR_BTN_W, BTNBAR_BTN_H,
                                    k_btn_labels[i], fg, bg, border,
                                    2, 0, 1, demo_button_callback, NULL};
        ui_screen_add_button(&s_demo_screen, k_btns[i]);
    }

    /* AGC profile badge/button (see its declaration + badges_draw()'s
     * comment) - lives in the badge grid, not the bottom bar, but is
     * registered here alongside the other 6 buttons for the same
     * reason: ui_screen_add_button() is what makes ui_screen_touch()
     * actually hit-test it. Position matches badges_draw()'s BADGE_X1/
     * row1 exactly (same constants, defined earlier in this file).
     * Same cyan "on" look the BW badge used to have here, since this
     * is always "live", never a dark/disabled state. */
    s_btn_agc_profile = (ui_button_t){
        BADGE_X1, (uint16_t)(BADGE_Y0 + BADGE_ROW_STEP),
        BADGE_W, BADGE_H,
        k_agc_profile_labels[(uint8_t)demod_am_get_agc_profile()],
        GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
        2, 0, 1, agc_profile_button_callback, NULL};
    ui_screen_add_button(&s_demo_screen, &s_btn_agc_profile);

    /* BW badge/button (see its declaration + badges_draw()'s comment)
     * - same "real widget, not just badge_draw()" treatment as
     * s_btn_agc_profile just above, same reason (ui_screen_add_button()
     * is what makes ui_screen_touch() actually hit-test it). Position
     * matches badges_draw()'s BADGE_X0/row2 exactly. Initial fg/bg here
     * are just a starting guess (cyan, "interactive") - badges_draw()
     * overwrites both every time it runs, based on the CURRENT mode,
     * before this is ever visible to anyone. */
    s_btn_audio_bw = (ui_button_t){
        BADGE_X0, (uint16_t)(BADGE_Y0 + 2 * BADGE_ROW_STEP),
        BADGE_W, BADGE_H,
        k_audio_bw_labels[(uint8_t)demod_am_get_audio_bw()],
        GFX_COLOR_BLACK, GFX_COLOR_CYAN, GFX_COLOR_GRAY,
        2, 0, 1, audio_bw_button_callback, NULL};
    ui_screen_add_button(&s_demo_screen, &s_btn_audio_bw);

    ui_screen_draw(&s_demo_screen);

    /* Static labels drawn once, bypassing the screen (no touch, no
     * state): S-meter caption + panadapter span labels. Span: 192kHz
     * I/Q sampling -> +/-96kHz around the LO, which sits on the
     * center line of the trace (the demod point marker may sit off-
     * center - see the SR/4 low-IF notes). */
    gfx_text((uint16_t)(RCOL_X + 6), (uint16_t)(RCOL_Y + 6), "SIGNAL",
              GFX_COLOR_GREEN, GFX_COLOR_BLACK, 1);
    spec_span_labels_draw();

    /* Dynamic readouts, first paint. */
    freq_display_draw();
    mode_display_draw();
    step_display_draw();
    aux_row_display_draw();
    time_display_draw();
    battery_display_draw();
    smeter_draw(0);
    badges_draw();
}

/*
 * Drag-to-tune on the spectrum panel - added 01/08/2026 per the
 * project owner: dragging a finger across the spectrum RIGHT lowers
 * the tuned frequency, LEFT raises it - the classic "pan the content"
 * feel (like dragging a map or photo: dragging right reveals what was
 * further left, i.e. LOWER frequency on this panadapter, since
 * frequency increases left-to-right - see spec_span_labels_draw()'s
 * tick ruler).
 *
 * QUANTIZED to whole SPEC_DRAG_HZ_STEP jumps (1kHz) rather than
 * applied as continuous fractional Hz - added same day, per the
 * project owner: raw pixel-proportional Hz produced odd non-round
 * frequencies and, on this still-uncalibrated resistive touch panel
 * (see demo_touch_poll()'s CALIBRATION NOTE), tiny single-pixel
 * jitter was enough to wobble the tuned frequency by a few Hz with no
 * visible finger movement. *hz_accum carries the sub-step remainder
 * between calls - same carry technique encoder_take_delta() already
 * uses for quarter-steps, so a slow drag that hasn't crossed a full
 * 1kHz yet isn't lost, just accumulated for next time. Reset by the
 * caller (demo_touch_poll()) at the START of each new drag gesture -
 * see its comment - so leftover remainder from a previous unrelated
 * drag never bleeds into this one.
 *
 * Hz-per-pixel (before quantizing) still comes from the current
 * zoom's span (see spec_span_labels_draw()'s switch for where these
 * same span numbers come from) divided across SPEC_TRACE_W, so how
 * much finger travel one 1kHz step takes still scales with zoom -
 * finer control zoomed in, coarser zoomed out - only the OUTPUT is
 * now snapped to round steps, not the sensitivity.
 *
 * Called once per touch SAMPLE while dragging (see demo_touch_poll()),
 * not once per gesture - dx_px is the delta since the LAST sample, not
 * since the press started, so the trace/frequency updates live as the
 * finger moves rather than jumping once on release.
 */
#define SPEC_DRAG_HZ_STEP 1000.0f /* frequency change per whole drag "step" */

static void spec_drag_tune_apply(uint16_t x, uint16_t prev_x, float *hz_accum)
{
    int32_t dx_px = (int32_t)x - (int32_t)prev_x;
    float hz_per_px;
    int32_t steps;
    int64_t f;

    if (dx_px == 0) {
        return; /* no horizontal movement since the last sample */
    }

    switch (s_spec_zoom) {
    case SPEC_ZOOM_2X: hz_per_px = 96000.0f  / (float)SPEC_TRACE_W; break;
    case SPEC_ZOOM_4X: hz_per_px = 48000.0f  / (float)SPEC_TRACE_W; break;
    case SPEC_ZOOM_8X: hz_per_px = 24000.0f  / (float)SPEC_TRACE_W; break;
    case SPEC_ZOOM_1X:
    default:            hz_per_px = 192000.0f / (float)SPEC_TRACE_W; break;
    }

    /* Drag right (dx_px > 0) -> frequency DOWN, so subtract - see this
     * function's comment for the "pan the content" reasoning. */
    *hz_accum -= (float)dx_px * hz_per_px;

    /* C99 truncates toward zero - exactly what we want for both signs
     * here, same reasoning as encoder_take_delta()'s comment. Usually
     * +/-1 for a normal drag speed; a fast flick between polls can
     * legitimately produce more in one call. */
    steps = (int32_t)(*hz_accum / SPEC_DRAG_HZ_STEP);
    if (steps == 0) {
        return; /* hasn't crossed a full step yet - remainder stays in *hz_accum for next time */
    }
    *hz_accum -= (float)steps * SPEC_DRAG_HZ_STEP;

    f = (int64_t)s_tune_hz + (int64_t)steps * (int64_t)SPEC_DRAG_HZ_STEP;

    if (f < (int64_t)TUNE_MIN_HZ) { f = (int64_t)TUNE_MIN_HZ; }
    if (f > (int64_t)TUNE_MAX_HZ) { f = (int64_t)TUNE_MAX_HZ; }

    if ((uint32_t)f != s_tune_hz) {
        s_tune_hz = (uint32_t)f;
        apply_lo_tune(s_tune_hz);
        freq_display_draw();
        spec_span_labels_draw(); /* "esta escala tiene que variar con
                                   * la frecuencia" - see its comment */
    }
}

/*
 * Touch input hook, already active. touch_read() does the heavy
 * lifting (bit-banged SPI transactions + averaging) ONLY once
 * touch_is_pressed() has already returned true (a cheap plain GPIO
 * read), so calling it every main-loop iteration is not expensive
 * while there's no contact.
 *
 * CALIBRATION NOTE: without calling touch_set_calibration(), touch.c
 * uses an identity mapping (raw 0-4095 -> screen 0-800/0-480) that
 * almost certainly does not match the resistive panel's real
 * orientation/scale. It's enough to validate that the whole pipeline
 * works (that a touch reaches and presses a button), but a real
 * calibration - touching the 4 known corners, noting the raw_x/raw_y
 * from touch_debug_raw() at each, and filling in a real
 * touch_calibration_t via touch_set_calibration() - is still pending.
 * Drag-to-tune (spec_drag_tune_apply() above) inherits this same
 * caveat: it works on RELATIVE deltas between consecutive samples, so
 * it degrades more gracefully than absolute-position touches (a
 * button hit-test) under a wrong scale/swap/invert - the DIRECTION
 * could still come out backwards if invert_x is wrong, worth
 * double-checking once real calibration lands.
 */
static void demo_touch_poll(void)
{
    /*
     * REGION-based routing (31/07/2026) - see s_menu_screen's
     * declaration comment on why the menu now only covers MENU_AREA
     * (the spectrum+waterfall panel) instead of the whole screen: the
     * top bar, right column, and bottom bar stay live and touchable
     * the entire time, so touches inside MENU_AREA while the menu is
     * open go to s_menu_screen, everything else always goes to
     * s_demo_screen.
     *
     * BUG FIXED HERE (same day): the region check must be decided
     * ONCE, on the FIRST sample of a touch (the press), and then held
     * for the WHOLE gesture through release - NOT re-evaluated on
     * every sample. The original version re-checked the region on
     * every poll, including the release sample - and on this
     * hardware's resistive touch driver, the release sample often
     * reports spurious coordinates (e.g. x=0,y=0) that fail the
     * region check even though the press that started the gesture
     * passed it. That sent the RELEASE to the WRONG screen: whichever
     * screen got the press (say s_menu_screen, for a tap on EXIT)
     * never received its matching release, so its active_index stayed
     * stuck on that widget forever - and since ui_screen_touch() only
     * does a fresh hit-test when active_index is -1, EVERY subsequent
     * tap on that screen (any tile, including EXIT again) then hit
     * the "already have an active widget, just check if still inside"
     * branch instead, matched against the WRONG (stuck) widget, and
     * did nothing. That's the exact "opens, then nothing responds,
     * not even EXIT" symptom.
     *
     * s_touch_owner_is_menu now records which screen owns the CURRENT
     * gesture, decided only when a NEW press starts (s_touch_active
     * transitions 0->1), and reused unchanged - including for the
     * release itself - until the finger lifts.
     */
    static uint8_t s_touch_active = 0U;         /* 1 while a press-hold is in progress */
    static uint8_t s_touch_owner_is_menu = 0U;  /* which screen the CURRENT gesture belongs to */
    static uint8_t s_spec_drag_active = 0U;     /* 1 while the CURRENT gesture is a spectrum drag-to-tune */
    static uint16_t s_spec_drag_prev_x = 0U;    /* x of the last sample applied, for per-sample deltas */
    static float s_spec_drag_hz_accum = 0.0f;   /* sub-SPEC_DRAG_HZ_STEP carry - see spec_drag_tune_apply()'s comment */
    uint16_t x = 0, y = 0;
    uint8_t pressed = touch_read(&x, &y);

    if (pressed && !s_touch_active) {
        s_touch_active = 1U;
        s_touch_owner_is_menu = (uint8_t)(s_menu_open != 0U
            && x < MENU_AREA_W && y >= MENU_AREA_Y
            && y < (uint16_t)(MENU_AREA_Y + MENU_AREA_H));

        /* Drag-to-tune: a press starting inside the spectrum panel
         * while the menu ISN'T covering it - see spec_drag_tune_apply()'s
         * comment. Mutually exclusive with s_touch_owner_is_menu by
         * construction: MENU_AREA exactly covers the spectrum+
         * waterfall, so whenever the menu is open and owns this press,
         * this stays 0. Decided once here, same "decide on the press,
         * hold for the whole gesture" reasoning as s_touch_owner_is_menu
         * above (and for the same reason: the release sample's
         * coordinates can be garbage). s_spec_drag_hz_accum resets
         * here too - a fresh gesture starts with a clean carry,
         * regardless of whatever remainder a previous drag left behind. */
        s_spec_drag_active = (uint8_t)(!s_touch_owner_is_menu
            && x < MAIN_W && y >= SPEC_Y && y < (uint16_t)(SPEC_Y + SPEC_H));
        s_spec_drag_prev_x = x;
        s_spec_drag_hz_accum = 0.0f;
    }

    if (s_touch_owner_is_menu) {
        ui_screen_touch(&s_menu_screen, x, y, pressed);
    } else {
        ui_screen_touch(&s_demo_screen, x, y, pressed);
    }

    if (s_spec_drag_active && pressed) {
        spec_drag_tune_apply(x, s_spec_drag_prev_x, &s_spec_drag_hz_accum);
        s_spec_drag_prev_x = x;
    }

    if (!pressed) {
        s_touch_active = 0U;   /* gesture over - the next press re-decides ownership */
        s_spec_drag_active = 0U;
    }
}

/*
 * s_db_min/s_db_max (now live, encoder-adjustable state - see their
 * declaration and full comment near s_encoder_target, above) are used
 * below for both spectrum_draw() and the waterfall's colormap. The
 * UNCALIBRATED-range caveat from their original comment still stands:
 * the "dB" value comes from a bit-manipulation log2 approximation
 * (see fft.c), not a referenced measurement.
 */

/* sdr_rx.h and fft.h define their sizes independently - if one is ever
 * changed without the other, a clear compile error is better than a
 * silent overflow of s_rx_i/s_rx_q. */
#if SDR_RX_BLOCK_SAMPLES != FFT_SIZE
#error "SDR_RX_BLOCK_SAMPLES (sdr_rx.h) and FFT_SIZE (fft.h) must match"
#endif

static int16_t s_rx_i[SDR_RX_BLOCK_SAMPLES];
static int16_t s_rx_q[SDR_RX_BLOCK_SAMPLES];
static float   s_db[FFT_BINS_IQ]; /* fftshifted: VFO at the center index */

/*
 * UPDATED (28/07/2026): aic3204.c now runs the full real captured
 * sequence (see aic3204_phase2_init()), which restores the
 * differential I/Q routing: left = I (IN2_L/IN2_R differential),
 * right = Q (IN3_R/IN3_L differential) - this is no longer the
 * single-ended baseline from a few rounds ago.
 */

/* debug_uart.h has no signed decimal print - I/Q sample min/max are
 * int16_t and can be negative, hence this small local helper instead
 * of touching the UART module for it. Builds the full "label = -N\n"
 * string in one pass (does NOT delegate to debug_print_dec with an
 * empty label - that duplicated the " = " prefix and produced
 * confusing output like "label = - = 1" instead of "label = -1"). */
static void debug_print_dec_signed(const char *label, int32_t val)
{
    char buf[12];
    int i = 11;
    uint32_t uval;
    uint8_t negative = 0;

    buf[11] = '\0';
    if (val < 0) {
        negative = 1;
        uval = (uint32_t)(-val);
    } else {
        uval = (uint32_t)val;
    }

    if (uval == 0U) {
        buf[--i] = '0';
    } else {
        while (uval > 0U && i > 0) {
            buf[--i] = (char)('0' + (uval % 10U));
            uval /= 10U;
        }
    }
    if (negative && i > 0) {
        buf[--i] = '-';
    }

    debug_print(label);
    debug_print(" = ");
    debug_print(&buf[i]);
    debug_print("\n");
}

/*
 * --- Spectrum ZOOM: decimator + processing -------------------------------
 *
 * See spec_zoom_t's comment (near the menu tile code, above) for the
 * overall design. This block holds the actual DSP: a generic
 * decimate-by-2 FIR, cascadable up to 3x, plus the accumulation state
 * that gathers enough decimated samples across multiple raw 192kHz
 * blocks to fill one FFT_SIZE window.
 */

/*
 * ZOOM_DECIM2_COEFFS: a single decimate-by-2 stage, reused identically
 * at every cascade level (1, 2, or 3 passes) since only the INPUT:
 * OUTPUT ratio matters, not the absolute sample rate - the same 31-tap
 * filter is correct whether it's decimating 192kHz->96kHz (stage 1),
 * 96kHz->48kHz (stage 2), or 48kHz->24kHz (stage 3). Designed offline
 * via scipy.signal.firwin (Hamming window, same general FIR-lowpass
 * approach as demod_am.c's DECIM_COEFFS, just far shorter - a single
 * x2 stage doesn't need anywhere near that much stopband rejection,
 * especially since ZOOM_8X cascades three of these for a combined
 * ~84dB at the critical alias-fold point, verified numerically:
 *
 *   0.10-0.30 x Nyquist: essentially flat (+/-0.01dB) - the passband
 *   0.42 x Nyquist:      -6.00dB (roughly the -6dB corner)
 *   0.50 x Nyquist:      -27.97dB (the post-decimation Nyquist - the
 *                         worst-case fold-back point for aliasing)
 *   0.55-1.0 x Nyquist:  -54 to -62dB (deep stopband)
 *
 * A single stage's -28dB at the fold point is on the modest side for
 * a rigorous decimator, but this feeds a VISUAL spectrum display, not
 * a measurement - and cascading stages for higher zoom multiplies
 * that rejection (three stages for ZOOM_8X -> ~84dB), so it only gets
 * better at higher zoom, not worse.
 */
#define ZOOM_DECIM2_TAPS 31U
static const float32_t ZOOM_DECIM2_COEFFS[ZOOM_DECIM2_TAPS] = {
    0.0013731076f, -0.0007535444f, -0.0029087841f, -0.0005579049f, 0.0062459162f, 0.0057986726f,
    -0.0089671792f, -0.0177057998f, 0.0050097395f, 0.0361091799f, 0.0151443729f, -0.0569498814f,
    -0.0705344064f, 0.0736069684f, 0.3051388190f, 0.4199014482f, 0.3051388190f, 0.0736069684f,
    -0.0705344064f, -0.0569498814f, 0.0151443729f, 0.0361091799f, 0.0050097395f, -0.0177057998f,
    -0.0089671792f, 0.0057986726f, 0.0062459162f, -0.0005579049f, -0.0029087841f, -0.0007535444f,
    0.0013731076f
};

/* Three cascaded stages, one CMSIS decimator instance per channel per
 * stage (6 total) - each stage's block size is half the previous
 * one's, so they can't share instances/state. Named by INPUT rate at
 * 192kHz: stage1 sees the raw 512-sample block, stage2 sees stage1's
 * 256-sample output, stage3 sees stage2's 128-sample output. */
static arm_fir_decimate_instance_f32 s_zoom_dec1_i, s_zoom_dec1_q;
static arm_fir_decimate_instance_f32 s_zoom_dec2_i, s_zoom_dec2_q;
static arm_fir_decimate_instance_f32 s_zoom_dec3_i, s_zoom_dec3_q;
static float32_t s_zoom_dec1_i_state[ZOOM_DECIM2_TAPS + SDR_RX_BLOCK_SAMPLES - 1U];
static float32_t s_zoom_dec1_q_state[ZOOM_DECIM2_TAPS + SDR_RX_BLOCK_SAMPLES - 1U];
static float32_t s_zoom_dec2_i_state[ZOOM_DECIM2_TAPS + (SDR_RX_BLOCK_SAMPLES / 2U) - 1U];
static float32_t s_zoom_dec2_q_state[ZOOM_DECIM2_TAPS + (SDR_RX_BLOCK_SAMPLES / 2U) - 1U];
static float32_t s_zoom_dec3_i_state[ZOOM_DECIM2_TAPS + (SDR_RX_BLOCK_SAMPLES / 4U) - 1U];
static float32_t s_zoom_dec3_q_state[ZOOM_DECIM2_TAPS + (SDR_RX_BLOCK_SAMPLES / 4U) - 1U];

/* Working buffers, one per stage boundary. s_zoom_f_i/q hold the raw
 * block cast to float and (if needed) re-centered on the tuned
 * frequency, BEFORE stage 1. */
static float32_t s_zoom_f_i[SDR_RX_BLOCK_SAMPLES];
static float32_t s_zoom_f_q[SDR_RX_BLOCK_SAMPLES];
static float32_t s_zoom_s1_i[SDR_RX_BLOCK_SAMPLES / 2U];
static float32_t s_zoom_s1_q[SDR_RX_BLOCK_SAMPLES / 2U];
static float32_t s_zoom_s2_i[SDR_RX_BLOCK_SAMPLES / 4U];
static float32_t s_zoom_s2_q[SDR_RX_BLOCK_SAMPLES / 4U];
static float32_t s_zoom_s3_i[SDR_RX_BLOCK_SAMPLES / 8U];
static float32_t s_zoom_s3_q[SDR_RX_BLOCK_SAMPLES / 8U];

/* Accumulates decimated samples across as many raw blocks as it takes
 * to fill one FFT_SIZE window (2/4/8 raw blocks for ZOOM_2X/4X/8X -
 * see spec_zoom_t's comment). int16_t, not float32: fft_compute_db_iq()
 * takes int16_t input (same type sdr_rx.c's raw blocks use) - cheaper
 * to convert once here than to touch fft.c's signature for this. */
static int16_t s_zoom_acc_i[FFT_SIZE];
static int16_t s_zoom_acc_q[FFT_SIZE];
static uint16_t s_zoom_acc_count = 0U;

static void zoom_decimators_init(void)
{
    /* Same "check the return status, shout over UART if it ever
     * fails" discipline demod_am.c's own decimator init uses - see
     * its comment. blockSize % decimFactor == 0 is the actual
     * constraint (512/256/128, decimating by 2 each time - always
     * exact), so this isn't expected to ever trip, but a silently
     * unusable instance with no other symptom than a garbled zoomed
     * spectrum would be a nasty thing to debug blind. */
    if (arm_fir_decimate_init_f32(&s_zoom_dec1_i, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec1_i_state, SDR_RX_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage1 I init FAILED ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_zoom_dec1_q, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec1_q_state, SDR_RX_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage1 Q init FAILED ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_zoom_dec2_i, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec2_i_state, SDR_RX_BLOCK_SAMPLES / 2U) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage2 I init FAILED ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_zoom_dec2_q, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec2_q_state, SDR_RX_BLOCK_SAMPLES / 2U) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage2 Q init FAILED ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_zoom_dec3_i, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec3_i_state, SDR_RX_BLOCK_SAMPLES / 4U) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage3 I init FAILED ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_zoom_dec3_q, ZOOM_DECIM2_TAPS, 2U, ZOOM_DECIM2_COEFFS,
                                    s_zoom_dec3_q_state, SDR_RX_BLOCK_SAMPLES / 4U) != ARM_MATH_SUCCESS) {
        debug_print("zoom: *** decimator stage3 Q init FAILED ***\n");
    }
    s_zoom_acc_count = 0U;
}

/*
 * Runs ONLY when s_spec_zoom != SPEC_ZOOM_1X (the caller checks first -
 * at 1X this whole pipeline is skipped, zero extra cost). Processes
 * ONE raw 192kHz block: casts to float, re-centers on the tuned
 * frequency if needed, runs however many cascaded x2 stages the
 * current zoom level calls for, and appends the result to the
 * accumulator. Returns 1 when the accumulator just became FULL (a
 * fresh FFT_SIZE window is ready in s_zoom_acc_i/q), 0 otherwise -
 * the caller only computes/draws a new frame on a 1.
 */
static uint8_t zoom_process_block(void)
{
    uint16_t n;
    const float32_t *out_i;
    const float32_t *out_q;
    uint16_t out_len;

    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
        s_zoom_f_i[n] = (float32_t)s_rx_i[n];
        s_zoom_f_q[n] = (float32_t)s_rx_q[n];
    }

    /* Re-center on the tuned frequency BEFORE decimating - otherwise
     * stage 1's anti-alias filter (centered on 0Hz) would attenuate
     * the very station this is supposed to zoom into, whenever the
     * low-IF down-mix has the LO sitting DEMOD_IF_OFFSET_HZ off the
     * true station (AM/USB/LSB/NFM - see demod_am.h's LOW-IF TUNING
     * note). Same sign-flip-only rotation demod_am.c's own down-mix
     * uses, at the same fixed Fs/4 rate (always relative to the RAW
     * 192kHz block, regardless of zoom level - this runs BEFORE any
     * decimation touches the data). WFM has no offset to correct
     * (demod_am_get_if_offset_active() is 0 there), so this is
     * skipped entirely in that mode - the station's already at DC. */
    if (demod_am_get_if_offset_active()) {
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n += 4U) {
            float32_t hh1, hh2;

            hh1 =  s_zoom_f_q[n + 1U];
            hh2 = -s_zoom_f_i[n + 1U];
            s_zoom_f_i[n + 1U] = hh1;
            s_zoom_f_q[n + 1U] = hh2;

            hh1 = -s_zoom_f_i[n + 2U];
            hh2 = -s_zoom_f_q[n + 2U];
            s_zoom_f_i[n + 2U] = hh1;
            s_zoom_f_q[n + 2U] = hh2;

            hh1 = -s_zoom_f_q[n + 3U];
            hh2 =  s_zoom_f_i[n + 3U];
            s_zoom_f_i[n + 3U] = hh1;
            s_zoom_f_q[n + 3U] = hh2;
        }
    }

    arm_fir_decimate_f32(&s_zoom_dec1_i, s_zoom_f_i, s_zoom_s1_i, SDR_RX_BLOCK_SAMPLES);
    arm_fir_decimate_f32(&s_zoom_dec1_q, s_zoom_f_q, s_zoom_s1_q, SDR_RX_BLOCK_SAMPLES);
    out_i = s_zoom_s1_i;
    out_q = s_zoom_s1_q;
    out_len = SDR_RX_BLOCK_SAMPLES / 2U;

    if (s_spec_zoom >= SPEC_ZOOM_4X) {
        arm_fir_decimate_f32(&s_zoom_dec2_i, s_zoom_s1_i, s_zoom_s2_i, SDR_RX_BLOCK_SAMPLES / 2U);
        arm_fir_decimate_f32(&s_zoom_dec2_q, s_zoom_s1_q, s_zoom_s2_q, SDR_RX_BLOCK_SAMPLES / 2U);
        out_i = s_zoom_s2_i;
        out_q = s_zoom_s2_q;
        out_len = SDR_RX_BLOCK_SAMPLES / 4U;
    }
    if (s_spec_zoom >= SPEC_ZOOM_8X) {
        arm_fir_decimate_f32(&s_zoom_dec3_i, s_zoom_s2_i, s_zoom_s3_i, SDR_RX_BLOCK_SAMPLES / 4U);
        arm_fir_decimate_f32(&s_zoom_dec3_q, s_zoom_s2_q, s_zoom_s3_q, SDR_RX_BLOCK_SAMPLES / 4U);
        out_i = s_zoom_s3_i;
        out_q = s_zoom_s3_q;
        out_len = SDR_RX_BLOCK_SAMPLES / 8U;
    }

    for (n = 0; n < out_len && s_zoom_acc_count < FFT_SIZE; n++, s_zoom_acc_count++) {
        float32_t vi = out_i[n];
        float32_t vq = out_q[n];
        if (vi > 32767.0f)  { vi = 32767.0f; }
        if (vi < -32768.0f) { vi = -32768.0f; }
        if (vq > 32767.0f)  { vq = 32767.0f; }
        if (vq < -32768.0f) { vq = -32768.0f; }
        s_zoom_acc_i[s_zoom_acc_count] = (int16_t)vi;
        s_zoom_acc_q[s_zoom_acc_count] = (int16_t)vq;
    }

    if (s_zoom_acc_count >= FFT_SIZE) {
        s_zoom_acc_count = 0U;
        return 1U;
    }
    return 0U;
}

/*
 * Replaces the earlier synthetic-gradient waterfall demo with the real
 * capture -> FFT -> spectrum/waterfall pipeline. Non-blocking: if
 * sdr_rx_poll_block_iq() has no new block yet, this tick does nothing
 * (the rest of the main loop - touch, UI - stays just as responsive).
 */
static void sdr_spectrum_waterfall_tick(void)
{
    /*
     * RESTRUCTURED (30/07/2026) - display decoupled from block rate.
     *
     * Blocks arrive at 192kHz/512 = 375/s; redrawing the whole
     * spectrum + waterfall (~128k EXMC pixel writes) for EVERY block
     * was the main reason the display felt slow AND the trace looked
     * nervous. Now:
     *
     *   - Every polled block still gets its FFT, but the dB bins are
     *     ACCUMULATED (summed) instead of drawn - all received signal
     *     contributes, nothing is thrown away.
     *   - Every SPECTRUM_FRAME_MS the accumulated average is drawn
     *     once: spectrum + one waterfall line. Averaging N FFTs per
     *     frame lowers the displayed noise variance (calmer floor,
     *     smoother waterfall) for free.
     *   - FFTs are capped at SPECTRUM_MAX_FFT_PER_FRAME per frame so
     *     the FFT itself can never starve touch/encoder polling in
     *     the main loop; excess blocks are simply skipped (poll still
     *     drains them so DMA never backs up).
     */
#define SPECTRUM_FRAME_MS        33U /* ~30 fps */
#define SPECTRUM_MAX_FFT_PER_FRAME 6U

    static float    s_db_frame[FFT_BINS_IQ]; /* frame-averaged dB */
    static float    s_db_sum[FFT_BINS_IQ];
    static uint32_t s_db_count = 0U;
    static uint32_t s_next_frame_ms = 0U;
    static uint16_t line[WATERFALL_WIDTH];
    static uint32_t s_frame_count = 0U;
    uint32_t t_fft0, t_fft1, t_spec0, t_spec1, t_wf0, t_wf1;
    uint32_t fft_us = 0U;
    uint16_t x, n;
    int16_t i_min, i_max, q_min, q_max;
    uint32_t bi;

    if (sdr_rx_poll_block_iq(s_rx_i, s_rx_q) == 0U) {
        return;
    }

    /* Min/max of both channels, every block - the numeric evidence for
     * whether the ADC is producing real, varying samples at all (vs.
     * pinned at a fixed value, which is what we've been chasing).
     * i_min/i_max is I (left, IN2 differential), q_min/q_max is Q
     * (right, IN3 differential) - naming now matches reality again. */
    i_min = s_rx_i[0]; i_max = s_rx_i[0];
    q_min = s_rx_q[0]; q_max = s_rx_q[0];
    for (n = 1; n < SDR_RX_BLOCK_SAMPLES; n++) {
        if (s_rx_i[n] < i_min) { i_min = s_rx_i[n]; }
        if (s_rx_i[n] > i_max) { i_max = s_rx_i[n]; }
        if (s_rx_q[n] < q_min) { q_min = s_rx_q[n]; }
        if (s_rx_q[n] > q_max) { q_max = s_rx_q[n]; }
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    /* --- accumulate this block's FFT --- */
    if (s_spec_zoom == SPEC_ZOOM_1X) {
        /* Unchanged existing behavior: average up to
         * SPECTRUM_MAX_FFT_PER_FRAME raw-rate FFTs per display frame
         * (block skipped once the quota's met, still drained). */
        if (s_db_count < SPECTRUM_MAX_FFT_PER_FRAME) {
            t_fft0 = DWT->CYCCNT;
            /* Complex I/Q transform: +f/-f separated, VFO at the center of
             * the display (fftshift order, see fft.h). This is what makes
             * the panadapter read like one: a signal tuned exactly on the
             * VFO sits on the center line, not at the left edge. */
            fft_compute_db_iq(s_rx_i, s_rx_q, s_db);
            t_fft1 = DWT->CYCCNT;
            fft_us = (uint32_t)(((uint64_t)(t_fft1 - t_fft0)) * 1000000U / SystemCoreClock);

            if (s_db_count == 0U) {
                for (bi = 0; bi < FFT_BINS_IQ; bi++) {
                    s_db_sum[bi] = s_db[bi];
                }
            } else {
                for (bi = 0; bi < FFT_BINS_IQ; bi++) {
                    s_db_sum[bi] += s_db[bi];
                }
            }
            s_db_count++;
        }
    } else {
        /* Zoom mode (see spec_zoom_t's comment) - zoom_process_block()
         * MUST run on every raw block regardless of whether a completed
         * window is already waiting to be drawn: it's what keeps the
         * cascaded decimators' internal FIR history continuous. Skipping
         * a block here would corrupt that history and glitch the very
         * next window, not just this one.
         *
         * No multi-window averaging at zoom>1X - there's no time budget
         * left for it on top of the refresh-rate cost zoom already pays
         * (see spec_zoom_t's comment) - just take the FRESHEST completed
         * window each time one finishes, discarding any that complete
         * before the display-frame timer below actually fires (that
         * happens routinely at ZOOM_2X, whose ~5.3ms window time is much
         * shorter than the ~33ms frame period). s_db_count is reused as
         * the same "is there anything fresh to draw" signal the 1X path
         * already uses - 0 after a draw (or skip), 1 once a window's
         * ready - so the frame-ready gate right below needs no zoom-
         * specific logic of its own. */
        if (zoom_process_block()) {
            t_fft0 = DWT->CYCCNT;
            fft_compute_db_iq(s_zoom_acc_i, s_zoom_acc_q, s_db);
            t_fft1 = DWT->CYCCNT;
            fft_us = (uint32_t)(((uint64_t)(t_fft1 - t_fft0)) * 1000000U / SystemCoreClock);

            for (bi = 0; bi < FFT_BINS_IQ; bi++) {
                s_db_sum[bi] = s_db[bi];
            }
            s_db_count = 1U;
        }
    }

    /* --- not yet time to draw a frame? then this tick is done ---
     * Signed-difference comparison so the 32-bit ms counter wrapping
     * (every ~49 days) doesn't freeze the display. */
    if (((int32_t)(g_msticks - s_next_frame_ms) < 0) || s_db_count == 0U) {
        return;
    }
    s_next_frame_ms = g_msticks + SPECTRUM_FRAME_MS;

    /* Per-frame status updates, all cheap and all OUTSIDE the ISR:
     * S-meter (skips its blits when the segment count is unchanged)
     * and the once-a-minute time readout. ALWAYS run, even while the
     * settings menu is open (31/07/2026, see s_menu_screen's
     * declaration comment) - the right column and top bar stay live
     * the whole time the menu is showing, only the spectrum/waterfall
     * panel underneath the menu gets skipped below. */
    smeter_draw(smeter_segments_from_peak(demod_am_get_signal_peak()));
    {
        static uint32_t s_last_time_min = 0xFFFFFFFFUL;
        uint32_t now_min = g_msticks / 60000UL;
        if (now_min != s_last_time_min) {
            s_last_time_min = now_min;
            time_display_draw();
        }
    }

    /* Settings menu covers the spectrum+waterfall panel while open
     * (see s_menu_screen's declaration comment) - no point spending
     * cycles/EXMC bandwidth averaging/smoothing FFT data or redrawing
     * a panel nobody can see underneath it. FFT accumulation above
     * this point keeps running regardless (cheap, and keeps data
     * fresh for the moment the menu closes) - just the CONSUMPTION of
     * it (this whole block) is skipped. s_db_count is still reset at
     * the very end either way, so accumulation restarts cleanly next
     * frame regardless of which path was taken. */
    if (!s_menu_open) {
    /* Average of all FFTs accumulated during this frame window. */
    {
        float inv = 1.0f / (float)s_db_count;
        for (bi = 0; bi < FFT_BINS_IQ; bi++) {
            s_db_frame[bi] = s_db_sum[bi] * inv;
        }
    }

    /*
     * TEMPORAL smoothing ACROSS display frames - added 31/07/2026 per
     * the project owner: the trace looked too "nervous"/jittery bin-
     * to-bin between frames. This is a SEPARATE thing from the
     * intra-frame FFT averaging just above (which combines multiple
     * FFT snapshots WITHIN one ~33ms display frame, reducing noise
     * within a single displayed frame) - this instead blends each new
     * frame with the PREVIOUS frame's already-smoothed result, an
     * exponential moving average per bin, reducing frame-to-frame
     * jumpiness on top of that.
     *
     * s_spectrum_smooth_alpha is the blend weight given to history:
     * 0.0 disables smoothing entirely (each frame drawn raw), 0.95 is
     * the practical ceiling (see SPECTRUM_SMOOTH_MAX below - true 1.0
     * would freeze the display forever). 0.75 (the default, same
     * value this started as a #define with) at ~33ms/frame works out
     * to a time constant of about -33ms/ln(0.75) =~ 115ms (a handful
     * of frames) - enough to visibly calm the trace down without
     * making it feel laggy behind a real signal actually changing.
     * NOW LIVE-ADJUSTABLE (31/07/2026) via ENCODER_TARGET_SMOOTH - see
     * its declaration and the SMOOTH tile in the settings menu screen
     * (menu_screen_open()).
     */
    {
        static float s_db_smooth[FFT_BINS_IQ];
        static uint8_t s_db_smooth_init = 0U;

        if (!s_db_smooth_init) {
            /* First frame ever: nothing to blend with yet - seed
             * directly, rather than blending against a zeroed array
             * (which would otherwise show a slow fade-IN from silence
             * on every boot, not just a jitter reduction). */
            for (bi = 0; bi < FFT_BINS_IQ; bi++) {
                s_db_smooth[bi] = s_db_frame[bi];
            }
            s_db_smooth_init = 1U;
        } else {
            for (bi = 0; bi < FFT_BINS_IQ; bi++) {
                s_db_smooth[bi] = s_spectrum_smooth_alpha * s_db_smooth[bi]
                                   + (1.0f - s_spectrum_smooth_alpha) * s_db_frame[bi];
            }
        }
        /* s_db_frame itself becomes the smoothed result from here on
         * - both spectrum_draw() and the waterfall colormap loop
         * below read s_db_frame, so this keeps them visually
         * consistent with each other without touching either call
         * site. */
        for (bi = 0; bi < FFT_BINS_IQ; bi++) {
            s_db_frame[bi] = s_db_smooth[bi];
        }
    }

    t_spec0 = DWT->CYCCNT;
    /* Spectrum trace inside its panel (see the RADIO UI LAYOUT block):
     * top margin 4px, bottom leaves room for the span-label row.
     *
     * center_mark_offset_px: when low-IF tuning is active (see
     * demod_am.h's LOW-IF TUNING note), the demodulated signal sits
     * DEMOD_IF_OFFSET_HZ away from the true LO/center bin, not on it
     * - shift the marker line to match. Full span is +/-96kHz (192kHz
     * I/Q rate), so pixels-per-Hz = SPEC_TRACE_W/192000; at exactly
     * Fs/4 that's SPEC_TRACE_W/4 = 168, exact. POSITIVE (right,
     * higher frequency): bench-confirmed 31/07/2026 (flipped from an
     * earlier NEGATIVE assumption, which had it backwards) - the
     * wanted signal lands at +SR/4 relative to the LO. This is a
     * DISPLAY-ONLY marker; it doesn't need to (and doesn't) match
     * sign with demod_am.c's down-mix rotation, which operates on a
     * different axis (time-domain phase rotation direction, not a
     * screen-pixel offset) - don't assume the two must carry the same
     * sign just because they're both "SR/4-related".
     *
     * ZOOM (s_spec_zoom != SPEC_ZOOM_1X): always 0 regardless of
     * if_offset_active - zoom_process_block() already re-centers the
     * signal on the tuned frequency BEFORE decimating (see its own
     * comment), specifically so the zoomed FFT has the station sitting
     * at DC. Applying the SR/4 offset on top of that would be double-
     * correcting for something the zoom pipeline already fixed. */
    {
        int16_t center_mark_offset_px = 0;
        uint8_t band_active = 0U;
        int16_t band_lo_offset_px = 0;
        int16_t band_hi_offset_px = 0;

        if (s_spec_zoom == SPEC_ZOOM_1X && demod_am_get_if_offset_active()) {
            center_mark_offset_px = (int16_t)((uint32_t)SPEC_TRACE_W * DEMOD_IF_OFFSET_HZ / 192000UL);
        }

        /*
         * Demodulated-bandwidth tint (see spectrum_draw()'s comment
         * in spectrum.h) - added 03/08/2026, per the project owner:
         * shows which slice of the panadapter the CURRENT audio
         * bandwidth selection (BW tile - see k_audio_bw_hz above)
         * actually covers, anchored on the SAME point
         * center_mark_offset_px already marks (so it moves together
         * with the low-IF marker, and collapses to the panel center
         * under ZOOM exactly like that marker does - see its comment
         * above for why).
         *
         * Only meaningful for AM/USB/LSB, which are the only modes
         * with a caller-selectable audio bandwidth (NFM's channel
         * filter and WFM's full-Nyquist width are both FIXED - see
         * the BW badge's "6K3"/"96K", non-interactive, in
         * aux_row_display_draw()) - band_active stays 0 for those,
         * same as the project owner asked ("en WFM, NFM no se
         * muestra").
         *
         * full_span_hz: the SAME halving-per-zoom-step span
         * spec_span_labels_draw() uses for its tick ruler (192000 at
         * 1X, matching DEMOD_IF_OFFSET_HZ's own scale above) - needed
         * here too since the tint's WIDTH in pixels must shrink/grow
         * the same way the ruler's tick spacing (and the marker's
         * position) does when ZOOM changes what one pixel is worth in
         * Hz.
         *
         * AM is double-sideband: the tint straddles the center point
         * both ways, +/-bw_hz. USB only demodulates the UPPER
         * sideband: the tint extends RIGHT (higher frequency) only,
         * from the center point out to +bw_hz. LSB is the mirror:
         * LEFT only, -bw_hz to the center point - exactly the "a la
         * derecha o la izquierda" the project owner asked for.
         */
        {
            demod_mode_t mode = demod_am_get_mode();

            if (mode == DEMOD_MODE_AM || mode == DEMOD_MODE_USB || mode == DEMOD_MODE_LSB) {
                uint32_t full_span_hz;
                uint32_t bw_hz = k_audio_bw_hz[(uint8_t)demod_am_get_audio_bw()];
                int16_t bw_px;

                switch (s_spec_zoom) {
                case SPEC_ZOOM_2X: full_span_hz = 96000UL; break;
                case SPEC_ZOOM_4X: full_span_hz = 48000UL; break;
                case SPEC_ZOOM_8X: full_span_hz = 24000UL; break;
                case SPEC_ZOOM_1X:
                default:           full_span_hz = 192000UL; break;
                }
                bw_px = (int16_t)((uint32_t)SPEC_TRACE_W * bw_hz / full_span_hz);

                band_active = 1U;
                if (mode == DEMOD_MODE_USB) {
                    band_lo_offset_px = center_mark_offset_px;
                    band_hi_offset_px = (int16_t)(center_mark_offset_px + bw_px);
                } else if (mode == DEMOD_MODE_LSB) {
                    band_lo_offset_px = (int16_t)(center_mark_offset_px - bw_px);
                    band_hi_offset_px = center_mark_offset_px;
                } else { /* DEMOD_MODE_AM */
                    band_lo_offset_px = (int16_t)(center_mark_offset_px - bw_px);
                    band_hi_offset_px = (int16_t)(center_mark_offset_px + bw_px);
                }
            }
        }

        spectrum_draw(s_db_frame, FFT_BINS_IQ,
                      SPEC_TRACE_X, (uint16_t)(SPEC_Y + 4),
                      SPEC_TRACE_W,
                      (uint16_t)(SPEC_H - 4 - 20 - 2),
                      s_db_min, s_db_max,
                      center_mark_offset_px,
                      band_active, band_lo_offset_px, band_hi_offset_px);
    }
    t_spec1 = DWT->CYCCNT;

    t_wf0 = DWT->CYCCNT;
    /* Waterfall: one row of WATERFALL_WIDTH px, each column mapped to
     * its FFT bin, colored through the shared (LUT-backed) palette.
     * Uses the frame-averaged dB, so the waterfall inherits the same
     * noise smoothing as the trace. Blitted just inside the panel
     * border, same x origin as the spectrum trace so columns line up
     * vertically between the two views. */
    for (x = 0; x < WATERFALL_WIDTH; x++) {
        uint32_t bin = ((uint32_t)x * FFT_BINS_IQ) / WATERFALL_WIDTH;
        line[x] = spectrum_colormap(s_db_frame[bin], s_db_min, s_db_max);
    }
    waterfall_push_line(line);
    waterfall_blit(SPEC_TRACE_X, WF_Y);
    t_wf1 = DWT->CYCCNT;
    } /* !s_menu_open - see this block's opening comment above */

    /* Frame drawn (or skipped, if the menu covered it): restart the
     * accumulator for the next window either way. */
    s_db_count = 0U;

    s_frame_count++;
    if (!s_menu_open && (s_frame_count % 30U) == 0U) {
        uint32_t spec_us = (uint32_t)(((uint64_t)(t_spec1 - t_spec0)) * 1000000U / SystemCoreClock);
        uint32_t wf_us   = (uint32_t)(((uint64_t)(t_wf1 - t_wf0)) * 1000000U / SystemCoreClock);
        debug_print_dec("sdr_tick: last fft (us)", fft_us);
        debug_print_dec("sdr_tick: spectrum_draw (us)", spec_us);
        debug_print_dec("sdr_tick: waterfall push+blit (us)", wf_us);
        debug_print_dec("sdr_tick: frame TOTAL (us)", fft_us + spec_us + wf_us);
        debug_print_dec_signed("sdr_tick: I(left) min", i_min);
        debug_print_dec_signed("sdr_tick: I(left) max", i_max);
        debug_print_dec_signed("sdr_tick: Q(right) min", q_min);
        debug_print_dec_signed("sdr_tick: Q(right) max", q_max);
        {
            uint32_t stat = SPI_STAT(I2S1_ADD);
            if ((stat & SPI_STAT_FERR) != 0U) {
                debug_print("sdr_tick: *** SPI_STAT_FERR (format error) SET on I2S1_ADD ***\n");
            }
            if ((stat & SPI_STAT_RXORERR) != 0U) {
                debug_print("sdr_tick: *** SPI_STAT_RXORERR (receive overrun) SET on "
                            "I2S1_ADD ***\n");
            }
        }
    }
}

static void led_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
}

/*
 * --- Speaker PA enable/mute (PB7) ---------------------------------------
 *
 * Added 03/08/2026, per the project owner: a software mute for the
 * onboard speaker power amplifier, toggled from the HW-page tile in
 * the settings menu (see MENU_PAGE_HW in menu_grid_show()) - lets you
 * listen on headphones only, without the speaker PA also driving.
 *
 * *** PIN/POLARITY UNCONFIRMED (03/08/2026) ***
 * PB7 was given from memory ("PB7 creo" - not yet checked against a
 * schematic or bench-verified). Per this project's own established
 * debugging principle (see the AIC3204 I2S master/slave bring-up
 * history: never assume a fixed hardware mapping from memory, always
 * confirm against the datasheet/schematic or a live measurement)
 * this should be verified with a multimeter/scope BEFORE relying on
 * it - worst case if it's wrong is simply that the tile does nothing
 * audible (or drives an unrelated/floating pin), not damage, but it's
 * still unconfirmed. SPEAKER_PA_PIN below is the only place to change
 * if PB7 turns out wrong. SPEAKER_PA_ACTIVE_HIGH is the only place to
 * flip if the enable polarity turns out to be active-LOW instead
 * (defaulted here to active-HIGH, the more common convention for a
 * load-switch/amp EN pin - equally UNCONFIRMED).
 */
#define SPEAKER_PA_GPIO         GPIOB
#define SPEAKER_PA_GPIO_RCU     RCU_GPIOB
#define SPEAKER_PA_PIN          GPIO_PIN_7
#define SPEAKER_PA_ACTIVE_HIGH  1 /* 1 = pin HIGH enables the PA, 0 = active-LOW - see comment above */

static void speaker_pa_set_enabled(uint8_t on)
{
    s_speaker_pa_enabled = on ? 1U : 0U;
#if SPEAKER_PA_ACTIVE_HIGH
    if (s_speaker_pa_enabled) { gpio_bit_set(SPEAKER_PA_GPIO, SPEAKER_PA_PIN); }
    else                      { gpio_bit_reset(SPEAKER_PA_GPIO, SPEAKER_PA_PIN); }
#else
    if (s_speaker_pa_enabled) { gpio_bit_reset(SPEAKER_PA_GPIO, SPEAKER_PA_PIN); }
    else                      { gpio_bit_set(SPEAKER_PA_GPIO, SPEAKER_PA_PIN); }
#endif
}

static void speaker_pa_gpio_init(void)
{
    rcu_periph_clock_enable(SPEAKER_PA_GPIO_RCU);
    gpio_mode_set(SPEAKER_PA_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPEAKER_PA_PIN);
    gpio_output_options_set(SPEAKER_PA_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPEAKER_PA_PIN);
    /* Drive the pin to match s_speaker_pa_enabled's initializer above,
     * so the GPIO's actual level and the firmware's idea of the
     * speaker state agree from the very first instant it's an output
     * (rather than whatever the pin defaulted to before this ran). */
    speaker_pa_set_enabled(s_speaker_pa_enabled);
}

static void systick_delay_init(void)
{
    /* SysTick at a real 1ms, based on SystemCoreClock (updated by
     * SystemInit()/system_gd32f4xx.c). Timing-sensitive code such as
     * the panel's power-up sequence in rm68120_exmc.c relies on this
     * being calibrated correctly. */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        while (1) {
            /* SysTick configuration failed - should never happen */
        }
    }
}

void SysTick_Handler(void)
{
    g_msticks++;
    encoder_tick(); /* 1kHz quadrature/button sampling, see encoder.c */
}

/*
 * The default HardFault_Handler (Default_Handler, weakly defined in
 * the startup file) is a SILENT infinite loop. This version does
 * report over UART, so a real HardFault can be told apart from any
 * other kind of hang (e.g. an infinite loop in application code) just
 * by checking whether this message appears.
 *
 * It does not decode CFSR/HFSR (that needs a debugger attached and the
 * stack frame inspected) - for now it's just the "we ended up here"
 * signal, enough to confirm or rule out a hard fault as the cause.
 */
void HardFault_Handler(void)
{
    debug_print("\n*** HARDFAULT_HANDLER: a bus/access fault has occurred ***\n");
    while (1) {
        __NOP();
    }
}


