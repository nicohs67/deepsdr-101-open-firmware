#include "gd32f4xx.h"
#include "rm68120_exmc.h"
#include "debug_uart.h"
#include "gfx.h"
#include "ui.h"
#include "waterfall.h"
#include "touch.h"
#include "aic3204.h"
#include "gd32_i2s.h"
#include "sdr_rx.h"
#include "fft.h"
#include "spectrum.h"

static void led_gpio_init(void);
static void systick_delay_init(void);
static void demo_screen_draw(void);
static void sdr_spectrum_waterfall_tick(void);
static void demo_touch_poll(void);

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

    debug_print("Calling rm68120_init() for the first time this session...\n");
    rm68120_init();
    debug_print("rm68120_init() done\n");
    debug_print_hex32("RCU_PLLI2S after rm68120_init", RCU_PLLI2S);

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

    debug_print("\n--- I2S1: phase 3 (clocks + circular DMA, test tone) ---\n");
    gd32_i2s_init_master_48k();

    debug_print("\n--- AIC3204: phase 2 (clock + differential I/Q ADC + power-up) ---\n");
    aic3204_phase2_init();

    debug_print("\n--- SDR: phase 4 (real RX capture + FFT + spectrum/waterfall) ---\n");
    fft_init();
    sdr_rx_init();

#if CALIB_HEIGHT_TEST
    calib_height_ruler_draw();
#else
    demo_screen_draw();
#endif

    debug_print("main: entering the main loop\n");

    while (1) {
        gpio_bit_toggle(GPIOA, GPIO_PIN_8);

#if !CALIB_HEIGHT_TEST
        sdr_spectrum_waterfall_tick();
        demo_touch_poll();
#endif

        g_fill_count++;

        if ((g_fill_count % 50) == 0) {
            debug_print_dec("waterfall ticks", g_fill_count);
#if !CALIB_HEIGHT_TEST
            debug_print_dec("PENIRQ raw level (1=active/low)", touch_is_pressed());
            debug_print_dec("EXTI2 real triggers since startup", touch_irq_count());
#endif
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
 * Fixed demo screen: validates gfx.c/ui.c on real hardware (panel,
 * EXMC bus, orientation) and serves as a template for how to register
 * widgets in a ui_screen_t. Drawn once; the waterfall is updated
 * separately in sdr_spectrum_waterfall_tick().
 *
 * Layout (landscape 800x480 - confirmed on real hardware, see the
 * GFX_SCREEN_WIDTH/HEIGHT comment in gfx.h):
 *   - Title bar at the top (24px)
 *   - "Spectrum" frame
 *   - Waterfall area (WATERFALL_ROWS rows of WATERFALL_WIDTH px)
 *   - Example button row at the bottom
 *
 * Each zone's Y coordinate is defined as an internally-linked constant
 * (static const, not a macro) so sdr_spectrum_waterfall_tick() uses
 * exactly the same waterfall_y value as demo_screen_draw(), without
 * duplicating the calculation by hand.
 */
static const uint16_t DEMO_TITLE_H     = 24;
static const uint16_t DEMO_SPECTRUM_Y  = 24;
static const uint16_t DEMO_SPECTRUM_H  = 140;
static const uint16_t DEMO_WATERFALL_Y = 24 + 140 + 2;
static const uint16_t DEMO_BTN_Y       = 24 + 140 + 2 + WATERFALL_ROWS + 10;

/*
 * IMPORTANT: these widgets are static (not local to
 * demo_screen_draw()) on purpose. ui_screen_t only stores POINTERS to
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
static ui_button_t s_btn_menu;
static ui_button_t s_btn_tune;
static ui_button_t s_btn_mode;

/*
 * Example callback shared by the demo's 3 buttons. Fires on RELEASE
 * (the actual "click") - this is just a template for wiring app logic
 * to UI events.
 */
static void demo_button_callback(void *widget, ui_event_t event, void *user_data)
{
    ui_button_t *btn = (ui_button_t *)widget;
    (void)user_data;

    if (event == UI_EVENT_RELEASE) {
        debug_print("button pressed: ");
        debug_print(btn->label);
        debug_print("\n");
    }
}

static void demo_screen_draw(void)
{
    const uint16_t btn_w = 120, btn_h = 44, btn_gap = 12;

    gfx_fill_screen(GFX_COLOR_BLACK);
    ui_screen_init(&s_demo_screen);

    s_title_panel = (ui_panel_t){0, 0, GFX_SCREEN_WIDTH, DEMO_TITLE_H,
                                  GFX_COLOR_DARKGRAY, GFX_COLOR_DARKGRAY};
    ui_screen_add_panel(&s_demo_screen, &s_title_panel);

    s_spectrum_panel = (ui_panel_t){0, DEMO_SPECTRUM_Y, GFX_SCREEN_WIDTH, DEMO_SPECTRUM_H,
                                     GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_spectrum_panel);

    s_waterfall_panel = (ui_panel_t){0, (uint16_t)(DEMO_WATERFALL_Y - 2), GFX_SCREEN_WIDTH,
                                      (uint16_t)(WATERFALL_ROWS + 4), GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_waterfall_panel);

    /* enabled=1 set explicitly - if omitted, a freshly declared
     * ui_button_t defaults to enabled=0 (C zero-initialization) and
     * ui_screen_touch() will always ignore it even though it draws
     * fine. */
    s_btn_menu = (ui_button_t){btn_gap, DEMO_BTN_Y, btn_w, btn_h, "MENU",
                                GFX_COLOR_WHITE, GFX_COLOR_BLUE, GFX_COLOR_WHITE,
                                1, 0, 1, demo_button_callback, NULL};
    s_btn_tune = (ui_button_t){(uint16_t)(btn_gap * 2 + btn_w), DEMO_BTN_Y, btn_w, btn_h, "TUNE",
                                GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_WHITE,
                                1, 0, 1, demo_button_callback, NULL};
    s_btn_mode = (ui_button_t){(uint16_t)(btn_gap * 3 + btn_w * 2), DEMO_BTN_Y, btn_w, btn_h, "MODE",
                                GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_BLACK,
                                1, 1, 1, demo_button_callback, NULL};
    ui_screen_add_button(&s_demo_screen, &s_btn_menu);
    ui_screen_add_button(&s_demo_screen, &s_btn_tune);
    ui_screen_add_button(&s_demo_screen, &s_btn_mode);

    ui_screen_draw(&s_demo_screen);

    /* Loose text that doesn't need to be interactive or survive after
     * being drawn: gfx_text()/ui_label_draw() can still be called
     * directly, bypassing the screen, for things that never change or
     * receive touches. */
    gfx_text(4, 4, "DEEPSDR - GFX/UI DEMO", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, 2);
    gfx_text(4, (uint16_t)(DEMO_SPECTRUM_Y + 4), "SPECTRUM",
              GFX_COLOR_GREEN, GFX_COLOR_BLACK, 1);
    gfx_line(0, (uint16_t)(DEMO_SPECTRUM_Y + DEMO_SPECTRUM_H - 20),
              GFX_SCREEN_WIDTH - 1, (uint16_t)(DEMO_SPECTRUM_Y + DEMO_SPECTRUM_H - 20),
              GFX_COLOR_DARKGRAY);
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
 */
static void demo_touch_poll(void)
{
    uint16_t x = 0, y = 0;
    uint8_t pressed = touch_read(&x, &y);

    ui_screen_touch(&s_demo_screen, x, y, pressed);

    if (touch_irq_pending()) {
        debug_print("touch: PENIRQ edge (new contact) -> ");
        touch_debug_raw();
    }
}

/*
 * SDR_DB_MIN/MAX define an UNCALIBRATED working dB range - the "dB"
 * value comes from a bit-manipulation log2 approximation (see fft.c),
 * not a referenced measurement. Good enough to show something
 * reasonable on screen from the start, but these two numbers should
 * be re-tuned once a real reference signal is available, by looking
 * at what s_db values actually come out in practice.
 */
static const float SDR_DB_MIN = -10.0f;
static const float SDR_DB_MAX = 90.0f;

/* sdr_rx.h and fft.h define their sizes independently - if one is ever
 * changed without the other, a clear compile error is better than a
 * silent overflow of s_rx_i/s_rx_q. */
#if SDR_RX_BLOCK_SAMPLES != FFT_SIZE
#error "SDR_RX_BLOCK_SAMPLES (sdr_rx.h) and FFT_SIZE (fft.h) must match"
#endif

static int16_t s_rx_i[SDR_RX_BLOCK_SAMPLES];
static int16_t s_rx_q[SDR_RX_BLOCK_SAMPLES];
static float   s_db[FFT_BINS_USEFUL];

/*
 * Q-CHANNEL VERIFICATION: I (IN2_L/R) is already confirmed on real
 * hardware. Q (IN3_R/L) is still an extrapolated register value (see
 * aic3204.c) pending independent verification. Set this define to 1
 * to show Q instead of I on the spectrum/waterfall - if injecting a
 * known signal into IN3_R/IN3_L (with nothing on IN2) shows up on
 * screen the same way IN2 used to, that confirms Q routing is
 * correct. Min/max of BOTH channels are always printed regardless of
 * which one is shown, so the numeric evidence doesn't require looking
 * at the screen.
 */
#define SDR_SHOW_CHANNEL_Q   0   /* 0 = show I (IN2), 1 = show Q (IN3) */

/* debug_uart.h has no signed decimal print - I/Q sample min/max are
 * int16_t and can be negative, hence this small local helper instead
 * of touching the UART module for it. */
static void debug_print_dec_signed(const char *label, int32_t val)
{
    if (val < 0) {
        debug_print(label);
        debug_print(" = -");
        debug_print_dec("", (uint32_t)(-val));
    } else {
        debug_print_dec(label, (uint32_t)val);
    }
}

/*
 * Replaces the earlier synthetic-gradient waterfall demo with the real
 * capture -> FFT -> spectrum/waterfall pipeline. Non-blocking: if
 * sdr_rx_poll_block_iq() has no new block yet, this tick does nothing
 * (the rest of the main loop - touch, UI - stays just as responsive).
 */
static void sdr_spectrum_waterfall_tick(void)
{
    static uint16_t line[WATERFALL_WIDTH];
    static uint32_t s_block_count = 0U;
    static uint32_t s_t_prev_block = 0U;
    static uint8_t  s_have_prev = 0U;
    uint32_t t_fft0, t_fft1, t_spec0, t_spec1, t_wf0, t_wf1, t_now;
    uint16_t x, n;
    int16_t i_min, i_max, q_min, q_max;
    const int16_t *show_block;

    if (sdr_rx_poll_block_iq(s_rx_i, s_rx_q) == 0U) {
        return;
    }

    /* Min/max of BOTH channels, every block - the numeric evidence for
     * whether IN2 and IN3 are seeing different things (real signal on
     * each) or whether Q is "dead" (min=max=0, or pinned at a fixed
     * value), which would indicate the P1_R55/P1_R57 routing is not
     * actually reaching the right ADC despite the register write
     * being ACKed. */
    i_min = s_rx_i[0]; i_max = s_rx_i[0];
    q_min = s_rx_q[0]; q_max = s_rx_q[0];
    for (n = 1; n < SDR_RX_BLOCK_SAMPLES; n++) {
        if (s_rx_i[n] < i_min) { i_min = s_rx_i[n]; }
        if (s_rx_i[n] > i_max) { i_max = s_rx_i[n]; }
        if (s_rx_q[n] < q_min) { q_min = s_rx_q[n]; }
        if (s_rx_q[n] > q_max) { q_max = s_rx_q[n]; }
    }

#if SDR_SHOW_CHANNEL_Q
    show_block = s_rx_q;
#else
    show_block = s_rx_i;
#endif

    /* Profiling: measure with the DWT cycle counter (same mechanism as
     * gd32_i2s.c) how long each stage of the pipeline takes, plus the
     * real elapsed time between blocks - if the observed period is
     * much larger than fft+spectrum+waterfall combined, the bottleneck
     * is elsewhere in the main loop (touch, systick, etc), not in this
     * tick. Printed for only 1 in 20 blocks to avoid flooding the UART
     * or skewing the very timing being measured. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    t_now = DWT->CYCCNT;

    t_fft0 = DWT->CYCCNT;
    fft_compute_db(show_block, s_db);
    t_fft1 = DWT->CYCCNT;

    t_spec0 = DWT->CYCCNT;

    /* Instantaneous spectrum, inside the panel already reserved by the
     * UI (see DEMO_SPECTRUM_Y/H) - a margin is left so it doesn't
     * overlap the "SPECTRUM" label or the separator line below it */
    spectrum_draw(s_db, FFT_BINS_USEFUL,
                  2, (uint16_t)(DEMO_SPECTRUM_Y + 18),
                  (uint16_t)(GFX_SCREEN_WIDTH - 4),
                  (uint16_t)(DEMO_SPECTRUM_H - 18 - 20 - 2),
                  SDR_DB_MIN, SDR_DB_MAX);
    t_spec1 = DWT->CYCCNT;

    t_wf0 = DWT->CYCCNT;
    /* Waterfall: one row of WATERFALL_WIDTH px, each column mapped
     * (nearest neighbor) to its corresponding FFT bin */
    for (x = 0; x < WATERFALL_WIDTH; x++) {
        uint32_t bin = ((uint32_t)x * FFT_BINS_USEFUL) / WATERFALL_WIDTH;
        line[x] = spectrum_colormap(s_db[bin], SDR_DB_MIN, SDR_DB_MAX);
    }
    waterfall_push_line(line);
    waterfall_blit(0, DEMO_WATERFALL_Y);
    t_wf1 = DWT->CYCCNT;

    s_block_count++;
    if ((s_block_count % 20U) == 0U) {
        uint32_t fft_us  = (uint32_t)(((uint64_t)(t_fft1 - t_fft0)) * 1000000U / SystemCoreClock);
        uint32_t spec_us = (uint32_t)(((uint64_t)(t_spec1 - t_spec0)) * 1000000U / SystemCoreClock);
        uint32_t wf_us   = (uint32_t)(((uint64_t)(t_wf1 - t_wf0)) * 1000000U / SystemCoreClock);
        debug_print_dec("sdr_tick: fft (us)", fft_us);
        debug_print_dec("sdr_tick: spectrum_draw (us)", spec_us);
        debug_print_dec("sdr_tick: waterfall push+blit (us)", wf_us);
        debug_print_dec("sdr_tick: TOTAL measured (us)", fft_us + spec_us + wf_us);
        if (s_have_prev) {
            uint32_t period_us = (uint32_t)(((uint64_t)(t_now - s_t_prev_block)) * 1000000U / SystemCoreClock);
            debug_print_dec("sdr_tick: real PERIOD between blocks (us) - compare with "
                            "TOTAL above", period_us);
        }
        /* Q-channel verification: min/max of each channel, every 20
         * blocks. If Q(IN3) is dead (min=max, or pinned at a fixed
         * value while I(IN2) moves), that's the signal that the
         * extrapolated P1_R55/P1_R57 routing is not actually reaching
         * the right ADC. Inject a known signal into IN3_R/IN3_L (with
         * IN2 quiet) and check that these numbers DO change. */
        debug_print_dec_signed("sdr_tick: I(IN2) min", i_min);
        debug_print_dec_signed("sdr_tick: I(IN2) max", i_max);
        debug_print_dec_signed("sdr_tick: Q(IN3) min", q_min);
        debug_print_dec_signed("sdr_tick: Q(IN3) max", q_max);
    }
    s_t_prev_block = t_now;
    s_have_prev = 1U;
}

static void led_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
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
