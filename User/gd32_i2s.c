#include "gd32_i2s.h"
#include "gd32f4xx.h"
#include "debug_uart.h"
#include "aic3204.h" /* only for the shared AIC3204_TEST_LOOPBACK diagnostic
                      * flag - temporary coupling, see the note below where
                      * it's used. */

static void tone_buf_fill_1khz(void);
static float sinf_approx(float x);

/*
 * Diagnostic utility: toggles the five I2S pins as plain GPIO output
 * (peripheral untouched). Useful to isolate a "no signal" symptom
 * between a peripheral/clock configuration issue and a physical
 * wiring issue - if even a plain GPIO toggle doesn't show up on a
 * scope, the problem is upstream of the I2S peripheral entirely.
 */
void gd32_i2s_pins_gpio_toggle_test(uint32_t cycles)
{
    uint32_t i;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    {
        uint32_t pinsB = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pinsB);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pinsB);
        gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    }

    debug_print("gd32_i2s: GPIO toggle test on PB12/13/14/15 and PC6 "
                "(no I2S) - probe now\n");

    for (i = 0; i < cycles; i++) {
        gpio_bit_set(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_bit_set(GPIOC, GPIO_PIN_6);
        {
            volatile uint32_t d;
            for (d = 0; d < 500000U; d++) {
                __NOP();
            }
        }
        gpio_bit_reset(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_bit_reset(GPIOC, GPIO_PIN_6);
        {
            volatile uint32_t d;
            for (d = 0; d < 500000U; d++) {
                __NOP();
            }
        }
    }

    debug_print("gd32_i2s: GPIO toggle test done\n");
}

void gd32_i2s_init_slave_192k(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI1);

    /*
     * Clean reset of SPI1/I2S1/I2S1_ADD in case any state was
     * inherited from the bootloader or a previous configuration -
     * this board is chained from a bootloader rather than power-cycled
     * on every flash, so residual peripheral state is possible.
     */
    spi_i2s_deinit(SPI1);

    /*
     * PLLI2S must be configured explicitly (not left at its reset/
     * inherited value) to get a known, reproducible i2sclock.
     *
     * IMPORTANT: the PSC field used here (via rcu_plli2s_config's M
     * divider) is the SAME field as the main system PLL's PSC in
     * system_gd32f4xx.c. If that PSC value ever changes, N/R here must
     * be recalculated together with it to keep the same i2sclock -
     * they are not independent.
     *
     * Current values (PSC=8 in system_gd32f4xx.c, N=400/R=4 here):
     * i2sclock = (12.288MHz/8) * 400/4 = 49.152MHz exactly.
     */
    rcu_osci_off(RCU_PLLI2S_CK);
    if (rcu_plli2s_config(400U, 4U) != SUCCESS) {
        debug_print("gd32_i2s: rcu_plli2s_config(400,4) FAILED\n");
    }
    rcu_osci_on(RCU_PLLI2S_CK);
    if (rcu_osci_stab_wait(RCU_PLLI2S_CK) != SUCCESS) {
        debug_print("gd32_i2s: *** PLLI2S DID NOT LOCK (stab_wait ERROR) - no i2sclock ***\n");
    }

    /*
     * BLOCK ROLES SWAPPED (28/07/2026): a real I2C capture combined
     * with disassembly of the original firmware's own I2S setup code
     * shows the struct tied to the SPI1_BASE literal being initialized
     * with Mode=0x100 - which matches I2S_MODE_SLAVERX exactly (this
     * project's own I2SCTL_I2SOPMOD(1) encoding). That means the
     * ORIGINAL firmware configures the MAIN SPI1/I2S1 block as the
     * RECEIVE side (ADC capture, on PB15/SD) and the I2S1_ADD
     * extension as the TRANSMIT side (DAC feed, on PB14/SDext) - the
     * OPPOSITE of what this driver assumed for a very long time (main
     * block = TX, extension = RX). The GD32F4xx User Manual's DMA0
     * request mapping table confirms both halves of this swap exist
     * as distinct, real options on the exact same DMA channels we
     * were already using (DMA0/CH3: SPI1_RX at SUBPERI0 vs
     * I2S1_ADD_RX at SUBPERI3; DMA0/CH4: I2S1_ADD_TX at SUBPERI2 vs
     * SPI1_TX at SUBPERI0) - meaning this whole time, ADC samples were
     * very plausibly being captured from the wrong subperipheral of
     * the right DMA channel, which would explain why no amount of
     * clock, DMA priority, or master/slave fixing on the OTHER half
     * ever changed anything: the real ADC data path was never the one
     * being read from at all.
     *
     * SLAVE mode (still applies, unrelated to this swap): a real I2C
     * capture of the original firmware's AIC3204 configuration shows
     * register 0x1B=0x0C, which (confirmed against a real TI forum
     * example using this exact byte, plus TI's own SLAA404C app note
     * describing bits D3:D2 of this register as the BCLK/WCLK
     * direction control) sets BOTH BCLK and WCLK as OUTPUTS on the
     * codec - i.e. the codec is the I2S master in this design. The
     * GD32 must be the SLAVE to match (confirmed clean, single-source
     * BCLK/WCLK on a real oscilloscope once this was corrected).
     */
    i2s_init(SPI1, I2S_MODE_SLAVERX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);

    /*
     * MCLK: now resolved - see gd32_i2s_mclk_timer_start(), called
     * before this function from main.c. Confirmed on a real
     * oscilloscope: MCLK=1.536MHz, BCLK=6.144MHz, WCLK=192kHz, exactly
     * matching the real board's measured values.
     */
    i2s_psc_config(SPI1, I2S_AUDIOSAMPLE_192K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_DISABLE);

    /* I2S1_ADD extension block: passed the SAME mode as the main block
     * (I2S_MODE_SLAVERX) - the library internally derives the OPPOSITE
     * role for the extension block, I2S_MODE_SLAVETX in this case
     * (shares the clock with the master - now the CODEC, not the GD32
     * - transmit-only). This is now the path that feeds the AIC3204's
     * DAC (silence/test tone), captured... err, driven, by DMA in
     * gd32_i2s_dma_start_silence()/gd32_i2s_dma_start_test_tone()
     * below, via PB14 (SDext). The MAIN block above is now the one
     * capturing ADC samples on PB15 (SD), via DMA in sdr_rx.c. */
    i2s_full_duplex_mode_config(I2S1_ADD, I2S_MODE_SLAVERX, I2S_STD_PHILLIPS,
                                 I2S_CKPL_LOW, I2S_FRAMEFORMAT_DT16B_CH16B);

    /*
     * GPIO/AF configuration must happen BEFORE i2s_enable(). If the
     * pins are still plain GPIO (e.g. left over from
     * gd32_i2s_pins_gpio_toggle_test) when the peripheral starts, the
     * internal clock generator can run with no signal reaching the
     * physical pins at all.
     *
     * WS, CK, SD(TX): AF5 on PB12/13/14/15 (SPI1/I2S1 block). PC6
     * (MCLK) is intentionally NOT configured - see the MCKOUT note
     * above.
     */
    {
        uint32_t pins = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);
        gpio_af_set(GPIOB, GPIO_AF_5, pins);
    }
    {
        /*
         * AF6 for I2S1_ADD_SD (RX data in), confirmed against the
         * GD32F450xx datasheet's Port B alternate function table:
         * PB14 lists SPI1_MISO on AF5 and I2S1_ADD_SD on AF6.
         */
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
        gpio_af_set(GPIOB, GPIO_AF_6, GPIO_PIN_14);
    }

    i2s_enable(SPI1);
    i2s_enable(I2S1_ADD);

    /*
     * REMOVED (28/07/2026): this loop used to write dummy TX data to
     * SPI1 to prime this MCU's old MASTER-mode clock generator. Now
     * that SPI1/I2S1 is a SLAVE **and** the RECEIVE side of the swap
     * above, writing to its data register doesn't make sense at all
     * (SPI_DATA(SPI1) is now an input path) - removed rather than
     * adapted, since none of its original rationale applies anymore.
     */

    debug_print_hex32("gd32_i2s: SPI_I2SCTL after i2s_enable (SPI1, bit10=I2SEN should be 1)",
                       SPI_I2SCTL(SPI1));
    debug_print_hex32("gd32_i2s: RCU_CFG0 raw (I2SSEL toggle bit is bit 23)", RCU_CFG0);

    debug_print("gd32_i2s: I2S1 SLAVE configured, roles SWAPPED to match the original "
                "firmware - SPI1(main)=RX (ADC capture, PB15/SD), I2S1_ADD=TX (DAC feed, "
                "PB14/SDext). WS=PB12 CK=PB13, MCK=PC6 via TIMER2_CH0 (confirmed on "
                "hardware: MCLK=1.536MHz, BCLK=6.144MHz, WCLK=192kHz, matching the real "
                "board exactly).\n");

#if AIC3204_TEST_LOOPBACK
    /* Loopback test needs a real, recognizable TX pattern - silence
     * looped back is still silence, not a useful test. */
    gd32_i2s_dma_start_test_tone();
#else
    gd32_i2s_dma_start_silence();
#endif
}

/*
 * 1kHz sine tone table, stereo (L=R), 16-bit, at the real confirmed
 * frame rate (192kHz). 192000/1000 = 192 samples per cycle exactly, so
 * the table is short and there is no frequency rounding error.
 * Amplitude is moderate (not full-scale) to be safe on a speaker or
 * headphones during bring-up testing.
 */
#define TONE_SAMPLES_PER_CYCLE   192U
static int16_t s_tone_buf[TONE_SAMPLES_PER_CYCLE * 2U]; /* *2: interleaved L/R */

static void tone_buf_fill_1khz(void)
{
    uint32_t n;
    for (n = 0; n < TONE_SAMPLES_PER_CYCLE; n++) {
        /* Amplitude ~+-8000 (25% of 16-bit full scale). Computed once
         * at init time (not in a hot path), so a compile-time LUT
         * isn't needed - a runtime float fill is fine here. */
        float angle = (2.0f * 3.14159265358979f * (float)n) / (float)TONE_SAMPLES_PER_CYCLE;
        int16_t sample = (int16_t)(8000.0f * sinf_approx(angle));
        s_tone_buf[2U * n]      = sample; /* left */
        s_tone_buf[2U * n + 1U] = sample; /* right (same tone on both) */
    }
}

/* Sine approximation with no libm dependency (this project links with
 * nano.specs and no -lm): Bhaskara I approximation, typical error
 * <0.2% across the full range - more than enough for an audible test
 * tone. */
static float sinf_approx(float x)
{
    float pi = 3.14159265358979f;
    float x2;
    int negate = 0;
    while (x > pi) { x -= 2.0f * pi; }
    while (x < -pi) { x += 2.0f * pi; }
    if (x < 0.0f) { x = -x; negate = 1; }
    x2 = (16.0f * x * (pi - x)) / (5.0f * pi * pi - 4.0f * x * (pi - x));
    return negate ? -x2 : x2;
}

/*
 * IMPORTANT: this MCU's I2S master clock generator only produces
 * continuous BCLK/WS while TX data keeps flowing - it is NOT
 * free-running (see the manual feed loop in
 * gd32_i2s_init_slave_192k()). Since I2S1_ADD (RX) is wired as a
 * SLAVE sharing the SAME BCLK/WS lines as the TX master block, RX
 * capture silently stops the moment BCLK/WS stop - even though RX
 * itself has nothing to do with the TX audio content. A continuous TX
 * feed is therefore required to keep the whole bus alive, not just to
 * produce audio output. Feeding silence (all zeros) instead of the
 * audible test tone keeps BCLK/WS running without an audible tone.
 */
static const int16_t s_silence_buf[16] = {0}; /* small, doesn't need to match any tone period */

/*
 * MCLK generation via TIMER2_CH0 on PC6, AF2 - confirmed against the
 * GD32F450xx datasheet's Port C alternate function table (Table 2-8),
 * cross-checked against the already-empirically-confirmed AF5=I2S1_MCK
 * on the same row.
 *
 * Targets exactly 1.536MHz, matching the real board's confirmed MCLK
 * frequency. With this project's clock config (PSC=8/PLLN=260/PLLP=2
 * in system_gd32f4xx.c -> SYSCLK=199.68MHz, APB1_PSC=4 -> PCLK1=
 * 49.92MHz -> TIMER2CLK=2xPCLK1=99.84MHz when APB1 prescaler != 1,
 * standard on this family): 99.84MHz / 65 = 1.536MHz exactly. PSC=0,
 * period=64 (65 total counts) gives that division with no remainder -
 * a clean, exact match, not a rounded approximation.
 *
 * Duty cycle is close to but not exactly 50% (32/65 ~= 49.2%) since 65
 * is odd and can't split evenly - a minor asymmetry that should not
 * matter for a clock reference input.
 */
void gd32_i2s_mclk_timer_start(void)
{
    timer_parameter_struct timer_init_struct;
    timer_oc_parameter_struct oc_init_struct;

    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_TIMER2);

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_6);

    timer_deinit(TIMER2);

    timer_struct_para_init(&timer_init_struct);
    timer_init_struct.prescaler         = 0U;
    timer_init_struct.alignedmode       = TIMER_COUNTER_EDGE;
    timer_init_struct.counterdirection  = TIMER_COUNTER_UP;
    timer_init_struct.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init_struct.period            = 64U; /* 65 counts: 99.84MHz/65 = 1.536MHz exact */
    timer_init_struct.repetitioncounter = 0U;
    gd32_timer_init(TIMER2, &timer_init_struct);

    timer_channel_output_struct_para_init(&oc_init_struct);
    oc_init_struct.outputstate  = TIMER_CCX_ENABLE;
    oc_init_struct.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_channel_output_config(TIMER2, TIMER_CH_0, &oc_init_struct);

    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 32U); /* ~49.2% duty */
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM0);

    timer_auto_reload_shadow_enable(TIMER2);
    timer_enable(TIMER2);

    debug_print("gd32_i2s: MCLK started via TIMER2_CH0/PC6 (AF2), target 1.536MHz "
                "exact - verify with a scope before relying on it\n");
}


void gd32_i2s_dma_start_silence(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA0);

    /* DMA0, Channel 4, PERIEN=010 (SUBPERI2) for I2S1_ADD_TX - now the
     * TX side after the block-role swap (see gd32_i2s_init_slave_192k)
     * - confirmed against the GD32F4xx User Manual's DMA request
     * mapping table (Table 10-2, "Peripheral requests to DMA0"). Same
     * channel as gd32_i2s_dma_start_test_tone(), just a silent source
     * buffer. */
    dma_deinit(DMA0, DMA_CH4);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(I2S1_ADD);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_silence_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number              = 16U;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH4, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH4, DMA_SUBPERI2);

    dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);

    spi_dma_enable(I2S1_ADD, SPI_DMA_TRANSMIT);
}

/*
 * --- TX audio stream (demodulated audio -> DAC) ---------------------
 *
 * A 2-half ping-pong buffer played by the same circular DMA channel
 * (DMA0/CH4) that the silence/test-tone paths use. Each half holds
 * exactly one RX block's worth of stereo frames: since RX and TX are
 * clocked by the SAME codec-driven BCLK/WS at the same rate, every
 * completed RX half corresponds to exactly one TX half consumed -
 * the two sides cannot drift, only the phase offset at startup
 * varies, and gd32_i2s_stream_write_half() re-derives the safe half
 * from the live DMA position on every call, so it self-corrects.
 */
#define STREAM_FRAMES_PER_HALF 512U /* MUST equal SDR_RX_BLOCK_SAMPLES */
#define STREAM_WORDS_PER_HALF  (STREAM_FRAMES_PER_HALF * 2U) /* stereo */
#define STREAM_TOTAL_WORDS     (STREAM_WORDS_PER_HALF * 2U)

static int16_t s_stream_buf[STREAM_TOTAL_WORDS]; /* zero-initialized: silence */

void gd32_i2s_dma_start_stream(void)
{
    dma_single_data_parameter_struct dma_init_struct;
    uint32_t w;

    for (w = 0; w < STREAM_TOTAL_WORDS; w++) {
        s_stream_buf[w] = 0;
    }

    rcu_periph_clock_enable(RCU_DMA0);

    /* Take over DMA0/CH4 from whatever fed it before (silence or the
     * test tone): disable cleanly, wait for CHEN to drop, deinit,
     * re-arm over the stream buffer. */
    dma_channel_disable(DMA0, DMA_CH4);
    while ((DMA_CHCTL(DMA0, DMA_CH4) & DMA_CHXCTL_CHEN) != 0U) {
        /* a disable request can take a few cycles to complete */
    }
    dma_deinit(DMA0, DMA_CH4);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(I2S1_ADD);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_stream_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number              = STREAM_TOTAL_WORDS;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH4, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH4, DMA_SUBPERI2);
    dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);
    spi_dma_enable(I2S1_ADD, SPI_DMA_TRANSMIT);

    debug_print("gd32_i2s: TX stream DMA armed (2x512 stereo frames, starts silent)\n");
}

void gd32_i2s_stream_write_half(const int16_t *stereo_frames)
{
    /* Remaining count -> current playback position -> half being
     * PLAYED right now; write the OTHER one. Called once per RX block
     * (every 2.67ms) from the RX DMA interrupt. */
    uint32_t remaining = dma_transfer_number_get(DMA0, DMA_CH4);
    uint32_t pos = STREAM_TOTAL_WORDS - remaining;
    uint32_t playing_half = (pos < STREAM_WORDS_PER_HALF) ? 0U : 1U;
    uint32_t write_half = 1U - playing_half;
    int16_t *dst = &s_stream_buf[write_half * STREAM_WORDS_PER_HALF];
    uint32_t w;

    for (w = 0; w < STREAM_WORDS_PER_HALF; w++) {
        dst[w] = stereo_frames[w];
    }
}

void gd32_i2s_dma_start_test_tone(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    tone_buf_fill_1khz();

    rcu_periph_clock_enable(RCU_DMA0);

    debug_print_hex32("gd32_i2s: SPI_STAT(I2S1_ADD) right before arming DMA "
                       "(bit1=TBE should be 1 if ready to write)",
                       SPI_STAT(I2S1_ADD));

    /* DMA0, Channel 4, PERIEN=010 (SUBPERI2) for I2S1_ADD_TX - now the
     * TX side after the block-role swap - confirmed against the
     * GD32F4xx User Manual's DMA request mapping table (Table 10-2,
     * "Peripheral requests to DMA0"). */
    dma_deinit(DMA0, DMA_CH4);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(I2S1_ADD);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_tone_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number              = TONE_SAMPLES_PER_CYCLE * 2U;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH4, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH4, DMA_SUBPERI2);

    dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);

    /* Enable the DMA transmit request on I2S1_ADD itself - without
     * this, the DMA channel is armed but the peripheral never
     * triggers it. */
    spi_dma_enable(I2S1_ADD, SPI_DMA_TRANSMIT);

    debug_print("gd32_i2s: circular DMA armed, 1kHz test tone should now be continuous "
                "(no longer stopping) on PB14, and audible on the AIC3204 output if "
                "phase 1 (I2C) left the codec unmuted\n");

    /*
     * Bring-up diagnostic: reads the DMA channel's remaining transfer
     * count twice, separated by a delay. If it doesn't change, the
     * channel is armed but inert (wrong DMA channel/subperipheral, or
     * DMATEN did not actually take); if it decrements, the channel is
     * firing and any remaining issue is elsewhere in the chain. Also
     * checks the DMATEN readback and the channel's error flags
     * (FEE/SDE/TAE), which would indicate an invalid
     * channel/subperipheral selection some other way.
     */
    {
        uint32_t cnt_a, cnt_b;
        volatile uint32_t d;

        debug_print_hex32("gd32_i2s: SPI_CTL1(I2S1_ADD) after spi_dma_enable (bit1=DMATEN should be 1)",
                           SPI_CTL1(I2S1_ADD));
        debug_print_hex32("gd32_i2s: DMA_CHCTL(DMA0,CH4) raw (bit0=CHEN should be 1)",
                           DMA_CHCTL(DMA0, DMA_CH4));

        cnt_a = dma_transfer_number_get(DMA0, DMA_CH4);
        for (d = 0; d < 2000000U; d++) { __NOP(); }
        cnt_b = dma_transfer_number_get(DMA0, DMA_CH4);

        debug_print_dec("gd32_i2s: DMA_CHCNT remaining, reading A", cnt_a);
        debug_print_dec("gd32_i2s: DMA_CHCNT remaining, reading B (after delay)", cnt_b);
        if (cnt_a == cnt_b) {
            debug_print("gd32_i2s: *** DMA_CHCNT NOT MOVING - channel is armed but INERT, "
                        "never triggers a request (check the DMA channel/subperipheral "
                        "selection against the request mapping table) ***\n");
        } else {
            debug_print("gd32_i2s: DMA_CHCNT IS decrementing - the channel is firing "
                        "requests, look elsewhere in the chain\n");
        }

        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FEE) == SET) {
            debug_print("gd32_i2s: *** DMA flag FEE (FIFO error/exception) set ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_SDE) == SET) {
            debug_print("gd32_i2s: *** DMA flag SDE (single data mode exception) set ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_TAE) == SET) {
            debug_print("gd32_i2s: *** DMA flag TAE (transfer access error) set - channel/"
                        "subperipheral is very likely misconfigured ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_HTF) == SET) {
            debug_print("gd32_i2s: DMA flag HTF (half transfer) set - "
                        "the channel HAS transferred at least half the buffer\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF) == SET) {
            debug_print("gd32_i2s: DMA flag FTF (full transfer) set - "
                        "the channel HAS completed at least one circular round\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_HTF) == RESET &&
            dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF) == RESET &&
            cnt_a == cnt_b) {
            debug_print("gd32_i2s: neither HTF nor FTF ever set - the channel has not "
                        "executed a single transfer since it was armed\n");
        }
    }
}
