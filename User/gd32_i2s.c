#include "gd32_i2s.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

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

void gd32_i2s2_isolation_test(void)
{
    /*
     * Plain GPIO toggle on PA4/PC10/PC12 first, same idea as the
     * toggle test above, to rule out a basic wiring issue on these
     * pins before interpreting the I2S2 isolation result.
     */
    {
        uint32_t i;

        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_GPIOC);

        gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10 | GPIO_PIN_12);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_12);

        debug_print("gd32_i2s2_test: GPIO toggle on PA4/PC10/PC12 (no I2S) - probe now\n");

        for (i = 0; i < 15; i++) {
            gpio_bit_set(GPIOA, GPIO_PIN_4);
            gpio_bit_set(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
            {
                volatile uint32_t d;
                for (d = 0; d < 500000U; d++) { __NOP(); }
            }
            gpio_bit_reset(GPIOA, GPIO_PIN_4);
            gpio_bit_reset(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
            {
                volatile uint32_t d;
                for (d = 0; d < 500000U; d++) { __NOP(); }
            }
        }
        debug_print("gd32_i2s2_test: GPIO toggle done, configuring I2S2 now\n");
    }

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI2);

    spi_i2s_deinit(SPI2);

    /* PA4=WS, PC10=CK, PC12=SD - free pins, unrelated to the AIC3204 */
    {
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_4);
    }
    {
        uint32_t pinsC = GPIO_PIN_10 | GPIO_PIN_12;
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, pinsC);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pinsC);
        gpio_af_set(GPIOC, GPIO_AF_5, pinsC);
    }

    /* PLLI2S is left at its reset value here (this test does not need
     * a specific rate). */
    i2s_init(SPI2, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI2, I2S_AUDIOSAMPLE_48K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_DISABLE);
    i2s_enable(SPI2);

    /*
     * A single dummy word only produces ~16 clock cycles (a couple of
     * microseconds at 6MHz) - essentially invisible without a
     * single-shot trigger. Feed data continuously instead, so the
     * isolation test result is actually interpretable on a scope.
     */
    {
        uint32_t i;
        uint32_t timeout_hits = 0;
        debug_print("gd32_i2s2_test: feeding dummy data continuously for ~3s - probe PC10 now\n");
        for (i = 0; i < 1200000U; i++) {
            uint32_t wait_cycles = 0;
            while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET) {
                wait_cycles++;
                if (wait_cycles > 100000U) { timeout_hits++; break; }
            }
            if (timeout_hits > 3) {
                debug_print("gd32_i2s2_test: TBE never clears - aborting loop\n");
                break;
            }
            spi_i2s_data_transmit(SPI2, (i & 1U) ? 0xAAAAU : 0x5555U);
        }
        debug_print("gd32_i2s2_test: continuous feed done\n");
    }

    debug_print_hex32("gd32_i2s2_test: SPI_I2SCTL(SPI2) after enable", SPI_I2SCTL(SPI2));
    debug_print_hex32("gd32_i2s2_test: SPI_I2SPSC(SPI2)", SPI_I2SPSC(SPI2));
    debug_print("gd32_i2s2_test: SPI2/I2S2 configured - WS=PA4 CK=PC10 SD=PC12 "
                "(no MCK). Check PC10 (CK) with a scope.\n");
}

void gd32_i2s_init_master_48k(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI1);

    debug_print_hex32("gd32_i2s: RCU_PLLI2S before spi_i2s_deinit", RCU_PLLI2S);

    /*
     * Clean reset of SPI1/I2S1/I2S1_ADD in case any state was
     * inherited from the bootloader or a previous configuration -
     * this board is chained from a bootloader rather than power-cycled
     * on every flash, so residual peripheral state is possible.
     */
    spi_i2s_deinit(SPI1);
    debug_print("gd32_i2s: SPI1/I2S1 reset via RCU before configuring\n");
    debug_print_hex32("gd32_i2s: RCU_PLLI2S after spi_i2s_deinit", RCU_PLLI2S);

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
     * Current values (PSC=8 in system_gd32f4xx.c, N=128/R=4 here):
     * i2sclock = (12.288MHz/8) * 128/4 = 49.152MHz exactly.
     */
    debug_print_hex32("gd32_i2s: RCU_PLLI2S before forcing N=128/R=4", RCU_PLLI2S);
    rcu_osci_off(RCU_PLLI2S_CK);
    if (rcu_plli2s_config(128U, 4U) != SUCCESS) {
        debug_print("gd32_i2s: rcu_plli2s_config(128,4) FAILED\n");
    }
    rcu_osci_on(RCU_PLLI2S_CK);
    if (rcu_osci_stab_wait(RCU_PLLI2S_CK) != SUCCESS) {
        debug_print("gd32_i2s: *** PLLI2S DID NOT LOCK (stab_wait ERROR) - no i2sclock ***\n");
    } else {
        debug_print("gd32_i2s: PLLI2S locked OK (stab_wait SUCCESS)\n");
    }
    debug_print_hex32("gd32_i2s: RCU_PLLI2S after forcing N=128/R=4", RCU_PLLI2S);

    i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);

    /*
     * Confirmed with a real oscilloscope measurement: MCLK=12.288MHz,
     * BCLK=1.536MHz, WCLK=48kHz - the fixed 256x/32x ratio that this
     * MCU's native MCKOUT block always produces for 16-bit words once
     * enabled. The real working sample rate is 48kHz.
     */
    i2s_psc_config(SPI1, I2S_AUDIOSAMPLE_48K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);

    debug_print_hex32("gd32_i2s: SPI_I2SCTL after i2s_init (SPI1)", SPI_I2SCTL(SPI1));
    debug_print_hex32("gd32_i2s: SPI_I2SPSC after i2s_psc_config (SPI1)", SPI_I2SPSC(SPI1));

    /* I2S1_ADD extension block: passed the SAME mode as the main block
     * (I2S_MODE_MASTERTX) - the library internally derives that the
     * extension block must be I2S_MODE_SLAVERX (shares the clock with
     * the master block, receive-only). This is the path the AIC3204's
     * I/Q samples arrive on (PB14), captured by DMA in sdr_rx.c. */
    i2s_full_duplex_mode_config(I2S1_ADD, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS,
                                 I2S_CKPL_LOW, I2S_FRAMEFORMAT_DT16B_CH16B);

    /*
     * GPIO/AF configuration must happen BEFORE i2s_enable(). If the
     * pins are still plain GPIO (e.g. left over from
     * gd32_i2s_pins_gpio_toggle_test) when the peripheral starts, the
     * internal clock generator can run with no signal reaching the
     * physical pins at all.
     *
     * WS, CK, SD(TX): AF5 on PB12/13/14/15 (SPI1/I2S1 block). PC6
     * (MCLK) is also AF5, confirmed wired to the AIC3204's MCLK pin.
     */
    {
        uint32_t pins = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);
        gpio_af_set(GPIOB, GPIO_AF_5, pins);
    }
    {
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
        gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_14);
    }
    {
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
        gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_6);
    }

    debug_print_hex32("gd32_i2s: GPIOB_CTL raw (pins 12-15 in bits[31:24], AF=10b)", GPIO_CTL(GPIOB));
    debug_print_hex32("gd32_i2s: GPIOB_AFSEL1 raw (should be 0x55550000 in nibbles 12-15)", GPIO_AFSEL1(GPIOB));
    debug_print_hex32("gd32_i2s: GPIOC_CTL raw (pin6 = bits[13:12], AF=10b)", GPIO_CTL(GPIOC));
    debug_print_hex32("gd32_i2s: GPIOC_AFSEL0 raw (pin6 = bits[27:24], should be 0x5)",
                       GPIO_AFSEL0(GPIOC));
    debug_print("gd32_i2s: PC6/MCLK enabled (MCKOUT). Confirmed on hardware: "
                "MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz.\n");

    i2s_enable(SPI1);
    i2s_enable(I2S1_ADD);

    /*
     * Known behavior on STM32/GD32-style I2S peripherals in
     * master-transmit mode: the internal clock generator can fail to
     * start until at least one data word is written to the data
     * register, even though I2SEN is already set. The loop below
     * feeds data continuously (not just once) for two reasons: a
     * single word only produces a couple of microseconds of clock
     * activity (invisible without a single-shot trigger), and this
     * loop doubles as a bring-up diagnostic - it proves the whole
     * clock chain end-to-end (comparison frequency, VCO lock, GPIO/AF
     * routing) by making the result directly observable both on a
     * scope and in this log.
     */
    {
        uint32_t i;
        uint32_t timeout_hits = 0;
        uint32_t wait_events = 0;   /* iterations that had to wait on TBE */
        uint32_t ch_left = 0, ch_right = 0; /* I2S channel-side flag samples */
        uint32_t t0, t1, elapsed_ms;

        /* DWT cycle counter, used to measure the loop's real duration */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        t0 = DWT->CYCCNT;

        debug_print("gd32_i2s: feeding dummy data continuously for a few seconds - probe now\n");
        for (i = 0; i < 1200000U; i++) {
            uint32_t wait_cycles = 0;
            if (spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET) {
                wait_events++;
            }
            while (spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET) {
                wait_cycles++;
                if (wait_cycles > 100000U) {
                    timeout_hits++;
                    break;
                }
            }
            if (timeout_hits > 3) {
                debug_print("gd32_i2s: TBE never clears (repeated timeout) - aborting "
                            "the feed loop so the rest of the system is not blocked\n");
                break;
            }
            spi_i2s_data_transmit(SPI1, (i & 1U) ? 0xAAAAU : 0x5555U);
            /* Sample the channel-side flag every so often: if the frame
             * counter is running, both values (left and right) should
             * show up over time. Sampling stride must be ODD (not a
             * multiple of 2), or it will alias onto a single
             * even/odd phase and always report the same channel. */
            if ((i % 1023U) == 0U) {
                if (spi_i2s_flag_get(SPI1, I2S_FLAG_CH) == SET) { ch_right++; }
                else { ch_left++; }
            }
        }
        t1 = DWT->CYCCNT;
        elapsed_ms = (uint32_t)(((uint64_t)(t1 - t0)) * 1000U / SystemCoreClock);

        debug_print("gd32_i2s: continuous feed done (or aborted on timeout)\n");
        /*
         * How to read this diagnostic:
         * - Non-zero wait_events, elapsed time matching the expected
         *   word rate, and BOTH ch_left/ch_right > 0 -> the internal
         *   clock is running and the shifter is transmitting; if
         *   there's still no signal on the scope, look at AF/pin
         *   routing next.
         * - elapsed time of only a few ms and wait_events = 0 -> TBE
         *   never cleared: writes to the data register are not
         *   actually being accepted by the peripheral.
         */
        debug_print_dec("gd32_i2s: loop duration (ms)", elapsed_ms);
        debug_print_dec("gd32_i2s: words written", i);
        debug_print_dec("gd32_i2s: wait_events (times we waited on TBE)", wait_events);
        debug_print_dec("gd32_i2s: left-channel samples", ch_left);
        debug_print_dec("gd32_i2s: right-channel samples", ch_right);
    }

    /*
     * Continuous audio requires circular DMA (see
     * gd32_i2s_dma_start_test_tone below) - the manual feed loop above
     * only produces clocks while it is actively writing; the clock
     * generator on this peripheral is gated by data availability, not
     * free-running.
     */

    debug_print_hex32("gd32_i2s: SPI_I2SCTL after i2s_enable (SPI1, bit10=I2SEN should be 1)",
                       SPI_I2SCTL(SPI1));
    debug_print_hex32("gd32_i2s: RCU_CFG0 raw (I2SSEL toggle bit is bit 23)", RCU_CFG0);

    debug_print("gd32_i2s: I2S1 (SPI1) master configured - WS=PB12 CK=PB13 SD=PB15 "
                "SDext=PB14, MCK=PC6. i2sclock=49.152MHz (N=128/R=4). "
                "Fs=48kHz, BCLK=1.536MHz, MCLK=12.288MHz (confirmed on hardware).\n");

    gd32_i2s_dma_start_test_tone();
}

/*
 * 1kHz sine tone table, stereo (L=R), 16-bit, at the real confirmed
 * frame rate (48kHz). 48000/1000 = 48 samples per cycle exactly, so
 * the table is short and there is no frequency rounding error.
 * Amplitude is moderate (not full-scale) to be safe on a speaker or
 * headphones during bring-up testing.
 */
#define TONE_SAMPLES_PER_CYCLE   48U
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

void gd32_i2s_dma_start_test_tone(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    tone_buf_fill_1khz();

    rcu_periph_clock_enable(RCU_DMA0);

    debug_print_hex32("gd32_i2s: SPI_STAT(SPI1) right before arming DMA "
                       "(bit1=TBE should be 1 if ready to write)",
                       SPI_STAT(SPI1));

    /* DMA0, Channel 4, PERIEN=000 (SUBPERI0) for SPI1_TX - confirmed
     * against the GD32F4xx User Manual's DMA request mapping table
     * (Table 10-2, "Peripheral requests to DMA0"). */
    dma_deinit(DMA0, DMA_CH4);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(SPI1);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_tone_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number              = TONE_SAMPLES_PER_CYCLE * 2U;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH4, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH4, DMA_SUBPERI0);

    dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);

    /* Enable the DMA transmit request on SPI1/I2S1 itself - without
     * this, the DMA channel is armed but the peripheral never
     * triggers it. */
    spi_dma_enable(SPI1, SPI_DMA_TRANSMIT);

    debug_print("gd32_i2s: circular DMA armed, 1kHz test tone should now be continuous "
                "(no longer stopping) on PB13/PB12, and audible on the AIC3204 output if "
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

        debug_print_hex32("gd32_i2s: SPI_CTL1(SPI1) after spi_dma_enable (bit1=DMATEN should be 1)",
                           SPI_CTL1(SPI1));
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
