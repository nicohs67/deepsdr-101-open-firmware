#include "sdr_rx.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

/*
 * Raw circular buffer (stereo, L/R interleaved), twice the delivered
 * block size: first half + second half, ping-pong via the DMA
 * channel's own HTF/FTF flags (same non-blocking poll pattern used in
 * gd32_i2s_dma_start_test_tone(), consistent with the rest of the
 * project).
 */
#define RAW_HALF_WORDS   (SDR_RX_BLOCK_SAMPLES * 2U) /* 16-bit words per half (L/R interleaved) */
#define RAW_TOTAL_WORDS  (RAW_HALF_WORDS * 2U)

static int16_t s_raw_buf[RAW_TOTAL_WORDS];

/* 0 = no half delivered yet; 1 = the first half was the last one
 * delivered (waiting on FTF for the second); 2 = the second half was
 * the last one delivered (waiting on HTF for the first). */
static uint8_t s_last_half_delivered;

/*
 * INTERRUPT-DRIVEN DELIVERY (30/07/2026): the HTF/FTF flags are now
 * consumed by the DMA0_Channel3 interrupt handler, not polled - the
 * main loop blocks for longer than one 2.67ms block while drawing a
 * spectrum frame, which is fine for the display (skipping blocks just
 * drops frames) but fatal for AUDIO. The ISR calls the registered
 * block hook (the AM demodulator) synchronously with a pointer to the
 * completed raw half, so the audio path is immune to whatever the
 * main loop is doing. It also leaves a "pending half" note that
 * sdr_rx_poll_block_iq() consumes for the display path (latest wins;
 * halves the display misses are simply skipped, as before).
 */
static void (*s_block_hook)(const int16_t *raw_interleaved) = 0;
static volatile uint8_t  s_pending_half = 0xFFU; /* 0xFF = none */
static volatile uint32_t s_hook_runs = 0U;       /* diagnostics */

void sdr_rx_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    s_last_half_delivered = 0U;

    /*
     * Clear a possible RXORERR (receive overrun) accumulated in the
     * gap between i2s_enable(SPI1) (in gd32_i2s_init_slave_192k())
     * and this function running. Real bits can start arriving as soon
     * as the peripheral is enabled, and if nothing reads them out
     * before a second word arrives, RXORERR sets - once set, the
     * receive data path can stop updating entirely until it's
     * explicitly cleared. This is now called IMMEDIATELY after
     * gd32_i2s_init_slave_192k() (see the reordering note in main.c)
     * specifically to keep this gap as small as possible - previously
     * aic3204_phase2_init()'s many bit-banged I2C writes ran first,
     * leaving SPI1's receive path unserviced for long enough that
     * RXORERR fired continuously and this clear sequence could never
     * catch a quiet moment to actually take effect.
     *
     * On this silicon (like the STM32 family it closely mirrors),
     * RXORERR is cleared by a genuine CPU read of SPI_DATA followed by
     * a read of SPI_STAT - a DMA-initiated read of SPI_DATA does NOT
     * clear it.
     */
    {
        volatile uint32_t discard;
        uint32_t stat_before, stat_after;

        stat_before = SPI_STAT(SPI1);
        discard = SPI_DATA(SPI1);
        (void)discard;
        stat_after = SPI_STAT(SPI1);

        debug_print_hex32("sdr_rx: SPI_STAT(SPI1) before RXORERR clear sequence",
                           stat_before);
        debug_print_hex32("sdr_rx: SPI_STAT(SPI1) after RXORERR clear sequence",
                           stat_after);
        if ((stat_before & SPI_STAT_RXORERR) != 0U && (stat_after & SPI_STAT_RXORERR) == 0U) {
            debug_print("sdr_rx: RXORERR was SET and the clear sequence worked - it is "
                        "now clear\n");
        } else if ((stat_before & SPI_STAT_RXORERR) != 0U) {
            debug_print("sdr_rx: RXORERR did not clear via read sequence - trying a full "
                        "disable/re-enable of SPI1 instead\n");
            /*
             * A plain read-sequence not working could mean this
             * silicon needs a harder reset of the receive path, not
             * just the STM32-style read-DATA-then-read-STAT
             * convention. Disabling and re-enabling I2SEN forces the
             * whole shifter/status logic back to a known state.
             */
            i2s_disable(SPI1);
            {
                volatile uint32_t d;
                for (d = 0; d < 1000U; d++) { __NOP(); }
            }
            i2s_enable(SPI1);
            {
                volatile uint32_t d;
                for (d = 0; d < 1000U; d++) { __NOP(); }
            }
            discard = SPI_DATA(SPI1);
            (void)discard;
            stat_after = SPI_STAT(SPI1);
            debug_print_hex32("sdr_rx: SPI_STAT(SPI1) after disable/re-enable + clear",
                               stat_after);
            if ((stat_after & SPI_STAT_RXORERR) == 0U) {
                debug_print("sdr_rx: disable/re-enable cleared RXORERR\n");
            } else {
                debug_print("sdr_rx: *** RXORERR STILL set after disable/re-enable - this "
                            "is not a simple stale-flag issue ***\n");
            }
        }
    }

    rcu_periph_clock_enable(RCU_DMA0);

    /*
     * SPI1_RX -> DMA0, Channel 3, DMA_SUBPERI0 - confirmed against
     * the DMA request mapping table (same source that resolved the TX
     * channel). Direction is PERIPH_TO_MEMORY (reading from the
     * peripheral into RAM), the opposite of the test tone's TX path.
     */
    dma_deinit(DMA0, DMA_CH3);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(SPI1);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_raw_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.number              = RAW_TOTAL_WORDS;
    /*
     * ULTRA_HIGH (the highest available level) rather than HIGH,
     * deliberately ABOVE the TX silence-keeper channel's priority
     * (DMA0/CH4, still HIGH) - both channels share DMA0, and RXORERR
     * has persisted identically across every clock/codec/loopback
     * configuration tried so far, which stopped looking like a
     * transient timing issue and started looking structural. If bus
     * arbitration between the two DMA0 channels was starving RX often
     * enough to cause a PERSISTENT overrun regardless of what else
     * changes, giving RX strictly higher priority should fix it; if
     * RXORERR still won't clear even with this, DMA priority is ruled
     * out too.
     */
    dma_init_struct.priority            = DMA_PRIORITY_ULTRA_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH3, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH3, DMA_SUBPERI0);

    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_HTF);
    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_FTF);

    /* Half/full-transfer interrupts drive block delivery (see the
     * ISR-model comment at the top of this file). NVIC preemption
     * priority 6: below nothing critical, above the main loop. */
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE);
    nvic_irq_enable(DMA0_Channel3_IRQn, 6U, 0U);
    dma_channel_enable(DMA0, DMA_CH3);

    /* Enable the DMA receive request on SPI1 itself - without
     * this, the channel is armed but nothing ever triggers it (the
     * mirror image of the same mistake we had on the TX side). */
    spi_dma_enable(SPI1, SPI_DMA_RECEIVE);

    debug_print("sdr_rx: DMA0 CH3 (SPI1_RX) armed in circular ping-pong mode, "
                "block = ");
    debug_print_dec("sdr_rx: SDR_RX_BLOCK_SAMPLES", SDR_RX_BLOCK_SAMPLES);

    /*
     * Bring-up diagnostic: this channel has never actually been
     * confirmed to fire requests (unlike the TX test tone's channel,
     * which went through this same check). Captured I/Q data staying
     * pinned at a fixed value (-1) even after fixing PB14's AF makes
     * this worth checking directly: read the remaining transfer count
     * twice with a delay in between, and check the HTF/FTF/error
     * flags, exactly like the TX diagnostic in gd32_i2s.c.
     */
    {
        uint32_t cnt_a, cnt_b;
        volatile uint32_t d;

        debug_print_hex32("sdr_rx: SPI_CTL1(SPI1) after spi_dma_enable "
                           "(bit0=DMAREN should be 1)", SPI_CTL1(SPI1));
        debug_print_hex32("sdr_rx: DMA_CHCTL(DMA0,CH3) raw (bit0=CHEN should be 1)",
                           DMA_CHCTL(DMA0, DMA_CH3));

        cnt_a = dma_transfer_number_get(DMA0, DMA_CH3);
        for (d = 0; d < 2000000U; d++) { __NOP(); }
        cnt_b = dma_transfer_number_get(DMA0, DMA_CH3);

        debug_print_dec("sdr_rx: DMA_CHCNT remaining, reading A", cnt_a);
        debug_print_dec("sdr_rx: DMA_CHCNT remaining, reading B (after delay)", cnt_b);
        if (cnt_a == cnt_b) {
            debug_print("sdr_rx: *** DMA_CHCNT NOT MOVING - the RX channel is armed but "
                        "INERT, never triggers a request ***\n");
        } else {
            debug_print("sdr_rx: DMA_CHCNT IS decrementing - the RX channel is firing "
                        "requests; if I/Q data is still stuck, the issue is upstream "
                        "(SPI1 not actually receiving real bits, or the AIC3204 ADC "
                        "not producing real samples)\n");
        }
        if (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_TAE) == SET) {
            debug_print("sdr_rx: *** DMA flag TAE (transfer access error) set ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_FEE) == SET) {
            debug_print("sdr_rx: *** DMA flag FEE (FIFO error/exception) set ***\n");
        }

        /*
         * Never checked until now: SPI1's own status register.
         * SPI_STAT_FERR ("format error - only used in I2S mode, used
         * to check audio format synchronization") is exactly the flag
         * that would confirm or rule out the frame-format-mismatch
         * hypothesis directly, instead of continuing to guess at
         * register 0x1B's bit layout. SPI_STAT_RXORERR (receive
         * overrun) is also worth ruling out, even though DMA has
         * plenty of margin at 192kHz.
         */
        {
            uint32_t stat = SPI_STAT(SPI1);
            debug_print_hex32("sdr_rx: SPI_STAT(SPI1) raw (bit0=RBNE, bit6=RXORERR, "
                               "bit8=FERR)", stat);
            if ((stat & SPI_STAT_FERR) != 0U) {
                debug_print("sdr_rx: *** SPI_STAT_FERR (format error) SET - SPI1 "
                            "reports the audio format is NOT synchronized with what's "
                            "arriving on the bus - this points directly at a frame "
                            "format/standard mismatch, not the ADC or DMA ***\n");
            }
            if ((stat & SPI_STAT_RXORERR) != 0U) {
                debug_print("sdr_rx: *** SPI_STAT_RXORERR (receive overrun) SET - data is "
                            "arriving faster than it's being read out ***\n");
            }
            if ((stat & SPI_STAT_RBNE) != 0U) {
                debug_print("sdr_rx: SPI_STAT_RBNE is set right now - a received word is "
                            "sitting in the data register waiting to be read\n");
            }
        }
    }
}

void sdr_rx_set_block_hook(void (*hook)(const int16_t *raw_interleaved))
{
    s_block_hook = hook;
}

/*
 * DMA0 Channel 3 (SPI1_RX) interrupt: fires on half-transfer (first
 * half of the raw buffer complete) and full-transfer (second half
 * complete). While this handler processes half N, the DMA is filling
 * the OTHER half, so the data is stable for the duration.
 *
 * Priority note: enabled at NVIC preemption priority 6 (see
 * sdr_rx_init()) - the demod hook costs on the order of 100us per
 * block, so even where it delays the 1kHz SysTick encoder sampling,
 * the quadrature transition-table decoder tolerates that jitter by
 * construction.
 */
void DMA0_Channel3_IRQHandler(void)
{
    const int16_t *half = 0;

    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_HTF) == SET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_HTF);
        half = &s_raw_buf[0];
        s_pending_half = 0U;
    }
    if (dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF) == SET) {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
        half = &s_raw_buf[RAW_HALF_WORDS];
        s_pending_half = 1U;
    }

    if ((half != 0) && (s_block_hook != 0)) {
        s_block_hook(half);
        s_hook_runs++;
    }
}

uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out)
{
    uint32_t n;
    uint8_t half;
    uint32_t base;
    uint32_t primask;

    /* Take (and clear) the ISR's pending-half note atomically. */
    primask = __get_PRIMASK();
    __disable_irq();
    half = s_pending_half;
    s_pending_half = 0xFFU;
    __set_PRIMASK(primask);

    if (half == 0xFFU) {
        return 0U;
    }

    /* NOTE: while we de-interleave half N here, the DMA is writing
     * the other half - unless the main loop was so late that the ISR
     * already flipped again, in which case a torn block is possible.
     * That was equally true of the old flag-polling version, and for
     * a DISPLAY consumer a rare torn spectrum frame is acceptable;
     * the audio path (the hook, in ISR context) is never affected. */
    base = (half == 0U) ? 0U : RAW_HALF_WORDS;
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
        /* de-interleave: even=left ADC channel, odd=right ADC channel */
        i_out[n] = s_raw_buf[base + 2U * n];
        q_out[n] = s_raw_buf[base + 2U * n + 1U];
    }
    (void)s_last_half_delivered; /* superseded by the ISR model */
    return 1U;
}
