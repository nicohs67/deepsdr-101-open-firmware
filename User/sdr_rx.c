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

void sdr_rx_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    s_last_half_delivered = 0U;

    rcu_periph_clock_enable(RCU_DMA0);

    /*
     * I2S1_ADD_RX -> DMA0, Channel 3, DMA_SUBPERI3 - confirmed against
     * the DMA request mapping table (same source that resolved the TX
     * channel). Direction is PERIPH_TO_MEMORY (reading from the
     * peripheral into RAM), the opposite of the test tone's TX path.
     */
    dma_deinit(DMA0, DMA_CH3);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(I2S1_ADD);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_raw_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.number              = RAW_TOTAL_WORDS;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH3, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH3, DMA_SUBPERI3);

    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_HTF);
    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_FTF);
    dma_channel_enable(DMA0, DMA_CH3);

    /* Enable the DMA receive request on I2S1_ADD itself - without
     * this, the channel is armed but nothing ever triggers it (the
     * mirror image of the same mistake we had on the TX side). */
    spi_dma_enable(I2S1_ADD, SPI_DMA_RECEIVE);

    debug_print("sdr_rx: DMA0 CH3 (I2S1_ADD_RX) armed in circular ping-pong mode, "
                "block = ");
    debug_print_dec("sdr_rx: SDR_RX_BLOCK_SAMPLES", SDR_RX_BLOCK_SAMPLES);
}

uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out)
{
    uint32_t n;

    /* First half (offset 0): ready when HTF fires and it wasn't
     * already the last one delivered. */
    if ((s_last_half_delivered != 1U) &&
        (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_HTF) == SET)) {
        dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_HTF);
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            /* de-interleave: even=left (I, IN2), odd=right (Q, IN3) */
            i_out[n] = s_raw_buf[2U * n];
            q_out[n] = s_raw_buf[2U * n + 1U];
        }
        s_last_half_delivered = 1U;
        return 1U;
    }

    /* Second half (offset RAW_HALF_WORDS): ready when FTF fires. */
    if ((s_last_half_delivered != 2U) &&
        (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_FTF) == SET)) {
        dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_FTF);
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            i_out[n] = s_raw_buf[RAW_HALF_WORDS + 2U * n];
            q_out[n] = s_raw_buf[RAW_HALF_WORDS + 2U * n + 1U];
        }
        s_last_half_delivered = 2U;
        return 1U;
    }

    return 0U;
}
