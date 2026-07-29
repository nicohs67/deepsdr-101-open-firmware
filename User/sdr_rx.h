#ifndef SDR_RX_H
#define SDR_RX_H

#include <stdint.h>

/*
 * Captures RX samples from the AIC3204 via I2S1_ADD (the full-duplex
 * extension of SPI1/I2S1, PB14=SDext), using circular DMA.
 *
 * DMA mapping: I2S1_ADD_RX -> DMA0, Channel 3, PERIEN[2:0]=011
 * (DMA_SUBPERI3). Confirmed against the GD32F4xx User Manual's DMA
 * request mapping table (Table 10-2, "Peripheral requests to DMA0").
 *
 * PING-PONG MODEL: a single circular buffer of 2*SDR_RX_BLOCK_SAMPLES
 * (stereo, 16-bit). When the DMA finishes the first half (HTF flag),
 * that half is complete and stable while the DMA writes into the
 * second half; when it completes the full circular round (FTF flag),
 * the second half is the one that's ready. sdr_rx_poll_block_iq()
 * tracks this and returns 0 if no new block is ready yet (non-blocking,
 * meant to be called from the main loop like the rest of this project).
 */

#define SDR_RX_BLOCK_SAMPLES   512U   /* mono samples per delivered block (= FFT size) */

/* Starts the capture DMA. Must be called AFTER
 * gd32_i2s_init_master_48k() (needs SPI1/I2S1_ADD already initialized
 * and enabled). */
void sdr_rx_init(void);

/*
 * If a new complete block is available since the last call, copies
 * SDR_RX_BLOCK_SAMPLES samples of I (left, IN2_L/IN2_R) into `i_out`
 * and of Q (right, IN3_R/IN3_L) into `q_out`, already de-interleaved,
 * and returns 1. If no new block is ready yet, returns 0 and leaves
 * both buffers untouched. Non-blocking.
 */
uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out);

#endif /* SDR_RX_H */
