#ifndef SDR_RX_H
#define SDR_RX_H

#include <stdint.h>

/*
 * Captures RX samples from the AIC3204 ADC via the MAIN SPI1/I2S1
 * block (PB15=SD), using circular DMA.
 *
 * BLOCK ROLES SWAPPED (28/07/2026): this used to read from the
 * I2S1_ADD extension (PB14) - disassembly of the original firmware's
 * own I2S setup, combined with the GD32F4xx User Manual's DMA request
 * mapping table showing both directions exist as distinct
 * subperipheral options on the same DMA channels, showed the ORIGINAL
 * design uses the MAIN block for RX and the extension for TX - the
 * opposite of what this driver assumed for a long time. See the
 * comments in gd32_i2s.c's gd32_i2s_init_slave_192k() for the full
 * reasoning.
 *
 * DMA mapping: SPI1_RX -> DMA0, Channel 3, PERIEN[2:0]=000
 * (DMA_SUBPERI0). Confirmed against the GD32F4xx User Manual's DMA
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
 * gd32_i2s_init_slave_192k() (needs SPI1/I2S1_ADD already initialized
 * and enabled). */
void sdr_rx_init(void);

/*
 * If a new complete block is available since the last call, copies
 * SDR_RX_BLOCK_SAMPLES samples from the left ADC channel into `i_out`
 * and from the right ADC channel into `q_out`, already de-interleaved,
 * and returns 1. As of the byte-for-byte real capture ported into
 * aic3204_phase2_init(), left = I (IN2_L/IN2_R differential) and
 * right = Q (IN3_R/IN3_L differential) - see that function's comments.
 * If no new block is ready yet, returns 0 and leaves both buffers
 * untouched. Non-blocking.
 */
uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out);

/*
 * Registers a hook called from the RX DMA interrupt for EVERY
 * completed block, with a pointer to the raw interleaved half (L/R =
 * I/Q, SDR_RX_BLOCK_SAMPLES frames). This is the real-time audio
 * path: unlike the polled display path, it never misses a block no
 * matter how long the main loop blocks. The hook runs in ISR context
 * - keep it bounded (the AM demodulator takes ~100us). Pass 0 to
 * unregister.
 */
void sdr_rx_set_block_hook(void (*hook)(const int16_t *raw_interleaved));

#endif /* SDR_RX_H */
