#ifndef GD32_I2S_H
#define GD32_I2S_H

#include <stdint.h>

/*
 * I2S1 (SPI1) peripheral configuration for the GD32F450, SLAVE mode,
 * full-duplex (main I2S1 block + I2S1_ADD extension block), for
 * 192kHz stereo I/Q with the AIC3204 codec.
 *
 * ARCHITECTURE (28/07/2026, corrected): a real I2C bus capture of the
 * original firmware's AIC3204 configuration shows register 0x1B=0x0C,
 * which sets BCLK and WCLK as OUTPUTS on the codec (confirmed against
 * a real TI forum example using this exact byte, and TI's SLAA404C
 * app note describing bits D3:D2 of this register as the BCLK/WCLK
 * direction control) - the codec is the I2S MASTER in this design,
 * not a slave. The GD32 must be configured as SLAVE to match. Running
 * both sides as master at the same time caused real, observed bus
 * contention on a real oscilloscope (BCLK/WCLK with irregular,
 * non-uniform pulse widths).
 *
 * STILL PENDING: MCLK is not generated at all right now. With the
 * codec expecting to be master, it needs a real MCLK to lock its own
 * PLL and generate a stable BCLK/WCLK - without it, expect BCLK/WCLK
 * to still be unstable even with the master/slave roles now correct.
 * This MCU's native I2S MCKOUT block can't produce the board's real
 * MCLK:BCLK:Fs ratio (8:32:1, fixed at 256x/32x instead) - the plan is
 * a general-purpose timer (TIMER2_CH0 on PC6 is available as an
 * alternate function on this pin) generating an independent 1.536MHz
 * square wave, pending confirmation of the exact AF number for that
 * function on this specific pin.
 *
 * Note: on this chip/board, the pins below belong to SPI1/I2S1, NOT to
 * SPI2/I2S2 - double-check this if porting from an STM32F4 reference,
 * since peripheral numbering on these specific pins differs.
 *
 * Pins (confirmed on real hardware):
 *   PB12 = WS   (I2S1_WS)        AF5 (now an INPUT - codec drives it)
 *   PB13 = BCLK (I2S1_CK)        AF5 (now an INPUT - codec drives it)
 *   PB15 = DOUT (I2S1_SD, TX)    AF5
 *   PB14 = DIN  (I2S1_ADD_SD,RX) AF6 (confirmed against the datasheet's
 *                                     Port B alternate function table -
 *                                     AF5 on this pin is SPI1_MISO, a
 *                                     different peripheral entirely)
 */
void gd32_i2s_init_slave_192k(void);

/*
 * Diagnostic utility: toggles the five I2S pins as plain GPIO (no I2S
 * peripheral involved) to isolate whether a "no signal" symptom is a
 * peripheral/clock issue or a physical wiring issue. `cycles` = number
 * of toggles (~100Hz each, visible on a scope or AC-coupled meter).
 */
void gd32_i2s_pins_gpio_toggle_test(uint32_t cycles);

/*
 * Starts a circular DMA channel that continuously feeds SPI1/I2S1 with
 * silence (all-zero samples). Called automatically at the end of
 * gd32_i2s_init_slave_192k(). Historically this was required to keep
 * this MCU's MASTER-mode clock generator running (it would stop
 * without continuous TX data) - now that SPI1/I2S1 is a SLAVE that
 * concern no longer applies, but this is kept as a reasonable default
 * TX source for whatever DAC path exists on the codec side.
 */
/*
 * Starts MCLK generation on PC6 via TIMER2_CH0 (AF2, confirmed against
 * the datasheet), targeting 1.536MHz - independent of the I2S1
 * peripheral entirely, since its native MCKOUT block can't produce
 * this board's real MCLK:BCLK:Fs ratio. Call this BEFORE
 * gd32_i2s_init_slave_192k(), so the codec has a valid MCLK reference
 * from the moment its own I2S clocking comes up.
 */
void gd32_i2s_mclk_timer_start(void);

void gd32_i2s_dma_start_silence(void);

/*
 * Starts a circular DMA channel that continuously feeds SPI1/I2S1 with
 * a 1kHz test tone (does not stop, unlike a manual feed loop would).
 * Not called automatically - call it manually for TX/DAC-side bring-up
 * testing (replaces the silence fed by gd32_i2s_dma_start_silence()).
 */
void gd32_i2s_dma_start_test_tone(void);

/*
 * Re-arms DMA0/CH4 as a circular 2-half ping-pong TX stream (2 x 512
 * stereo frames, starts silent), replacing the silence/test-tone
 * feed. This is the DAC-side transport for demodulated audio. Call
 * after the codec is fully configured (phase 2).
 */
void gd32_i2s_dma_start_stream(void);

/*
 * Copies one half's worth of stereo frames (512 frames = 1024 int16,
 * L/R interleaved) into whichever stream half is NOT currently being
 * played, derived from the live DMA position. Intended to be called
 * once per RX block from the RX DMA interrupt (see demod_am.c).
 */
void gd32_i2s_stream_write_half(const int16_t *stereo_frames);

#endif /* GD32_I2S_H */
