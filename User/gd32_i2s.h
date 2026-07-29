#ifndef GD32_I2S_H
#define GD32_I2S_H

#include <stdint.h>

/*
 * I2S1 (SPI1) peripheral configuration for the GD32F450, master mode,
 * full-duplex (main I2S1 block + I2S1_ADD extension block), for 48kHz
 * stereo I/Q with the AIC3204 codec.
 *
 * Note: on this chip/board, the pins below belong to SPI1/I2S1, NOT to
 * SPI2/I2S2 - double-check this if porting from an STM32F4 reference,
 * since peripheral numbering on these specific pins differs.
 *
 * Pins (confirmed on real hardware):
 *   PB12 = WS   (I2S1_WS)        AF5
 *   PB13 = BCLK (I2S1_CK)        AF5
 *   PB15 = DOUT (I2S1_SD, TX)    AF5
 *   PB14 = DIN  (I2S1_ADD_SD,RX) AF6
 *   PC6  = MCLK (I2S1_MCK)       AF5
 *
 * Confirmed clock relationship (measured with an oscilloscope):
 * MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz - the fixed 256x/32x ratio
 * that this MCU's native I2S MCKOUT block always produces for 16-bit
 * words, once MCKOUT is enabled.
 */
void gd32_i2s_init_master_48k(void);

/*
 * Isolation test utility: configures the OTHER I2S block on the chip
 * (SPI2/I2S2) on pins unrelated to the AIC3204 (PA4=WS, PC10=CK,
 * PC12=SD, no MCK). Useful to check whether a clocking issue is
 * specific to the SPI1/I2S1 block or a more general clock-tree
 * problem. Not called from the main init path.
 */
void gd32_i2s2_isolation_test(void);

/*
 * Diagnostic utility: toggles the five I2S pins as plain GPIO (no I2S
 * peripheral involved) to isolate whether a "no signal" symptom is a
 * peripheral/clock issue or a physical wiring issue. `cycles` = number
 * of toggles (~100Hz each, visible on a scope or AC-coupled meter).
 */
void gd32_i2s_pins_gpio_toggle_test(uint32_t cycles);

/*
 * Starts a circular DMA channel that continuously feeds SPI1/I2S1 with
 * a 1kHz test tone (does not stop, unlike a manual feed loop would).
 * Called automatically at the end of gd32_i2s_init_master_48k() - no
 * need to call it separately.
 */
void gd32_i2s_dma_start_test_tone(void);

#endif /* GD32_I2S_H */
