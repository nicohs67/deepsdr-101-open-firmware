#ifndef AIC3204_H
#define AIC3204_H

#include <stdint.h>

/*
 * TLV320AIC3204 codec driver.
 *
 * The AIC3204 organizes its registers into "pages": register 0x00 on
 * ANY page is the "Page Control Register" - writing it selects the
 * active page, and reading it always returns the currently selected
 * page. This gives a free closed-loop test to validate the I2C link
 * without touching any audio register: write a page, read it back,
 * and it must match.
 *
 * I2C address: defaults to 0x18 with the AIC3204's MODE pin tied to
 * ground (the most common breakout configuration), or 0x19 if tied to
 * VDD. If aic3204_init() doesn't detect 0x18, use aic3204_scan_bus()
 * to find the real address before assuming the chip isn't responding.
 */

#define AIC3204_ADDR_DEFAULT 0x18U

/* AIC3204 nRESET - PB11, active low (confirmed on real hardware).
 * aic3204_init() releases it before touching the I2C bus - without
 * this the chip never responds reliably. Defined inside aic3204.c,
 * not here, to avoid pulling gd32f4xx.h into this header. */

/* Call once at startup: initializes the I2C bus (i2c_bitbang) and
 * gets the driver ready to use the given address (normally
 * AIC3204_ADDR_DEFAULT). Does not touch the chip yet - that's
 * aic3204_probe_and_reset(). */
void aic3204_init(uint8_t i2c_addr);

/* Selects `page` and writes `value` to `reg` within that page.
 * Returns 1 if both writes (page select + register) were ACKed, 0 if
 * either failed. */
uint8_t aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t value);

/* Selects `page` and reads `reg` within that page into `*value`.
 * Returns 1 if the whole operation (page write + register read) was
 * ACKed, 0 on failure (in which case *value is not valid). */
uint8_t aic3204_read_reg(uint8_t page, uint8_t reg, uint8_t *value);

/*
 * Full bring-up test, meant to run once at startup and dump its
 * result over UART (uses debug_print internally):
 *   1. Page Control Register closed-loop test (write page 1, read
 *      back, write page 0, read back) - confirms the I2C link works,
 *      without assuming anything about the rest of the chip.
 *   2. Software reset (page 0, register 0x01 = 0x01) - the chip
 *      returns to its reset values; there is no direct way to read
 *      back confirmation of this (the reset bit is self-clearing),
 *      but if the closed-loop test in step 1 already succeeded, the
 *      reset is essentially guaranteed to have worked too.
 * Returns 1 if step 1 (the only directly verifiable part) succeeded.
 */
uint8_t aic3204_probe_and_reset(void);

/* Scans the I2C bus (addresses 0x08-0x77) and prints over UART
 * (debug_print) every address that ACKs. Useful if
 * aic3204_probe_and_reset() fails and the real address needs
 * confirming instead of assuming 0x18. */
void aic3204_scan_bus(void);

/*
 * Phase 2 init: the AIC3204's clock/PLL, ADC input routing, and ADC
 * power-up, ported byte-for-byte (138 register writes, exact order)
 * from a REAL I2C bus capture of the original, known-working
 * firmware's own configuration of this same codec on this same board
 * - not derived, not extrapolated, not guessed.
 *
 * This capture revealed the actual clock architecture: the AIC3204's
 * own PLL is enabled, clocked from BCLK (not a separate MCLK signal).
 * Confirmed by exact arithmetic against oscilloscope measurements:
 * BCLK=6.144MHz, PLL J=14/D=0 -> PLL_CLK=86.016MHz=CODEC_CLKIN, which
 * both the ADC (NADC=1/MADC=7/AOSR=64) and DAC (NDAC=2/MDAC=7/DOSR=32)
 * divider chains reduce to exactly Fs=192kHz. See gd32_i2s.c - no
 * MCLK pin is configured at all now, since the codec doesn't need one
 * for this design.
 *
 * Call this after aic3204_probe_and_reset() AND after the I2S side is
 * already generating a real clock towards the codec
 * (gd32_i2s_init_slave_192k already executed) - phase 2 configures
 * clock registers that depend on BCLK/WCLK already running.
 */
void aic3204_phase2_init(void);

/*
 * Sets the DAC digital output volume, both L/R channels identically.
 * Range -63.5dB to +24dB in 0.5dB steps (silently clamped) - the
 * AIC3204's native DAC digital volume control (Page 0, R65/R66).
 * aic3204_phase2_init() leaves both at 0dB (unity, the byte-exact
 * captured baseline); call this afterwards to move away from that.
 * Does NOT touch the analog HPL/HPR headphone driver gain (Page 1,
 * R16/R17) - see the comment above this function's definition in
 * aic3204.c for why. Returns 1 if both L/R writes were ACKed.
 */
uint8_t aic3204_set_volume_db(float db);

/*
 * Sets the MIC_PGA analog input gain (Page 1, R59/R60), both L/R
 * channels identically - the stage BEFORE the ADC, unlike
 * aic3204_set_volume_db() above (which is after the DAC, on the
 * output side). Range 0dB to 47.5dB in 0.5dB steps (silently
 * clamped, unsigned - there's no attenuation direction, only gain).
 * aic3204_phase2_init() leaves both at 20dB (0x28, the byte-exact
 * captured baseline); call this afterwards to move away from that.
 * Returns 1 if both L/R writes were ACKed.
 */
uint8_t aic3204_set_pga_gain_db(float db);

/*
 * TEMPORARY DIAGNOSTIC (28/07/2026): TI documents a digital "Audio Bus
 * Loopback" mode (Page 0 / Register 29, bit D5) that reflects
 * whatever comes in on DIN straight back out on DOUT, entirely
 * bypassing the ADC/DAC converters - designed specifically to verify
 * host<->codec digital bus communication in isolation from the analog
 * front-end. Set to 1 to enable it: if a known TX pattern comes back
 * correctly on RX, the whole digital I2S/DMA path is proven working
 * end-to-end and the remaining problem is isolated to the ADC analog
 * front-end. If RX is still stuck even in loopback, the bug is in the
 * digital bus framing/format itself. Set back to 0 for normal
 * operation. Also read by gd32_i2s.c, to automatically feed a real TX
 * pattern instead of silence while this is enabled (silence looped
 * back is still silence - not a useful test).
 */
#define AIC3204_TEST_LOOPBACK 0

#endif /* AIC3204_H */
