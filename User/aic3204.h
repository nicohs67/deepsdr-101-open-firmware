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
 * Phase 2 init: clock (MCLK fed directly to CODEC_CLKIN, no PLL) +
 * differential input routing (I=IN2_L/IN2_R, confirmed on real
 * hardware; Q=IN3_R/IN3_L, extrapolated from the register bit pattern
 * of an earlier Arduino driver for this same board and confirmed at
 * the architecture level against TI's SLAA557 Application Reference
 * Guide - the exact register value is still pending independent
 * verification) + ADC power-up.
 *
 * Confirmed with an oscilloscope: MCLK=12.288MHz, BCLK=1.536MHz,
 * WCLK=48kHz - no longer dependent on any formula prediction.
 * NADC=1/MADC=2/AOSR=128 are calculated from these real, measured
 * values (formula: CODEC_CLKIN = NADC x MADC x AOSR x ADC_FS =
 * 1x2x128x48000 = 12,288,000, exact).
 *
 * Call this after aic3204_probe_and_reset() AND after the I2S side is
 * already generating a real clock towards the codec
 * (gd32_i2s_init_master_48k already executed) - phase 2 configures
 * clock registers that depend on MCLK/BCLK/WCLK already running.
 */
void aic3204_phase2_init(void);

#endif /* AIC3204_H */
