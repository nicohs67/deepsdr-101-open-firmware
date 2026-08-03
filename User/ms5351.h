#ifndef MS5351_H
#define MS5351_H

#include <stdint.h>

/*
 * MS5351M clock generator driver (Si5351A register-compatible clone).
 *
 * Role in this SDR: quadrature local oscillator for the QSD (quadrature
 * sampling detector). CLK0 and CLK1 both run at the LO frequency from
 * PLLB, with CLK0 phase-offset by exactly one output-divider count,
 * which at an even divider N means a delay of N/(4*fVCO) = 1/(4*fOUT)
 * = 90 degrees. Classic Si5351 quadrature-LO trick, no external
 * divide-by-4 Johnson counter needed. That trick only reaches down to
 * ~4.8MHz, though - below that, ms5351_set_lo_freq() switches to a
 * second technique entirely (a ported, community-vetted design, not a
 * capture from this board) extending coverage down to 100kHz - see
 * that function's comment below for which does what, and
 * ms5351_set_lo_freq_lowband()'s comment in ms5351.c for the low-band
 * technique's own derivation.
 *
 * Everything in ms5351_init() and ms5351_tune_captured() is a
 * byte-exact replay of an I2C capture taken from the original firmware
 * of this board (capture: captura_i2c_sdr.txt, device address 0x60).
 * Decoded facts from that capture:
 *
 *   - Reference: 26MHz (inferred: PLLA is set to x32 integer = 832MHz
 *     and MS2 to /104, which lands on exactly 8.000MHz only with a
 *     26MHz reference; a 25MHz one would give an implausible
 *     7.6923MHz).
 *   - PLLA = 832MHz fixed, integer mode. Only used by CLK2.
 *   - CLK2 = 8MHz from PLLA/MS2, fully configured but left POWERED
 *     DOWN (reg18 = 0xCC, PDN bit set) and not output-enabled. Likely
 *     a TX clock or auxiliary reference the original firmware only
 *     turns on when needed. We replicate it as-is; see
 *     ms5351_clk2_8mhz() to switch it on.
 *   - PLLB = fractional, retuned on every LO change. The captured tune
 *     is PLLB = 908.000MHz / MS0 = MS1 = /10 -> LO = 90.800MHz (an FM
 *     broadcast frequency - clearly a reception test). Note 908MHz is
 *     marginally above the 900MHz datasheet VCO max; MS5351 clones
 *     take this without complaint, and the capture proves this board
 *     does.
 *   - Quadrature: CLK0 phase offset (reg165) = output divider (10),
 *     CLK1 offset (reg166) = 0, then PLLB soft reset (reg177 = 0x80)
 *     to latch the new phase relationship.
 *
 * I2C: uses the shared bit-banged bus (i2c_bitbang). ms5351_init()
 * does NOT initialize the bus itself - call it after i2c_bitbang_init()
 * has already run (in main.c the AIC3204 phase-1 probe does that).
 */

#define MS5351_ADDR      0x60U
#define MS5351_XTAL_HZ   26000000UL

/* The LO frequency encoded in the captured tune block, for reference
 * and for validating ms5351_set_lo_freq() against known-good bytes:
 * ms5351_set_lo_freq(MS5351_CAPTURED_LO_HZ) with divider 10 reproduces
 * the capture bit-exactly. */
#define MS5351_CAPTURED_LO_HZ 90800000UL

/*
 * Boot-time base configuration, byte-exact replay of the captured
 * startup block: outputs off, CLK0-2 powered down, 8pF crystal load,
 * PLLA = x32 integer (832MHz), PLL reset, MS2 = /104 (8MHz, left
 * powered down), output-enable mask = CLK0|CLK1.
 *
 * Call once at startup, after the I2C bus is up, BEFORE the first
 * tune. Returns 1 if every write was ACKed, 0 otherwise.
 */
uint8_t ms5351_init(void);

/*
 * Byte-exact replay of the captured tune block: LO = 90.800MHz in
 * quadrature on CLK0/CLK1. Useful as ground truth during bring-up -
 * these exact bytes are hardware-proven. Returns 1 if all ACKed.
 */
uint8_t ms5351_tune_captured(void);

/*
 * General quadrature tune: puts freq_hz on CLK0 and CLK1, CLK0 leading
 * by 90 degrees. TWO different techniques under the hood, picked
 * automatically by frequency - see ms5351.c for both:
 *
 *   >= 4.8MHz: the ORIGINAL phase-offset-register scheme (unchanged
 *   from before 31/07/2026) - PLLB fractional feedback + an EVEN
 *   integer output divider N (4-126) whose value is also written to
 *   the CLKx_PHOFF registers to get exactly 90 degrees. Divider
 *   selection: largest even N with N*freq inside the VCO window,
 *   capped at 126 (the 7-bit phase-offset register's limit).
 *
 *   < 4.8MHz: a genuinely FRACTIONAL output divider technique ported
 *   from a well-known, community-vetted SI5351 driver (JF3HZB /
 *   T.UEBO) - see ms5351_set_lo_freq_lowband()'s big comment in
 *   ms5351.c for the full derivation (a "photo finish" trick: run
 *   CLK1 a few Hz slow for a precisely-timed window, then snap it to
 *   the target frequency once it's accumulated exactly 90 degrees of
 *   lag behind CLK0). Extends the usable range down to 100kHz
 *   (LOWF_FLOOR_HZ in ms5351.c) - NOT bench-confirmed on this board
 *   yet at the low end, see that comment's PLL FEEDBACK MULTIPLIER
 *   WARNING before relying on frequencies much below ~1MHz. Crossing
 *   between the two techniques' zones (1.5MHz, 500kHz) costs a one-off
 *   ~62.5ms blocking delay - tuning within a zone is fully responsive.
 *
 * PLLB is soft-reset only when the divider (and therefore the phase
 * offset) changes with respect to the previous call - resetting the
 * PLL on every retune causes an audible click and is not needed when
 * only the fractional feedback moves. First call always resets.
 *
 * Returns 1 on success (all writes ACKed), 0 on I2C failure or if
 * freq_hz is outside the supported range (below LOWF_FLOOR_HZ or
 * above the high-band ceiling).
 */
uint8_t ms5351_set_lo_freq(uint32_t freq_hz);

/*
 * CLK2 8MHz auxiliary output (PLLA/104), captured as configured but
 * powered down. on=1 powers it up and enables its output, on=0
 * returns it to the captured (off) state. Returns 1 if ACKed.
 */
uint8_t ms5351_clk2_8mhz(uint8_t on);

#endif /* MS5351_H */
