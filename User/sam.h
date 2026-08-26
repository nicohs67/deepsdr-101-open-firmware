#ifndef SAM_H
#define SAM_H

#include "arm_math.h"

/*
 * SAM - synchronous AM demodulation (both sidebands), 21/08/2026.
 *
 * Ported from Jorge's own sam.c/sam.h, itself derived from Warren
 * Pratt's WDSP SAM demodulator (2016, GPL, used in PowerSDR/Thetis
 * and many other open SDR projects). Two real, deliberate departures
 * from that source, both documented here rather than silently:
 *
 * 1. NO Hilbert-transform carrier-selective path (SAML/SAMU - single-
 *    sideband synchronous demod that rejects the OTHER sideband using
 *    a 7-stage allpass Hilbert transformer, sam_variables_t's c0[]/
 *    c1[]/a[]/b[]/c[]/d[] arrays in the source Jorge provided). That
 *    source declares those arrays but never initializes them anywhere
 *    - the actual WDSP-standard 7-stage allpass pole coefficients
 *    aren't present in what was handed over, and this project would
 *    rather ship a plain, VERIFIED-correct both-sideband SAM now than
 *    guess at coefficients for a carrier-selective mode and risk
 *    shipping something subtly wrong. DEMOD_SAM (both sidebands) never
 *    touches those arrays in the original source either - it uses
 *    corr[0] directly - so this omission costs nothing for the mode
 *    actually implemented here. Add SAML/SAMU later once real,
 *    verified Hilbert coefficients are sourced.
 *
 * 2. Fixed a real typo in the original source's own (commented-out)
 *    frequency-offset calculation - a stray unmatched '(' before
 *    `omega2` left that line unable to compile as written:
 *      SAM_carrier = 0.08 * (omega2 * SAMPLERATE / (DF * TPI);
 *    sam_step()'s carrier_freq_offset_hz output has the corrected,
 *    complete version - see that function's own comment.
 *
 * Everything else - the PLL (phase detector/loop filter/NCO) and the
 * fade leveler (DC-tracking amplitude smoother for fading AM
 * carriers) - is ported as-is, same structure and constants as the
 * original.
 *
 * Runs on s_i_buf/s_q_buf (96kHz, post channel-filter, same tap AM's
 * own arm_cmplx_mag_f32() envelope detector already uses) - see
 * demod_am.c's DEMOD_MODE_SAM branch.
 */

typedef struct {
    /* PLL */
    float32_t pll_fmax;
    float32_t zeta;
    float32_t omegaN;
    float32_t omega_min;
    float32_t omega_max;
    float32_t g1;
    float32_t g2;
    float32_t phzerror;
    float32_t det;
    float32_t fil_out;
    float32_t del_out;
    float32_t omega2;

    /* fade leveler */
    float32_t tauR;
    float32_t tauI;
    float32_t dc;
    float32_t dc_insert;
    float32_t mtauR;
    float32_t onem_mtauR;
    float32_t mtauI;
    float32_t onem_mtauI;
    uint8_t   fade_leveler; /* on/off - matches sam_variables_t's own flag, default on */

    /* outputs, updated every sam_init()/sam_step() call */
    float32_t corr0; /* = det's own numerator - real part of the carrier-relative signal, this mode's actual audio */
    float32_t corr1; /* imaginary part - phase-detector input, not audio */
    float32_t audio;
    float32_t carrier_hz_raw;    /* omega2 converted to Hz, unsmoothed */
    float32_t carrier_hz;        /* smoothed (0.08/0.92 IIR) - the number to read for PPM calibration, matches original's SAM_carrier/SAM_lowpass pair collapsed into one running value */
} sam_t;

/* sample_rate_hz: the rate sam_step() is actually called at (this
 * project's s_i_buf/s_q_buf, 96000.0f).
 * pll_fmax_hz/omega_n_hz/zeta: same meaning and same WDSP defaults as
 * the original (4000.0f / 200.0f / 65.0f/75.0f). */
void sam_init(sam_t *s, float32_t sample_rate_hz, float32_t pll_fmax_hz,
        float32_t omega_n_hz, float32_t zeta);

/* Resets the PLL's running state (phzerror/det/fil_out/del_out/omega2
 * and the carrier_hz readings) without recomputing g1/g2/omega_min/
 * omega_max - call when retuning to a different station so the loop
 * re-acquires cleanly instead of carrying over the previous station's
 * (irrelevant) frequency estimate. Does not touch the fade leveler's
 * own dc/dc_insert state - a brief fade-leveler transient after
 * retuning is harmless and self-corrects within tauR/tauI, unlike a
 * stale PLL frequency estimate which would actively fight the new
 * carrier. */
void sam_reset(sam_t *s);

/* Feeds one (i,q) sample through the demodulator. Returns the audio
 * sample (same value as s->audio afterward). sample_rate_hz must be
 * the SAME value passed to sam_init() - passed again here rather than
 * stored, matching this project's own established RAM-saving
 * convention elsewhere (see hfdl_project's sam_freq_offset.c, same
 * author, same session, same reasoning). */
float32_t sam_step(sam_t *s, float32_t i_in, float32_t q_in, float32_t sample_rate_hz);

#endif /* SAM_H */
