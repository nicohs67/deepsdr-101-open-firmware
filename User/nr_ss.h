#ifndef NR_SS_H
#define NR_SS_H

#include <stdint.h>

/*
 * Noise reduction by Spectral Subtraction - ported 03/08/2026 from
 * JF3HZB / T. Uebo's ESP32 nr_ss.c/h (uploaded by the project owner)
 * for AM/USB/LSB use, mainly SSB - explicitly NOT WFM/NFM, per the
 * project owner ("no tiene sentido en WFM ni NFM"). See nr_ss.c's own
 * header comment for the full list of what changed in the port.
 *
 * This module knows NOTHING about demod modes, decimation, or the
 * radio pipeline - it just processes whatever NR_SS_BLOCK_SAMPLES-
 * sample float audio block it's handed, in place. demod_am.c owns
 * deciding WHEN to call it (AM/USB/LSB only) and HOW the audio gets
 * to/from the 12kHz rate this module operates at (decimate in,
 * interpolate out - see demod_am.c's own NR integration comment,
 * right where DECIM_COEFFS/INTERP_COEFFS are reused for it).
 */

/*
 * Block size this module operates on - MUST match demod_am.c's
 * DEC_BLOCK_SAMPLES (32 samples @ 12kHz - the SAME decimated rate its
 * existing SSB phasing-method chain already runs at, see its PIPELINE
 * comment above DECIM_COEFFS). Not #include-d from demod_am.h on
 * purpose, to keep this module fully decoupled from the radio
 * pipeline's internals - demod_am.c _Static_assert()s the two values
 * match at compile time instead (see its NR integration comment), so
 * a future change to one side can't silently desync from the other.
 */
#define NR_SS_BLOCK_SAMPLES 32U

/*
 * Precomputes the Hann window + FFT twiddle/bit-reversal tables and
 * resets all state (ring buffer, overlap-add frames, strength). Call
 * once at startup, same as fft_init(). Strength defaults to 0 (off -
 * see nr_ss_set_strength()'s comment).
 */
void nr_ss_init(void);

/*
 * Processes exactly NR_SS_BLOCK_SAMPLES of audio IN PLACE (reads
 * audio[0..n-1], overwrites the SAME array with the noise-reduced
 * result). `n` must equal NR_SS_BLOCK_SAMPLES every call - the
 * internal 4-frame overlap-add ring buffer assumes a constant block
 * size, same contract the original ESP32 code's BUFFER_SIZE had.
 */
void nr_ss_process(float *audio, uint32_t n);

/*
 * Sets the noise-reduction strength/threshold, 0-4095 - same range
 * and mapping as the original ESP32 code's "V_Rsv" potentiometer
 * reading (see nr_ss.c's nr_ss_set_strength() for the exact formula).
 * 0 is an effectively-transparent bypass (same comment) - wire this
 * up to the settings menu's NR tile (RADIO page - see main.c's
 * ENCODER_TARGET_NR).
 */
void nr_ss_set_strength(uint16_t v);

/*
 * Master on/off switch, independent of the strength value above - a
 * distinct control, per the project owner, rather than overloading
 * strength=0 to mean "off" (added 03/08/2026, replacing the earlier
 * strength-only design). Defaults to OFF (0) at nr_ss_init(). demod_am.c
 * checks nr_ss_get_enabled() and skips the whole decimate/process/
 * interpolate stage entirely while off - not just a transparent
 * bypass, genuinely zero extra ISR cycles spent. Wire nr_ss_set_enabled()
 * up to the bottom bar's NR button (main.c's s_btn_nr).
 */
void nr_ss_set_enabled(uint8_t on);
uint8_t nr_ss_get_enabled(void);

#endif /* NR_SS_H */
