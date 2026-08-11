#include "demod_am.h"
#include "config.h"
#include "sdr_rx.h"
#include "gd32_i2s.h"
#include "gd32f4xx.h"
#include "debug_uart.h"
#include "arm_math.h"
#include "nr_ss.h" /* Spectral Subtraction NR, AM/USB/LSB only - see
                     * this file's NR INTEGRATION comment above the
                     * decimate/interpolate instances it reuses. */
#include "rtty.h" /* RTTY decoder, USB/LSB only - see this file's
                    * RTTY INTEGRATION comment. */
#include "rtty_scope.h" /* dedicated audio-domain tuning scope for RTTY,
                           * see this file's RTTY INTEGRATION comment
                           * and rtty_scope.h's own "why a separate FFT"
                           * story. */
#include <math.h> /* atan2f() for the WFM discriminator - see demod_am.h's
                    * WFM note on why this uses libm instead of
                    * arm_atan2_f32() (not present in this project's
                    * pruned CMSIS-DSP tree). */

#define SPK_EN_PORT GPIOB
#define SPK_EN_PIN  GPIO_PIN_7

/*
 * CHANNEL FILTER + AUDIO LPF, both 4th-order Butterworth low-pass
 * (2 cascaded CMSIS biquad DF1 stages each), designed offline via
 * bilinear transform with prewarping at fs=96kHz (was 48kHz, and
 * 192kHz before that - see sdr_rx.h's SDR_RX_BLOCK_SAMPLES comment for
 * why; regenerated with the exact same "each biquad stage individually
 * normalized to unity DC gain" method the earlier values already used
 * - verified by first reproducing the 48kHz coefficients bit-for-bit
 * with that method (in turn already verified against the 192kHz
 * originals) before trusting it for these):
 *
 *   CHF_COEFFS:  -3dB at ~4kHz  (channel filter, applied to I and Q)
 *   ALPF_COEFFS: -3dB at ~6kHz  (audio LPF, applied after the DC
 *                blocker, replaces the old 2x one-pole ~12dB/oct
 *                cascade with a steeper ~24dB/oct rolloff at the
 *                same corner)
 *
 * Coefficient order per CMSIS's DF1 convention: {b0,b1,b2,a1,a2},
 * where y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]
 * (a1/a2 are already the CMSIS sign convention, i.e. pre-negated
 * relative to the textbook "-a1*y[n-1] - a2*y[n-2]" form). Both
 * filters were verified numerically (frequency response at the
 * design corners and well past them) before being embedded here. To
 * retune either corner, regenerate the full 10-value array; don't
 * hand-edit individual coefficients, the 5 values per stage are
 * coupled.
 *
 * ALPF_COEFFS is NFM-ONLY as of 02/08/2026 - AM/USB/LSB moved to their
 * own 3-way selector (ALPF_4K0/2K3/1K8_COEFFS below) instead of sharing
 * this one, so retuning THIS array only ever affects NFM. See
 * AUDIO_BW_4K0's comment in demod_am.h for why they're kept separate
 * rather than just adding this same 6kHz corner as a 4th selectable
 * option.
 */
#define CHF_STAGES 2U
static const float32_t CHF_COEFFS[CHF_STAGES * 5U] = {
    0.0137494f, 0.0274987f, 0.0137494f,  1.5590543f, -0.6140518f,
    0.0155017f, 0.0310034f, 0.0155017f,  1.7577536f, -0.8197604f
};

#define ALPF_STAGES 2U
static const float32_t ALPF_COEFFS[ALPF_STAGES * 5U] = {
    0.0281188f, 0.0562375f, 0.0281188f,  1.3651172f, -0.4775923f,
    0.0331984f, 0.0663969f, 0.0331984f,  1.6117271f, -0.7445208f
};

/*
 * AM/SSB AUDIO FILTER SET - 3 selectable widths, added 02/08/2026 (the
 * 2.3kHz one first, then extended to 3-way the same day per the
 * project owner - see AUDIO_BW_4K0's comment in demod_am.h). Same
 * design method throughout (4th-order Butterworth, 2 cascaded CMSIS
 * biquad DF1 stages, bilinear transform via scipy.signal.butter at
 * fs=96kHz (was 48kHz, and 192kHz before that, see sdr_rx.h's
 * SDR_RX_BLOCK_SAMPLES comment), same b0 used for both stages -
 * the geometric mean of what each stage's OWN unity-DC-gain b0 would
 * be - matching this trio's own established style rather than
 * CHF_COEFFS/ALPF_COEFFS' per-stage-independent split above; cascaded
 * stages give an identical aggregate response either way, this is
 * purely cosmetic/numerical-precision consistency, reproduced exactly
 * against the 48kHz values (in turn already verified against the
 * original 192kHz ones) before trusting it for these).
 * Low-pass only (no high-pass/low-cut) - same shape as ALPF_COEFFS,
 * not a true bandpass SSB filter; a low-cut around 300Hz could be
 * added the same way later if wanted.
 *
 * ALPF_4K0_COEFFS happens to land on the same -3dB corner as
 * CHF_COEFFS above - purely coincidental (CHF_COEFFS runs on the
 * complex RF I/Q pre-demod, this runs on the real audio post-demod;
 * same corner, completely different stage/purpose), not a shared
 * design or a typo. Verified numerically (all at fs=96kHz):
 *   ALPF_4K0: flat to 1.5kHz, -3.02dB at 4.0kHz, -18.86dB at 6.8kHz,
 *             -34.37dB at 10.4kHz
 *   ALPF_2K3: flat to 1kHz, -3.00dB at 2.3kHz, -19.43dB at 4.0kHz,
 *             -33.70dB at 6.0kHz, -52.28dB at 10kHz
 *   ALPF_1K8: flat to 1kHz, -3.01dB at 1.8kHz, -18.57dB at 3.06kHz,
 *             -33.43dB at 4.68kHz, -60.82dB at 10kHz
 * (The slightly LESS steep stopband numbers vs. the 48kHz design at
 * the same relative offsets are expected and harmless - the SAME
 * analog Butterworth response, just less warped by the bilinear
 * transform now that the corner sits closer to DC relative to the new,
 * higher Nyquist. Passband/corner accuracy is still exact either way.)
 */
#define ALPF_4K0_STAGES 2U
static const float32_t ALPF_4K0_COEFFS[ALPF_4K0_STAGES * 5U] = {
    0.0145993f, 0.0291985f, 0.0145993f,  1.5590543f, -0.6140518f,
    0.0145993f, 0.0291985f, 0.0145993f,  1.7577536f, -0.8197604f
};

#define ALPF_2K3_STAGES 2U
static const float32_t ALPF_2K3_COEFFS[ALPF_2K3_STAGES * 5U] = {
    0.0051535f, 0.0103069f, 0.0051535f,  1.7367529f, -0.7566184f,
    0.0051535f, 0.0103069f, 0.0051535f,  1.8700597f, -0.8914501f
};

#define ALPF_1K8_STAGES 2U
static const float32_t ALPF_1K8_COEFFS[ALPF_1K8_STAGES * 5U] = {
    0.0032200f, 0.0064401f, 0.0032200f,  1.7915877f, -0.8040928f,
    0.0032200f, 0.0064401f, 0.0032200f,  1.9006466f, -0.9139129f
};

/*
 * WFM AUDIO LPF, 4th-order Butterworth (2 cascaded CMSIS biquad DF1
 * stages, same shape/convention as CHF_COEFFS/ALPF_COEFFS above),
 * -3dB at ~15kHz @ fs=192kHz - designed offline via scipy.signal.butter
 * (bilinear, matching the project's existing filter-design method) and
 * verified numerically at the corners that matter for broadcast FM:
 *
 *   100Hz-10kHz: essentially flat (0 to -0.15dB) - the broadcast audio
 *                band passes through basically untouched.
 *   15kHz:       -3.01dB (the design corner)
 *   19kHz:       -9.21dB  (knocks down the stereo pilot tone)
 *   23kHz:       -15.95dB (stereo pilot's upper skirt)
 *   38kHz:       -36.52dB (L-R subcarrier, deep into the stopband)
 *
 * *** 05/08/2026: back to 192kHz *** - briefly redesigned for 48kHz on
 * 04/08/2026 when this project moved AM/SSB/NFM to 48kHz, but WFM
 * itself stayed selectable at that (fundamentally too-narrow, see
 * demod_am.h's WFM note) rate for only a day: 05/08/2026 gave WFM its
 * own separate 192kHz-only processing path (demod_wfm_process_raw())
 * instead, so this filter (used ONLY by that function) goes back to
 * its original design, unchanged from before 04/08/2026.
 *
 * This is a SEPARATE instance/state from ALPF_COEFFS (~6kHz, used by
 * AM/SSB) - AM/SSB's narrower voice-bandwidth corner would strip most
 * of what broadcast FM is worth listening to, so WFM gets its own
 * wider filter instead of retuning the shared one. See demod_am.h's
 * WFM note for why 15kHz (not 53kHz+) is the right target: mono-only
 * reception, no stereo subcarrier decode.
 */
#define WFM_ALPF_STAGES 2U
static const float32_t WFM_ALPF_COEFFS[WFM_ALPF_STAGES * 5U] = {
    0.0020570668f, 0.0041141336f, 0.0020570668f,  1.2287186181f, -0.3932293820f,
    1.0000000000f, 2.0000000000f, 1.0000000000f,  1.4942806865f, -0.6943470431f
};

/*
 * WFM DE-EMPHASIS, single-pole LPF: y[n] = ALPHA*y[n-1] + (1-ALPHA)*x[n].
 * ALPHA = exp(-1/(fs*tau)) with fs=192kHz, tau=50us (Europe/CCIR de-
 * emphasis time constant - this receiver is built and used in Spain,
 * see demod_am.h's WFM note; US/NTSC broadcasts use 75us instead,
 * which would be ALPHA=0.9329119604f - if this ever needs to be
 * switchable, add a runtime flag, don't just swap the constant).
 * Corner works out to ~3.18kHz (1/(2*pi*tau)), matching the pre-
 * emphasis broadcast transmitters apply - skipping this stage makes
 * WFM audio sound harsh/treble-heavy, not wrong-frequency, so it's an
 * easy mistake to not notice immediately on the bench.
 *
 * Back to its original 192kHz value 05/08/2026 - see WFM_ALPF_COEFFS'
 * comment just above for why.
 */
#define WFM_DEEMPH_ALPHA 0.9010751057f

/*
 * WFM DISCRIMINATOR GAIN: scales atan2f()'s raw radians-per-sample
 * output before it enters the shared DC-blocker/AGC/output chain.
 * Picked so a full +/-75kHz deviation (phase step ~2.454 rad @ the
 * old 192kHz rate, see demod_am.h) lands close to AGC_TARGET below -
 * not that the exact value matters much, since the AGC peak-
 * normalizes downstream regardless (step 4), this just keeps its gain
 * excursion sane instead of starting from a near-silent or heavily-
 * clipped signal.
 *
 * *** 04/08/2026: at the new 48kHz rate, that same +/-75kHz deviation
 * works out to a phase step of ~9.82 rad - past +/-pi, which means the
 * discriminator's atan2f() wraps/aliases rather than tracking it
 * correctly. This is the SAME underlying problem as the RF-bandwidth
 * note in demod_am.h's WFM section (a full broadcast FM channel no
 * longer fits at this Fs) showing up a second way, at the
 * demodulator itself rather than just the front-end capture window -
 * not something this gain constant can fix. Left unchanged (still
 * the right ballpark for whatever WFM audio does come through
 * degraded) since WFM is accepted as degraded-but-selectable for now,
 * per the project owner - see demod_am.h's WFM note for the deferred
 * dual-rate plan that actually fixes this. */
#define WFM_DISC_GAIN 7300.0f

/*
 * NFM CHANNEL FILTER, 4th-order Butterworth (2 cascaded biquad DF1
 * stages, same shape/convention as CHF_COEFFS/WFM_ALPF_COEFFS), -3dB
 * at 6.25kHz @ fs=96kHz (was 48kHz, and 192kHz before that - see
 * sdr_rx.h's SDR_RX_BLOCK_SAMPLES comment) - applied to I and Q
 * post-down-mix, same spot CHF_COEFFS occupies for AM/SSB, just wider.
 * Designed offline via scipy.signal.butter (same method as the other
 * filters here, reproduced against the 48kHz values bit-for-bit - in
 * turn already verified against the original 192kHz ones - before
 * trusting it for these) and verified numerically:
 *
 *   500Hz-2.5kHz: essentially flat (0 to -0.0dB)
 *   5kHz:         -0.65dB
 *   6.25kHz:      -3.02dB (the design corner)
 *   8kHz:         -9.41dB
 *   10kHz:        -17.20dB
 *   12.5kHz:      -25.62dB (adjacent 12.5kHz-spaced channel, well
 *                 attenuated)
 *
 * Sized via Carson's rule for a 12.5kHz-spaced narrowband FM channel
 * (+/-2.5kHz max deviation, ~3kHz voice -> ~11kHz occupied bandwidth,
 * so a complex LPF corner around half that, 6.25kHz, passes the whole
 * modulated channel through to the discriminator) - see demod_am.h's
 * NFM note for why this can't just reuse CHF_COEFFS' ~4kHz corner.
 * SEPARATE CMSIS instance/state from both s_chf_i/q_inst (AM/SSB) and
 * anything WFM uses - NFM needs its own filter running, potentially
 * mid-session right after AM/SSB were using CHF_COEFFS' instances.
 */
#define NFM_CHF_STAGES 2U
static const float32_t NFM_CHF_COEFFS[NFM_CHF_STAGES * 5U] = {
    0.0010801f, 0.0021602f, 0.0010801f,  1.3418845f, -0.4625531f,
    1.0000000f, 2.0000000f, 1.0000000f,  1.5925797f, -0.7357921f
};

/*
 * NFM DISCRIMINATOR GAIN: same reasoning as WFM_DISC_GAIN, scaled for
 * NFM's much smaller deviation - worst case +/-5kHz (covers both the
 * 2.5kHz and 5kHz narrowband deviation conventions in use) gives a
 * phase step of ~0.3273 rad @ 96kHz (was ~0.6545 rad @ 48kHz, and
 * ~0.1636 rad @ the original 192kHz rate before that - see sdr_rx.h's
 * SDR_RX_BLOCK_SAMPLES comment; still comfortably under +/-pi at any
 * of these, so unlike WFM's case just above, NFM's discriminator stays
 * valid regardless of Fs - no wraparound). The raw pre-AGC amplitude
 * is proportionally smaller now (roughly half, tracking the halved
 * phase step vs the 48kHz value), but that's exactly the kind of thing
 * the AGC downstream absorbs on its own - GAIN below is left
 * unchanged rather than re-tuned for it, same "the exact value doesn't
 * matter much" reasoning as WFM_DISC_GAIN.
 */
#define NFM_DISC_GAIN 110000.0f

/*
 * SQUELCH constants (AM + NFM) - see agc_profile_t-adjacent squelch
 * block comment in demod_am.h for the full design rationale (RF-level
 * metric, not noise-band).
 *
 * SQUELCH_HYSTERESIS_DB: total dead zone width, split evenly
 * around the threshold (+/-0.5x this) between the OPEN and CLOSE trip
 * points, so a signal sitting right at the threshold doesn't chatter.
 * 3dB is a modest, commonly-used margin - wide enough to kill
 * chatter, narrow enough that the squelch still feels responsive
 * where you set it, not "a few dB later than you'd expect".
 *
 * SQUELCH_LEVEL_ALPHA: exponential smoothing of the per-block RF-
 * level reading, applied ONCE PER BLOCK (not per-sample - this is a
 * block-rate EMA, unlike the AGC's per-sample peak release). At
 * SDR_RX_BLOCK_SAMPLES=128 samples/block @ 48kHz (was 512 @ 192kHz
 * before 04/08/2026 - see sdr_rx.h's own comment; SAME ~2.667ms/block
 * either way, by design, so this constant's value below is UNCHANGED),
 * alpha=0.85 gives a time constant of about
 * -2.667ms/ln(0.85) =~ 16ms - reacts to a station appearing/fading in
 * a couple of tens of milliseconds (fast enough not to visibly lag
 * behind the audio unmuting), while still damping block-to-block
 * jitter enough for the hysteresis above to actually work as
 * intended.
 */
#define SQUELCH_HYSTERESIS_DB 3.0f
#define SQUELCH_LEVEL_ALPHA   0.85f

/*
 * SSB (USB/LSB), DECIMATED phasing-method architecture (30/07/2026,
 * replacing the full-rate Hilbert transformer from the same day after
 * real ISR cycle-count data showed it starving the main loop - see
 * demod_am.h's SSB note for the full history).
 *
 * COEFFICIENT ORDER BUG FOUND WHILE REBUILDING THIS: CMSIS's raw FIR
 * functions (arm_fir_f32/arm_fir_decimate_f32/arm_fir_interpolate_f32
 * - unlike the biquad DF1 functions, which use a different {b,a}
 * difference-equation form) all require coefficients in TIME-REVERSED
 * order: {b[numTaps-1], ..., b[1], b[0]}, not natural chronological
 * order. The original full-rate HILBERT_COEFFS array was in natural
 * order - for an ANTISYMMETRIC filter like a Hilbert transformer,
 * reversing the array is mathematically identical to negating it
 * (reversed[i] = h[N-1-i] = -h[i]), so that bug silently computed the
 * NEGATIVE Hilbert transform the whole time: a straightforward,
 * fixable ordering mistake, not the "may need bench swapping, this
 * hardware's I/Q sense is unusual" situation the SSB_SWAP_SIDEBANDS
 * toggle below originally assumed. All three arrays below are now
 * generated pre-reversed. The decimation/interpolation LPFs are
 * symmetric (linear-phase), so reversing them is a no-op either way -
 * only the Hilbert transformer's order actually mattered.
 *
 * PIPELINE: after the shared 0a-0c steps (down-mix, channel filter,
 * still at the full 96kHz rate - was 48kHz, and 192kHz before that,
 * see sdr_rx.h's SDR_RX_BLOCK_SAMPLES comment), SSB diverges:
 *   1. DECIMATE I and Q by 8 (96kHz -> 12kHz, 256 samples/block ->
 *      32 - was decimate-by-4, 48kHz->12kHz, 128->32, and decimate-
 *      by-16, 192kHz->12kHz, 512->32 before that: SAME 12kHz decimated
 *      rate and SAME 32-sample decimated block every time, since the
 *      whole point of each move was changing the FULL rate while
 *      deliberately keeping THIS decimated domain unchanged - see
 *      nr_ss.c's header comment, which depends on it) via
 *      arm_fir_decimate_f32 with DECIM_COEFFS - a 96-tap anti-alias
 *      lowpass (-3dB ~5-6kHz, -54.7dB by 8kHz, -69dB by 10kHz;
 *      verified numerically, combined with the existing channel
 *      filter's own rolloff this comfortably covers everything that
 *      would otherwise fold into the 0-6kHz decimated Nyquist band -
 *      up from 41 taps @ decimate-4, since a decimate-8 anti-alias
 *      filter needs roughly double the taps for the same absolute-Hz
 *      transition steepness, now that it's a narrower slice of the
 *      doubled Fs). Despite the extra taps this is still CHEAP:
 *      arm_fir_decimate_f32 only evaluates the FIR sum at the
 *      DECIMATED output rate (32 samples), so cost is numTaps*32 =
 *      3072 MACs per channel, not numTaps*256 - the whole point of
 *      doing it this way.
 *   2. Hilbert-shift the decimated Q and delay-match the decimated I
 *      (same technique as always, still at 12kHz, UNCHANGED by any Fs
 *      move - see step 1 above) - HILBERT_COEFFS is still 63 taps,
 *      still at 12kHz, covering 300Hz-6kHz the same way it always has
 *      (|H|=0.913 @ 300Hz, essentially flat 1.000 from 500Hz up -
 *      verified numerically). Combine per demod_am.h's mechanics.
 *   3. INTERPOLATE the resulting 12kHz audio back up to 96kHz (32
 *      samples -> 256) via arm_fir_interpolate_f32 with
 *      INTERP_COEFFS, straight into s_env[] - everything downstream
 *      (DC blocker, audio LPF, AGC, output) is then IDENTICAL to AM,
 *      unchanged. INTERP_COEFFS is the SAME ~5-6kHz lowpass design as
 *      DECIM_COEFFS but scaled to DC gain=8 instead of 1 (was gain=4
 *      @ 48kHz, gain=16 @ the original 192kHz - the gain a
 *      reconstruction filter must supply is always exactly the
 *      decimation/interpolation factor L, since zero-stuffing L-1
 *      zeros between samples divides energy by L) - zero-stuffing
 *      divides energy by L, so the reconstruction filter must supply
 *      that L back or the audio comes out at 1/L volume (confirmed
 *      with an explicit round-trip simulation before writing any
 *      firmware - see the design notes referenced above; would have
 *      been yet another "why is the volume so low" report otherwise).
 *      numTaps (96) must be a multiple of the interpolation factor (8)
 *      - arm_fir_interpolate_init_f32() enforces this and
 *      demod_am_init() checks its return status.
 */
#define DECIM_FACTOR 8U /* was 4U @ 48kHz (16U before that, @ 192kHz) - see
                          * this comment block above. AM/SSB/NFM moved from
                          * 48kHz to 96kHz for better RF coverage while
                          * keeping the SAME 12kHz decimated domain the
                          * Hilbert/NR/interpolate chain already runs at -
                          * decimate-by-8 (96kHz -> 12kHz) replaces
                          * decimate-by-4 (48kHz -> 12kHz), same target
                          * output rate either way. */
#define DEC_BLOCK_SAMPLES (SDR_RX_BLOCK_SAMPLES / DECIM_FACTOR) /* 32, @ 12kHz - UNCHANGED
                                                                   * numeric value across every
                                                                   * Fs move this project has made
                                                                   * (192k/16, 48k/4, now 96k/8 -
                                                                   * all land on 32 samples @ 12kHz
                                                                   * by design, see the comment
                                                                   * above) - this is also why
                                                                   * nr_ss.h's NR_SS_BLOCK_SAMPLES
                                                                   * needed zero changes, again. */

/*
 * DECIM_COEFFS/INTERP_COEFFS, regenerated 05/08/2026 for decimate/
 * interpolate-by-8 (96kHz<->12kHz), replacing the by-4 (48kHz<->12kHz)
 * pair. Same passband/stopband TARGET as before (pass to ~5kHz, alias
 * protection from ~8kHz) - Parks-McClellan/Remez equiripple design via
 * scipy.signal.remez, NOT the same design method as the previous by-4
 * filter (that one's exact algorithm/window wasn't reverse-engineered
 * bit-for-bit before this change - only its {taps, corners, reported
 * attenuation} spec was known and matched here as a design TARGET, not
 * reproduced verbatim). Needs far more taps than the by-4 version for
 * the same Hz-domain transition width (96 vs 41) - the normalized
 * transition band (pass-to-stop, relative to Fs) is roughly HALF as
 * wide now that Fs doubled for the same absolute Hz spread, and FIR
 * length scales roughly inversely with normalized transition width for
 * a given stopband depth. Verified numerically (scipy freqz): pass
 * ~flat to 5kHz (-0.01dB), -54.68dB @ 8kHz, -69.06dB @ 10kHz - somewhat
 * less aggressive than the by-4 filter's reported -45dB/-88dB, but
 * still comfortably beyond anything this application's noise floor
 * needs; the extra taps (96 vs 41, ~2.3x) cost is negligible against
 * the ISR's real-time budget (see sdr_tick's own cycle-count prints).
 */
#define DECIM_COEFFS_TAPS 96U
static const float32_t DECIM_COEFFS[DECIM_COEFFS_TAPS] = {
    0.0010989f, 0.0003992f, 0.0002785f, 0.0000067f, -0.0003927f, -0.0008565f,
    -0.0012870f, -0.0015684f, -0.0015915f, -0.0012798f, -0.0006144f, 0.0003447f,
    0.0014523f, 0.0025049f, 0.0032565f, 0.0034804f, 0.0030143f, 0.0018132f,
    -0.0000196f, -0.0022256f, -0.0044165f, -0.0061322f, -0.0069256f, -0.0064572f,
    -0.0045839f, -0.0014265f, 0.0026122f, 0.0068766f, 0.0105477f, 0.0127755f,
    0.0128414f, 0.0103201f, 0.0052114f, -0.0019850f, -0.0102740f, -0.0182575f,
    -0.0243083f, -0.0268031f, -0.0243839f, -0.0162023f, -0.0021094f, 0.0172491f,
    0.0404497f, 0.0654278f, 0.0897184f, 0.1107682f, 0.1262732f, 0.1344935f,
    0.1344935f, 0.1262732f, 0.1107682f, 0.0897184f, 0.0654278f, 0.0404497f,
    0.0172491f, -0.0021094f, -0.0162023f, -0.0243839f, -0.0268031f, -0.0243083f,
    -0.0182575f, -0.0102740f, -0.0019850f, 0.0052114f, 0.0103201f, 0.0128414f,
    0.0127755f, 0.0105477f, 0.0068766f, 0.0026122f, -0.0014265f, -0.0045839f,
    -0.0064572f, -0.0069256f, -0.0061322f, -0.0044165f, -0.0022256f, -0.0000196f,
    0.0018132f, 0.0030143f, 0.0034804f, 0.0032565f, 0.0025049f, 0.0014523f,
    0.0003447f, -0.0006144f, -0.0012798f, -0.0015915f, -0.0015684f, -0.0012870f,
    -0.0008565f, -0.0003927f, 0.0000067f, 0.0002785f, 0.0003992f, 0.0010989f
};

#define HILBERT_COEFFS_TAPS 63U
#define HILBERT_GROUP_DELAY_DEC ((HILBERT_COEFFS_TAPS - 1U) / 2U) /* 31 samples @ 12kHz */
static const float32_t HILBERT_COEFFS[HILBERT_COEFFS_TAPS] = {
    -0.0000000000f, 0.0000000000f, 0.0000823578f, 0.0000000000f, 0.0003687313f, 0.0000000000f,
    0.0009524519f, 0.0000000000f, 0.0019780775f, 0.0000000000f, 0.0036465115f, 0.0000000000f,
    0.0062203355f, 0.0000000000f, 0.0100344372f, 0.0000000000f, 0.0155222771f, 0.0000000000f,
    0.0232796443f, 0.0000000000f, 0.0342170741f, 0.0000000000f, 0.0499393402f, 0.0000000000f,
    0.0738018099f, 0.0000000000f, 0.1145266784f, 0.0000000000f, 0.2042978004f, 0.0000000000f,
    0.6339440957f, 0.0000000000f, -0.6339440957f, 0.0000000000f, -0.2042978004f, 0.0000000000f,
    -0.1145266784f, 0.0000000000f, -0.0738018099f, 0.0000000000f, -0.0499393402f, 0.0000000000f,
    -0.0342170741f, 0.0000000000f, -0.0232796443f, 0.0000000000f, -0.0155222771f, 0.0000000000f,
    -0.0100344372f, 0.0000000000f, -0.0062203355f, 0.0000000000f, -0.0036465115f, 0.0000000000f,
    -0.0019780775f, 0.0000000000f, -0.0009524519f, 0.0000000000f, -0.0003687313f, 0.0000000000f,
    -0.0000823578f, 0.0000000000f, 0.0000000000f
};

#define INTERP_COEFFS_TAPS 96U /* must be a multiple of DECIM_FACTOR (8) - was 40 @
                                 * decimate-4/48kHz, 160 @ decimate-16/192kHz. SAME
                                 * shape as DECIM_COEFFS above (both 96 taps now, for
                                 * the first time actually identical tap counts - the
                                 * by-4 pair had 41 (decimator, no multiple-of-L
                                 * constraint) vs 40 (interpolator, arm_fir_interpolate_
                                 * init_f32() requires a multiple of L) taps, close but
                                 * not literally the same array), scaled to DC gain = 8
                                 * (the interpolation factor L) instead of 1 - see the
                                 * comment above DECIM_COEFFS_TAPS' old value for why
                                 * the reconstruction filter must supply exactly L of
                                 * gain back after zero-stuffing L-1 zeros between
                                 * samples. */
static const float32_t INTERP_COEFFS[INTERP_COEFFS_TAPS] = {
    0.0087911f, 0.0031937f, 0.0022281f, 0.0000539f, -0.0031416f, -0.0068521f,
    -0.0102960f, -0.0125474f, -0.0127322f, -0.0102388f, -0.0049153f, 0.0027572f,
    0.0116186f, 0.0200389f, 0.0260523f, 0.0278433f, 0.0241143f, 0.0145053f,
    -0.0001567f, -0.0178046f, -0.0353319f, -0.0490577f, -0.0554052f, -0.0516576f,
    -0.0366711f, -0.0114122f, 0.0208977f, 0.0550129f, 0.0843813f, 0.1022039f,
    0.1027312f, 0.0825604f, 0.0416915f, -0.0158798f, -0.0821919f, -0.1460602f,
    -0.1944667f, -0.2144249f, -0.1950712f, -0.1296186f, -0.0168749f, 0.1379930f,
    0.3235973f, 0.5234220f, 0.7177471f, 0.8861453f, 1.0101856f, 1.0759478f,
    1.0759478f, 1.0101856f, 0.8861453f, 0.7177471f, 0.5234220f, 0.3235973f,
    0.1379930f, -0.0168749f, -0.1296186f, -0.1950712f, -0.2144249f, -0.1944667f,
    -0.1460602f, -0.0821919f, -0.0158798f, 0.0416915f, 0.0825604f, 0.1027312f,
    0.1022039f, 0.0843813f, 0.0550129f, 0.0208977f, -0.0114122f, -0.0366711f,
    -0.0516576f, -0.0554052f, -0.0490577f, -0.0353319f, -0.0178046f, -0.0001567f,
    0.0145053f, 0.0241143f, 0.0278433f, 0.0260523f, 0.0200389f, 0.0116186f,
    0.0027572f, -0.0049153f, -0.0102388f, -0.0127322f, -0.0125474f, -0.0102960f,
    -0.0068521f, -0.0031416f, 0.0000539f, 0.0022281f, 0.0031937f, 0.0087911f
};

/*
 * SSB sideband sign convention: audio = I_delayed + sign*Hilbert(Q).
 * NOTE (30/07/2026): the original full-rate implementation had the
 * coefficient-order bug described above HILBERT_COEFFS, which
 * silently negated the Hilbert output - if the sidebands ever seemed
 * "swapped" on that build, that bug (now fixed) was why. With the
 * order corrected, the default below is the fresh best guess; if USB
 * and LSB still come out swapped on the bench, flip this to 1 -
 * don't touch anything else.
 */
#define SSB_SWAP_SIDEBANDS 0
#if SSB_SWAP_SIDEBANDS
#define SSB_USB_SIGN (+1.0f)
#define SSB_LSB_SIGN (-1.0f)
#else
#define SSB_USB_SIGN (-1.0f)
#define SSB_LSB_SIGN (+1.0f)
#endif

/* DC blocker pole: y[n] = x[n] - x[n-1] + R*y[n-1]. R=0.9990 at
 * 96kHz (was 0.9980 @ 48kHz, and 0.9995 @ 192kHz before that - see
 * sdr_rx.h's SDR_RX_BLOCK_SAMPLES comment), same tau (~10.4ms) puts
 * the corner around the same ~15Hz regardless of Fs, since R=exp(-1/
 * (fs*tau)) - verified: fs=96000 gives 0.99900 to 5 decimal places.
 * Left as a scalar one-pole - cheap enough (1 MAC/sample) that a
 * CMSIS block call isn't worth the extra buffer/call overhead. */
#define DCB_R 0.9990f

/* AGC: target output amplitude (of int16 full scale 32767), peak
 * release per sample, and gain bounds. GAIN_MAX bounds how far pure
 * band noise gets amplified. Attack is instant (unconditional in the
 * loop below) in every profile - only release varies. */
#define AGC_TARGET   18000.0f
#define AGC_PEAK_MIN 40.0f
/*
 * WFM's OWN AGC peak floor - added 05/08/2026 after real hardware
 * logs pinned down the "first WFM entry after boot is silent/
 * distorted, second one works" report to exactly this constant.
 * AGC_PEAK_MIN (40.0f) above is tuned for AM/SSB/NFM's signal scale -
 * WFM's raw discriminator naturally runs up to roughly
 * +/-pi*WFM_DISC_GAIN (~+/-22900, confirmed in the logs), and even
 * after the DC blocker/de-emphasis/audio LPF chain, its typical
 * envelope settles in the low thousands (~3000-9000, also confirmed
 * in the logs) - starting s_wfm_agc_peak's AGC loop from a floor of
 * 40 when the real signal is 100-200x larger meant the very first
 * block(s) computed gain=AGC_TARGET/40 (clamped to AGC_GAIN_MAX=300),
 * driving the output to hard-clip at exactly +/-AGC_TARGET (18000)
 * sample after sample until the peak-tracking caught up several
 * blocks later - audible as a harsh clipped buzz, not silence, but
 * apparently indistinguishable from "not working" on the bench. A
 * SECOND WFM entry sounded fine because s_wfm_agc_peak carries over
 * between WFM sessions (never reset except at boot - see its own
 * declaration comment) and had already adapted to something close to
 * the real signal level by then. Set here to 4000.0f - comfortably
 * inside the observed steady-state range, so the very first block
 * starts already in the right ballpark instead of climbing there the
 * hard way.
 */
#define WFM_AGC_PEAK_MIN 4000.0f
#define AGC_GAIN_MAX 300.0f

/*
 * Per-profile release coefficients (peak *= this, per sample @ 96kHz,
 * was 48kHz, and 192kHz before that - see sdr_rx.h's
 * SDR_RX_BLOCK_SAMPLES comment) - see agc_profile_t's comment in
 * demod_am.h for what each one is for. Computed offline as
 * exp(-1/(fs*tau)) for the target time constant tau (UNCHANGED tau
 * values across every Fs move - only fs itself moves), same method as
 * WFM_DEEMPH_ALPHA above:
 *
 *   AGC_RELEASE_SLOW   : tau=700ms -> 0.99998512f
 *   AGC_RELEASE_MEDIUM : tau=175ms -> 0.99994048f (this was, to 5
 *                         decimal places at the OLD 192kHz rate, the
 *                         ORIGINAL single AGC_RELEASE value this
 *                         project always used before profiles existed
 *                         - MEDIUM is the default specifically so
 *                         existing behavior doesn't change for anyone
 *                         who never touches the new badge)
 *   AGC_RELEASE_FAST   : tau=60ms  -> 0.99982640f
 *
 * AGC_PROFILE_MANUAL doesn't use any of these - see the loop in
 * demod_am_process_raw() and agc_profile_t's MANUAL note.
 */
#define AGC_RELEASE_SLOW   0.99998512f
#define AGC_RELEASE_MEDIUM 0.99994048f
#define AGC_RELEASE_FAST   0.99982640f

/*
 * WFM's OWN DC blocker pole + AGC release coefficients, at WFM's
 * actual 192kHz rate - added 05/08/2026 when WFM got its own separate
 * processing path (demod_wfm_process_raw()) running at 192kHz while
 * AM/SSB/NFM (DCB_R/AGC_RELEASE_* above) stayed at 48kHz. These are
 * simply the ORIGINAL values this project used everywhere before
 * 04/08/2026's 48kHz move - same tau targets (DCB ~15Hz corner;
 * AGC_RELEASE tau=700/175/60ms), just computed at fs=192kHz instead of
 * 48kHz, since exp(-1/(fs*tau)) genuinely depends on fs. WFM has its
 * own separate AGC state (s_wfm_agc_peak) and DC blocker state
 * (s_wfm_dcb_x1/y1) too - see demod_wfm_process_raw().
 */
#define WFM_DCB_R               0.9995f
#define WFM_AGC_RELEASE_SLOW    0.99999256f
#define WFM_AGC_RELEASE_MEDIUM  0.99997024f
#define WFM_AGC_RELEASE_FAST    0.99991320f

/*
 * LOW-IF DOWN-MIX, SR/4 rotation (30/07/2026, provided by the
 * project owner as known-working, efficient code from elsewhere -
 * see demod_am.h's LOW-IF TUNING note for the full background,
 * including the earlier Fs/8-NCO attempt that was reverted, and the
 * unrelated -O0->-O2 I2C bit-bang timing bug that turned out to be
 * the actual cause of that regression).
 *
 * DIRECTION FIX (30/07/2026, same day): the first version of this
 * rotation used e^(+j*2*pi*n/4) (spectrum shifts UP), on the
 * assumption the wanted signal - LO tuned DEMOD_IF_OFFSET_HZ below
 * the selected station - would land at +SR/4. On the bench it turned
 * out to land at -SR/4 instead (this board's I/Q channel mapping is
 * already documented elsewhere in this codebase as non-textbook -
 * see sdr_rx.h - so the sign of "which way is up" isn't something to
 * assume from first principles here). Fixed by using the conjugate
 * rotation, e^(-j*2*pi*n/4), which cycles through 1, -j, -1, +j
 * instead of 1, +j, -1, -j - still no multiplications, just sign
 * flips and an I/Q swap, applied to every group of 4 samples:
 *   n%4==0: identity                    (x1 unchanged)
 *   n%4==1: (I,Q) -> (Q, -I)             (x -j)
 *   n%4==2: (I,Q) -> (-I, -Q)            (x -1)
 *   n%4==3: (I,Q) -> (-Q, I)             (x +j)
 * SDR_RX_BLOCK_SAMPLES (512) is a whole multiple of 4, so - same
 * argument as the earlier Fs/8 version - phase never drifts or breaks
 * across block boundaries: n=0 of every block always lands back on
 * the identity case.
 */

/* CMSIS biquad instances + their state (state must be caller-owned,
 * 4 floats per stage: x[n-1], x[n-2], y[n-1], y[n-2]). chf_i/chf_q
 * share CHF_COEFFS (read-only) but each needs its own state, since
 * I and Q are independent signals through the same filter shape. */
static arm_biquad_casd_df1_inst_f32 s_chf_i_inst;
static arm_biquad_casd_df1_inst_f32 s_chf_q_inst;
static arm_biquad_casd_df1_inst_f32 s_alpf_inst; /* NFM-only as of 02/08/2026 - see ALPF_COEFFS' comment */
static float32_t s_chf_i_state[CHF_STAGES * 4U];
static float32_t s_chf_q_state[CHF_STAGES * 4U];
static float32_t s_alpf_state[ALPF_STAGES * 4U];

/* AM/SSB audio filter instances/state - THREE separate ones (4K0/2K3/
 * 1K8), see ALPF_4K0_COEFFS' comment above for the design set and
 * demod_am_set_audio_bw()'s comment in demod_am.h for the selector
 * itself. Independent instances rather than re-pointing one shared
 * instance at different coefficients on every switch: CMSIS's biquad
 * state buffer holds each stage's x[n-1]/x[n-2]/y[n-1]/y[n-2] history,
 * which is only valid for the coefficients that produced it - swapping
 * coefficients under a live state buffer would feed the NEW filter's
 * difference equation stale history from the OLD one, glitching for a
 * few samples every switch. Three full instances sidesteps that
 * entirely: each only ever sees data through its own coefficients, so
 * there's nothing to resync when demod_am_process_raw() switches which
 * one's OUTPUT it uses. Same reasoning WFM/NFM's own separate
 * instances already rely on, just applied to a runtime selector
 * instead of a mode switch. */
static arm_biquad_casd_df1_inst_f32 s_alpf_4k0_inst;
static arm_biquad_casd_df1_inst_f32 s_alpf_2k3_inst;
static arm_biquad_casd_df1_inst_f32 s_alpf_1k8_inst;
static float32_t s_alpf_4k0_state[ALPF_4K0_STAGES * 4U];
static float32_t s_alpf_2k3_state[ALPF_2K3_STAGES * 4U];
static float32_t s_alpf_1k8_state[ALPF_1K8_STAGES * 4U];

/* WFM audio LPF instance/state - separate from s_alpf_inst above
 * (NFM), see WFM_ALPF_COEFFS' comment for why. */
static arm_biquad_casd_df1_inst_f32 s_wfm_alpf_inst;
static float32_t s_wfm_alpf_state[WFM_ALPF_STAGES * 4U];

/* NFM channel filter instance/state - separate from s_chf_i/q_inst
 * above (AM/SSB) - see NFM_CHF_COEFFS' comment for why NFM can't just
 * reuse CHF_COEFFS' narrower corner. NFM has no audio-LPF instance of
 * its own (unlike WFM) - it reuses s_alpf_inst, see demod_am.h's NFM
 * note. */
static arm_biquad_casd_df1_inst_f32 s_nfm_chf_i_inst;
static arm_biquad_casd_df1_inst_f32 s_nfm_chf_q_inst;
static float32_t s_nfm_chf_i_state[NFM_CHF_STAGES * 4U];
static float32_t s_nfm_chf_q_state[NFM_CHF_STAGES * 4U];

/* FM discriminator state: previous block's last complex sample,
 * carried across blocks so the delay-and-conjugate-multiply (see
 * demod_am.h's WFM note) has a valid z[n-1] for n=0 of every new
 * block, same pattern as the SSB delay-match history below. SHARED
 * BETWEEN WFM AND NFM (added 31/07/2026) - both call the same
 * fm_discriminate() helper and are never active at once, so one set
 * of state is enough; see demod_am.h's NFM note. WFM de-emphasis
 * state (single-pole LPF, see WFM_DEEMPH_ALPHA) is NFM-irrelevant
 * (NFM doesn't de-emphasize, see demod_am.h) and stays WFM-only. */
static float32_t s_fm_i_prev;
static float32_t s_fm_q_prev;
static float s_wfm_deemph_y1;

/*
 * WFM's OWN buffers/state, at its OWN 512-sample/192kHz block size -
 * added 05/08/2026 when WFM got its own separate processing path
 * (demod_wfm_process_raw()), independent of AM/SSB/NFM's 128-sample/
 * 48kHz s_i_buf/s_q_buf/s_env/s_audio_out/s_dcb_x1/y1/s_agc_peak
 * above (see sdr_rx.h's SDR_RX_BLOCK_SAMPLES_MAX comment for why this
 * project went with fully separate buffers per rate rather than
 * making every shared buffer size-aware - the "ruta separada"
 * decision). s_wfm_alpf_inst/state and s_fm_i_prev/q_prev and
 * s_wfm_deemph_y1 just above are REUSED as-is, not duplicated here -
 * CMSIS biquad DF1 state is always 4 floats/stage regardless of block
 * size, and the FM discriminator/de-emphasis states are single
 * scalars either way, so none of those needed a WFM-specific twin.
 */
static float32_t s_wfm_i_buf[SDR_RX_BLOCK_SAMPLES_WFM];
static float32_t s_wfm_q_buf[SDR_RX_BLOCK_SAMPLES_WFM];
static float32_t s_wfm_env[SDR_RX_BLOCK_SAMPLES_WFM];
static int16_t   s_wfm_audio_out[SDR_RX_BLOCK_SAMPLES_WFM * 2U];
static float s_wfm_dcb_x1;
static float s_wfm_dcb_y1;
static float s_wfm_agc_peak = WFM_AGC_PEAK_MIN;
static volatile uint32_t s_wfm_last_cycles; /* mirrors demod_am_get_last_cycles() for WFM's own path */

/* Diagnostic-only: logs min/max at 3 checkpoints (raw discriminator
 * output, post-filter-chain pre-AGC, final int16 output) for the
 * first several demod_wfm_process_raw() calls after each WFM entry -
 * added 05/08/2026 to chase the "first WFM entry after boot is
 * silent, second one works" report, once the write_half()/DMA-
 * position theory (gd32_i2s.c) and the PLL-settling-time theory
 * (aic3204_set_rate_power_up()) were BOTH ruled out by real hardware
 * A/B logs - this checks whether the actual DEMODULATED AUDIO differs
 * between a failing and a working entry, to tell an RF/discriminator/
 * AGC problem apart from a TX/DMA problem the other diagnostics
 * didn't catch. Reset by demod_wfm_reset_diag() - see its own comment
 * for why main.c calls it right when switching INTO WFM. */
static uint32_t s_wfm_diag_count;

/*
 * *** 05/08/2026, DISABLED (always returns 0) *** - this per-block
 * logging (raw discriminator/post-filter/AGC-peak/output values, plus
 * the raw I/Q dump gated on s_wfm_diag_count==1) did its job: it's
 * what identified the AGC convergence behavior and, via the raw I/Q
 * dump, the I2S frame-sync bug now fixed by main.c's rx_lock retry
 * logic. But real hardware logs then showed the logging itself had
 * become the problem: with the rx_lock fix guaranteeing a clean raw
 * capture, the project owner still heard WFM as "muy distorsionado/
 * entrecortado" - and gd32_i2s.c's write_half() diagnostic (now also
 * removed, see its own history) showed "SAME HALF WRITTEN TWICE"
 * firing in tight clusters immediately after every one of THIS
 * function's logged blocks. debug_print()/debug_print_dec() go out
 * over a slow, blocking UART; this function's checkpoints run several
 * of them per logged block, synchronously inside the audio ISR chain,
 * every 2.67ms - stacking enough of them back-to-back can itself eat
 * into that budget and desync the very ping-pong buffer being played,
 * which is audible as exactly the choppiness being chased. In short,
 * the instrumentation was measuring its own footprint. s_wfm_diag_count
 * itself still increments in the caller (kept as a free-running block
 * counter in case something needs it later) - only the actual UART
 * output is disabled here. Change the `return 0U;` below back to the
 * original range check (see git history/transcript) if this class of
 * bug ever needs revisiting, and interpret results with this same
 * caveat in mind.
 */
static uint8_t wfm_should_log_diag(void)
{
    (void)s_wfm_diag_count;
    return 0U;
}

/*
 * AM/SSB/NFM's own diagnostic counter, added 05/08/2026 after the
 * project owner's report that AM/SSB/NFM can ALSO come back broken
 * ("no hay demodulacion, ruido muy fuerte") specifically right after
 * a WFM session, even though the panadapter's own RF spectrum looks
 * correct at the same moment - pointing at the TX/audio-output side
 * rather than the RX/RF side. WFM's own equivalent diagnostic
 * (wfm_should_log_diag() above) already showed WFM's own AGC/filter
 * chain converging correctly, so this adds the SAME visibility to
 * demod_am_process_raw()'s side of a rate-switch transition. Reset by
 * demod_am_reset_diag() - called from main.c's apply_demod_mode()
 * whenever switching OUT of WFM (into AM/USB/LSB/NFM), mirroring
 * demod_wfm_reset_diag()'s own call on the way in. Sparse logging
 * pattern identical to wfm_should_log_diag() - see its comment - but
 * over a shorter span (100 blocks = ~267ms @ 128 samples/48kHz) since
 * AM/SSB/NFM doesn't have anything like WFM's long settle-mute window
 * to span past.
 */
static uint32_t s_am_diag_count;

/*
 * *** 05/08/2026, DISABLED (always returns 0) *** - same reasoning
 * and same fix as wfm_should_log_diag() above: this logging did its
 * job (confirmed AM/SSB/NFM's own AGC converges correctly across a
 * switch) but real-time UART output from inside the audio ISR chain
 * risks causing the exact choppiness it would be used to diagnose.
 * See wfm_should_log_diag()'s comment for the full story.
 */
static uint8_t am_should_log_diag(void)
{
    (void)s_am_diag_count;
    return 0U;
}

/*
 * AM/SSB/NFM's own settle-mute, added 05/08/2026 alongside the
 * s_mode race fix in main.c's apply_demod_mode() - fixing the race
 * (s_mode reading back as WFM/4 for the first 20-30 blocks) turned
 * out not to be the whole story: logs taken AFTER that fix still
 * showed s_agc_peak starting well above its normal steady-state range
 * right after a WFM->AM switch (e.g. 24227, vs a typical 8000-15000
 * once settled) - the SAME "transient poisons the slowly-decaying AGC
 * peak" mechanism WFM_SETTLE_MUTE_BLOCKS already exists to protect
 * against on the way INTO WFM, just never mirrored on the way OUT.
 * Same fix, same reasoning: let s_agc_peak/s_dcb_x1/y1 keep adapting
 * normally to whatever's actually arriving, only force the AUDIBLE
 * OUTPUT to silence for the first AM_SETTLE_MUTE_BLOCKS blocks. 50
 * blocks = ~133ms @ 128 samples/48kHz - shorter than WFM's 200-block
 * window since these logs show AM settling faster (steady-state
 * reached by roughly block 20-30 even without muting), but easy to
 * extend if a future log shows it's still not enough.
 */
#define AM_SETTLE_MUTE_BLOCKS 50U
static uint32_t s_am_mute_remaining;

void demod_am_reset_diag(void)
{
    s_am_diag_count = 0U;
    s_am_mute_remaining = AM_SETTLE_MUTE_BLOCKS;
}

/*
 * "Settle mute" - added 05/08/2026 after real hardware logs showed
 * the RAW RF I/Q captured for the panadapter is genuinely SATURATED
 * (pegged near +/-32768) for a brief window right after a live WFM
 * entry, while a later (second, same-session) entry captures clean
 * I/Q - a transient the codec/DMA fixes on its own within some tens
 * of ms, not something this project has fully eliminated at the
 * hardware/register level despite everything already tried (see
 * aic3204_set_rate_registers()'s and sdr_rx_resync_spi()'s comments
 * for that history).
 *
 * *** FIRST DRAFT WAS WRONG, fixed 05/08/2026 same day *** - it
 * skipped the ENTIRE pipeline (discriminator/DC-blocker/de-emphasis/
 * ALPF/AGC) during the mute window, on the theory that this would
 * protect s_wfm_agc_peak from being poisoned by the transient. Real
 * hardware logs proved that backfired: skipping ALL processing also
 * means s_wfm_agc_peak never gets a chance to ADAPT during the mute
 * window either, so it was still sitting at its cold-start
 * WFM_AGC_PEAK_MIN default the instant the mute ended - just
 * delaying the exact same ramp-up distortion by
 * WFM_SETTLE_MUTE_BLOCKS blocks instead of fixing it. The whole
 * reason a SECOND WFM entry sounds clean is that s_wfm_agc_peak
 * carries over already-adapted from the end of the first session -
 * i.e. adaptation during "unheard" time is exactly what needs to
 * happen, not what needs to be prevented.
 *
 * FIXED VERSION: run the FULL pipeline normally every block, so
 * s_wfm_agc_peak (and the DC blocker/de-emphasis/ALPF state) keep
 * adapting to whatever's actually arriving - only the very last step
 * (what gets sent to gd32_i2s_stream_write_half()) is forced to
 * silence during the mute window. By the time real audio is heard,
 * the AGC has already had WFM_SETTLE_MUTE_BLOCKS blocks to converge,
 * the same way it would have anyway by the second WFM entry.
 *
 * Reset by demod_wfm_reset_diag() alongside the diagnostic counter,
 * since both need to restart together on every WFM entry. 15 blocks
 * = ~40ms @ 512 samples/192kHz - a guess at a safe margin, worth
 * shortening once it's confirmed how long the transient actually
 * lasts (the diagnostic logging already in place would show that,
 * now that it reflects blocks that were ACTUALLY adapting).
 *
 * *** 05/08/2026, bumped 15 -> 200 blocks *** - the FIRST attempt at
 * this number (15 blocks, ~40ms) turned out to be far too short: a
 * later log showed the panadapter's raw I/Q still pegged/saturated
 * (+/-32768) in the sdr_tick print that fires AFTER the mute window
 * plus the first 8 diagnostic blocks (~61ms total) - so whatever this
 * transient actually is, it outlasts 61ms, let alone the original
 * 40ms guess. 200 blocks = ~520ms - a much more generous margin,
 * meant to be safely past the transient regardless of its real
 * duration, at the cost of a longer silence before WFM audio starts
 * (acceptable, given a mode switch already has some silence either
 * way - see apply_demod_mode()'s own comment in main.c). The
 * diagnostic logging below is now spread across the whole window
 * (see its own comment) specifically so the NEXT log can show
 * exactly when the transient actually ends, instead of only covering
 * the first 8 blocks the way it used to - that number can then come
 * back down to whatever's actually needed.
 */
#define WFM_SETTLE_MUTE_BLOCKS 200U
static uint32_t s_wfm_mute_remaining;

void demod_wfm_reset_diag(void)
{
    s_wfm_diag_count = 0U;
    s_wfm_mute_remaining = WFM_SETTLE_MUTE_BLOCKS;
}

/* Local signed-decimal print helper - main.c has its own
 * debug_print_dec_signed(), but it's static there (not exported via
 * debug_uart.h), so this file needs its own copy for the diagnostics
 * below. Simplest form: print a '-' then the magnitude via the
 * shared debug_print_dec(), which only takes uint32_t. */
static void debug_print_dec_signed_local(const char *label, int32_t val)
{
    if (val < 0) {
        debug_print(label);
        debug_print(" = -");
        debug_print_dec("", (uint32_t)(-val));
    } else {
        debug_print_dec(label, (uint32_t)val);
    }
}

/* SSB decimated-chain instances + state (see the PIPELINE comment
 * above DECIM_COEFFS). CMSIS state sizes:
 *   decimate:    numTaps + blockSize(INPUT, 128) - 1 (was 512 before
 *                04/08/2026, see sdr_rx.h's SDR_RX_BLOCK_SAMPLES comment)
 *   plain FIR:   numTaps + blockSize(decimated, 32) - 1
 *   interpolate: (numTaps/L) + blockSize(decimated, 32) - 1
 * I and Q each need their own decimator state (same coefficients,
 * independent signals - same pattern as the channel filter pair). */
static arm_fir_decimate_instance_f32 s_decim_i_inst;
static arm_fir_decimate_instance_f32 s_decim_q_inst;
static arm_fir_instance_f32          s_hilbert_inst;
static arm_fir_interpolate_instance_f32 s_interp_inst;
static float32_t s_decim_i_state[DECIM_COEFFS_TAPS + SDR_RX_BLOCK_SAMPLES - 1U];
static float32_t s_decim_q_state[DECIM_COEFFS_TAPS + SDR_RX_BLOCK_SAMPLES - 1U];
static float32_t s_hilbert_state[HILBERT_COEFFS_TAPS + DEC_BLOCK_SAMPLES - 1U];
static float32_t s_interp_state[(INTERP_COEFFS_TAPS / DECIM_FACTOR) + DEC_BLOCK_SAMPLES - 1U];

/* Decimated-rate (12kHz, 32-sample) working buffers. */
static float32_t s_i_dec[DEC_BLOCK_SAMPLES];         /* decimated I */
static float32_t s_q_dec[DEC_BLOCK_SAMPLES];         /* decimated Q */
static float32_t s_q_hilbert_out[DEC_BLOCK_SAMPLES]; /* Hilbert(Q), 90deg-shifted, @ 12kHz */
static float32_t s_ssb_dec[DEC_BLOCK_SAMPLES];       /* combined SSB audio @ 12kHz */

/*
 * --- NR INTEGRATION (Spectral Subtraction, 03/08/2026, revised 04/08/2026) ---
 *
 * AM/USB/LSB, mainly SSB - per the project owner, explicitly NOT
 * WFM/NFM. This decimator/interpolator pair is used for AM ONLY as of
 * 04/08/2026 - USB/LSB instead run NR inline on s_ssb_dec, the SSB
 * chain's OWN already-12kHz buffer (see step 2d's comment in
 * demod_am_process_raw()), which avoids interpolating SSB's audio up
 * to 192kHz only to immediately decimate it back down again just for
 * NR - a wasted round trip that was pushing SSB+NR up against the
 * ISR's cycle budget (the project owner's own report: "en modo SSB y
 * con NR ya estamos llegando al limite del controlador"). AM has no
 * equivalent already-decimated buffer (its envelope only ever exists
 * at the full s_env[]), so it still needs this dedicated pair. Same
 * coefficients as the SSB chain above either way (passed by pointer,
 * not duplicated - CMSIS's init functions don't copy the coefficient
 * array), just this is a MONO signal (s_env[], not I/Q), hence its own
 * instances rather than sharing the SSB chain's I/Q pair. This is also
 * what gets nr_ss.c's Nstfft down to 128 instead of the 2048 a naive
 * full-192kHz-rate port would have needed - see nr_ss.c's header
 * comment for the full reasoning.
 *
 * Gated on nr_ss_get_enabled() (a genuine on/off switch, the bottom
 * bar's NR button - see nr_ss_set_enabled()'s comment in nr_ss.h) -
 * skipped ENTIRELY while off, not just a transparent bypass: zero
 * extra ISR cycles spent, for both the AM path here and USB/LSB's
 * inline path in step 2d. */
static arm_fir_decimate_instance_f32    s_nr_decim_inst;
static arm_fir_interpolate_instance_f32 s_nr_interp_inst;
static float32_t s_nr_decim_state[DECIM_COEFFS_TAPS + SDR_RX_BLOCK_SAMPLES - 1U];
static float32_t s_nr_interp_state[(INTERP_COEFFS_TAPS / DECIM_FACTOR) + DEC_BLOCK_SAMPLES - 1U];
static float32_t s_nr_buf[DEC_BLOCK_SAMPLES]; /* decimated audio (AM only), in place through nr_ss_process() */

/* nr_ss.h fixes its own block size independently (see its comment on
 * NR_SS_BLOCK_SAMPLES) - this guarantees it never silently drifts
 * from demod_am.c's actual decimated rate/block size instead of
 * failing subtly (wrong-length overlap-add) at runtime. */
_Static_assert(DEC_BLOCK_SAMPLES == NR_SS_BLOCK_SAMPLES,
               "nr_ss.h's NR_SS_BLOCK_SAMPLES must match demod_am.c's DEC_BLOCK_SAMPLES");

/* Delay-matching for decimated I: same technique as before (linear-
 * phase FIR group delay = (numTaps-1)/2, history carried across
 * blocks), just at the 12kHz rate now - so the history is only 31
 * samples of the 32-sample decimated block. */
static float32_t s_i_delay_hist[HILBERT_GROUP_DELAY_DEC];
static float32_t s_i_delayed[HILBERT_GROUP_DELAY_DEC + DEC_BLOCK_SAMPLES];

/* Demodulation mode - see demod_am.h. Plain uint8_t, not behind a
 * critical section: main.c writes it from the main loop, the ISR
 * reads it once per block: worst case a mode change lands one block
 * late, which is inaudible and not worth stalling the ISR over. */
static uint8_t s_mode = (uint8_t)CONFIG_START_MODE; /* see config.h - was DEMOD_MODE_WFM - changed
                                    * 05/08/2026: harmless before WFM got its own
                                    * separate 192kHz path (demod_wfm_process_raw()),
                                    * since demod_am_process_raw() used to handle WFM
                                    * too - now that it doesn't, defaulting to WFM here
                                    * while main.c's boot sequence registers
                                    * demod_am_process_raw() as the hook (matching the
                                    * codec/DMA, which always boot at 48kHz - see
                                    * aic3204_phase2_init()) would boot into a genuine
                                    * mismatch: UI shows WFM selected, audio runs
                                    * through the 48kHz AM/SSB/NFM path regardless. If
                                    * WFM as the boot default is wanted back, it needs
                                    * to go through apply_demod_mode() in main.c
                                    * AFTER the normal 48kHz boot sequence completes,
                                    * not just this static initializer - see that
                                    * function's comment for why (it drives a real
                                    * codec/DMA reconfiguration, not just a UI label). */

void demod_am_set_mode(demod_mode_t mode)
{
    s_mode = (uint8_t)mode;
}

demod_mode_t demod_am_get_mode(void)
{
    return (demod_mode_t)s_mode;
}

/* AM/SSB audio filter width - see demod_am_set_audio_bw()'s comment in
 * demod_am.h. Same "plain uint8_t-sized enum, no critical section"
 * reasoning as s_mode above. Default AUDIO_BW_4K0 (the widest of the
 * three - see ALPF_4K0_COEFFS' comment). */
static audio_bw_t s_audio_bw = AUDIO_BW_4K0;

void demod_am_set_audio_bw(audio_bw_t bw)
{
    s_audio_bw = bw;
}

audio_bw_t demod_am_get_audio_bw(void)
{
    return s_audio_bw;
}

/*
 * s_agc_profile/s_agc_release: same "plain aligned variable, no
 * critical section" reasoning as s_mode above - main.c writes,
 * the ISR reads once per block. s_agc_release is the ALREADY-RESOLVED
 * coefficient (not re-switched on s_agc_profile every block) so the
 * hot loop in demod_am_process_raw() is just "peak *= s_agc_release",
 * same cost as before profiles existed - the switch only happens here,
 * in demod_am_set_agc_profile(), on the rare event of the user
 * actually changing it.
 */
static agc_profile_t s_agc_profile = AGC_PROFILE_MEDIUM;
static float s_agc_release = AGC_RELEASE_MEDIUM;
/* WFM's OWN resolved release coefficient, at WFM_AGC_RELEASE_*'s
 * 192kHz values - added 05/08/2026 alongside demod_wfm_process_raw().
 * Kept in lockstep with s_agc_release above from the SAME profile
 * selection (one AGC profile setting, two Fs-appropriate numbers
 * resolved from it) rather than a second independent setting - the
 * operator picks "how fast should the AGC react", not "how fast
 * should it react in which mode". */
static float s_wfm_agc_release = WFM_AGC_RELEASE_MEDIUM;

void demod_am_set_agc_profile(agc_profile_t profile)
{
    s_agc_profile = profile;
    switch (profile) {
    case AGC_PROFILE_SLOW:
        s_agc_release = AGC_RELEASE_SLOW;
        s_wfm_agc_release = WFM_AGC_RELEASE_SLOW;
        break;
    case AGC_PROFILE_FAST:
        s_agc_release = AGC_RELEASE_FAST;
        s_wfm_agc_release = WFM_AGC_RELEASE_FAST;
        break;
    case AGC_PROFILE_MANUAL: break; /* unused in MANUAL - see the AGC loop */
    case AGC_PROFILE_MEDIUM:
    default:
        s_agc_release = AGC_RELEASE_MEDIUM;
        s_wfm_agc_release = WFM_AGC_RELEASE_MEDIUM;
        break;
    }
}

agc_profile_t demod_am_get_agc_profile(void)
{
    return s_agc_profile;
}

/*
 * Squelch state (AM + NFM) - same "plain aligned variables, no critical
 * section" reasoning as s_mode/s_agc_profile above. s_squelch_open
 * is READ by demod_am_process_raw() every block (to decide whether to
 * silence s_env[]) and WRITTEN by it too (that's where the hysteresis
 * decision happens) - main.c never touches it directly, only reads it
 * via the accessor for a UI indicator if it wants one.
 */
static float s_squelch_threshold_db = 0.0f; /* effectively OFF - see demod_am.h */
static float s_squelch_level_db = -100.0f;
static uint8_t s_squelch_open = 1U; /* start OPEN - matches "OFF by default" above */

void demod_am_set_squelch_db(float threshold_db)
{
    s_squelch_threshold_db = threshold_db;
}

float demod_am_get_squelch_db(void)
{
    return s_squelch_threshold_db;
}

float demod_am_get_squelch_level_db(void)
{
    return s_squelch_level_db;
}

uint8_t demod_am_get_squelch_open(void)
{
    return s_squelch_open;
}

/*
 * ISR TIMING INSTRUMENTATION (30/07/2026, added after a report of the
 * whole system hanging on switching into USB/LSB - see the tap-count
 * comment above HILBERT_COEFFS). Cheap on purpose: just two
 * DWT->CYCCNT reads and a subtraction per block, no UART access here
 * - debug_print() is a blocking, polled, character-at-a-time UART
 * write (see debug_uart.c), and calling that from this ISR (NVIC
 * priority 6) would itself burn a chunk of the ~2.67ms block budget
 * every time it fired, which would make this instrumentation part of
 * the problem instead of diagnosing it. main.c's main loop reads
 * demod_am_get_last_cycles() periodically instead and prints THERE,
 * where blocking UART is cheap (nothing else is time-critical at that
 * priority level). Requires DWT->CYCCNT already running - main.c
 * enables it once, before the first call into this ISR, in
 * sdr_spectrum_waterfall_tick()'s one-time init. */
static volatile uint32_t s_last_cycles = 0U;

uint32_t demod_am_get_last_cycles(void)
{
    return s_last_cycles;
}

/*
 * WFM's OWN ISR timing getter (05/08/2026) - mirrors
 * demod_am_get_last_cycles() exactly, same reasoning (cheap
 * DWT->CYCCNT capture inside the ISR, read out and printed from
 * main.c's main loop where blocking UART is affordable). s_wfm_last_
 * cycles was already being captured every block in
 * demod_wfm_process_raw() but had NO public getter until now, so
 * main.c's periodic ISR-timing check (which only ever called
 * demod_am_get_last_cycles(), unconditionally, even while WFM was the
 * active mode) had no way to surface it - WFM's real-time budget has
 * never actually been checked. See main.c's main loop for the
 * corresponding fix that finally reads this. */
uint32_t demod_wfm_get_last_cycles(void)
{
    return s_wfm_last_cycles;
}

/* Per-stage breakdown (see demod_am.h's comment on
 * demod_am_get_last_cycles_breakdown()). Plain volatile uint32_t
 * words, same "not worth a critical section" reasoning as s_mode -
 * worst case the main loop reads one stage from the current block and
 * another from the previous one, which for a diagnostic readout is
 * irrelevant. */
static volatile uint32_t s_last_cycles_frontend = 0U;
static volatile uint32_t s_last_cycles_extract  = 0U;
static volatile uint32_t s_last_cycles_audio    = 0U;
static volatile uint32_t s_last_cycles_nr       = 0U;
static volatile uint32_t s_last_cycles_agc_out  = 0U;

demod_am_cycles_breakdown_t demod_am_get_last_cycles_breakdown(void)
{
    demod_am_cycles_breakdown_t b;
    b.frontend = s_last_cycles_frontend;
    b.extract  = s_last_cycles_extract;
    b.audio    = s_last_cycles_audio;
    b.nr       = s_last_cycles_nr;
    b.agc_out  = s_last_cycles_agc_out;
    return b;
}


/* DC blocker + AGC state (scalar stages, unchanged from before). */
static float s_dcb_x1, s_dcb_y1;
static float s_agc_peak;

/* Signal strength for the UI's S-meter: the AGC's pre-gain envelope
 * peak follower (instant attack, ~180ms release - exactly the
 * ballistics a signal meter wants, no extra state needed). int16
 * full-scale units; the UI converts to dB/S-units itself, OUTSIDE
 * the ISR. */
float demod_am_get_signal_peak(void)
{
    return s_agc_peak;
}

/*
 * RF front-end clipping detection - see its getter's comment in
 * demod_am.h for the full "why". s_rf_clip_flag is only ever SET from
 * inside an ISR (rf_clip_scan(), called from both
 * demod_am_process_raw() and demod_wfm_process_raw()) and only ever
 * CLEARED from the main loop (the getter below) - a one-directional
 * flag, not a counter, so there's no lost-update risk worth guarding
 * further.
 *
 * Threshold/count reasoning: RF_CLIP_THRESHOLD (32000) sits close to
 * the ADC's real full scale (+/-32768) without demanding an exact
 * railed value - a genuinely clipped waveform sits flat at or near
 * the rail for MULTIPLE consecutive samples, not just one. Requiring
 * RF_CLIP_MIN_COUNT (3) samples over that threshold in the same block
 * (out of 256-512) filters out a single legitimate near-full-scale
 * peak (a strong-but-not-clipping signal can absolutely touch high
 * values for one sample without being clipped) while still catching
 * real clipping fast - a genuinely overloaded front end will have
 * FAR more than 3 railed samples per block, so this threshold errs
 * toward triggering readily rather than missing real clipping.
 */
#define RF_CLIP_THRESHOLD  CONFIG_RF_CLIP_THRESHOLD /* see config.h */
#define RF_CLIP_MIN_COUNT  CONFIG_RF_CLIP_MIN_COUNT /* see config.h */
static volatile uint8_t s_rf_clip_flag = 0U;

static void rf_clip_scan(const int16_t *raw_interleaved, uint32_t n_samples)
{
    uint32_t i;
    uint32_t hits = 0U;

    for (i = 0; i < 2U * n_samples; i++) {
        int16_t v = raw_interleaved[i];
        if (v >= RF_CLIP_THRESHOLD || v <= -RF_CLIP_THRESHOLD) {
            hits++;
            if (hits >= RF_CLIP_MIN_COUNT) {
                s_rf_clip_flag = 1U;
                break; /* already confirmed for this block, no need to keep scanning */
            }
        }
    }
}

uint8_t demod_am_get_and_clear_rf_clip_flag(void)
{
    uint8_t v = s_rf_clip_flag;
    s_rf_clip_flag = 0U;
    return v;
}

/* Low-IF down-mix on/off, kept in sync with the actual LO by whoever
 * tunes it (see demod_am.h) - same safety pattern as the earlier
 * Fs/8 attempt: default OFF, so demod_am_init() running before the
 * first tune (which is still the unmodified byte-exact captured
 * replay, no offset - see main.c) never mismatches the LO. Only
 * tune_encoder_poll() turns this on, once it has actually programmed
 * the LO with the offset. */
static uint8_t s_if_offset_active = 0U;

void demod_am_set_if_offset_active(uint8_t active)
{
    s_if_offset_active = active ? 1U : 0U;
}

uint8_t demod_am_get_if_offset_active(void)
{
    return s_if_offset_active;
}

/*
 * Per-block scratch buffers. Static - this runs in ISR context, keep
 * it off the stack. See demod_am.h for the RAM budget.
 */
static float32_t s_i_buf[SDR_RX_BLOCK_SAMPLES];      /* I rail, deinterleaved */
static float32_t s_q_buf[SDR_RX_BLOCK_SAMPLES];      /* Q rail, deinterleaved */
static float32_t s_iq_cplx[SDR_RX_BLOCK_SAMPLES * 2U]; /* re-interleaved for arm_cmplx_mag_f32 */
static float32_t s_env[SDR_RX_BLOCK_SAMPLES];        /* |I+jQ|, then DC-blocked, then audio-LPF'd */

/* Output assembly buffer: one TX half (stereo interleaved). Static -
 * this runs in ISR context, keep it off the stack. */
static int16_t s_audio_out[SDR_RX_BLOCK_SAMPLES * 2U];

void demod_am_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(SPK_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPK_EN_PIN);
    gpio_output_options_set(SPK_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, SPK_EN_PIN);
    gpio_bit_set(SPK_EN_PORT, SPK_EN_PIN); /* speaker amp ON */

    /* arm_biquad_cascade_df1_init_f32() zeroes the state buffer it's
     * given, so we don't need to memset s_chf_i_state/etc ourselves. */
    arm_biquad_cascade_df1_init_f32(&s_chf_i_inst, CHF_STAGES, CHF_COEFFS, s_chf_i_state);
    arm_biquad_cascade_df1_init_f32(&s_chf_q_inst, CHF_STAGES, CHF_COEFFS, s_chf_q_state);
    arm_biquad_cascade_df1_init_f32(&s_alpf_inst, ALPF_STAGES, ALPF_COEFFS, s_alpf_state);
    arm_biquad_cascade_df1_init_f32(&s_alpf_4k0_inst, ALPF_4K0_STAGES, ALPF_4K0_COEFFS, s_alpf_4k0_state);
    arm_biquad_cascade_df1_init_f32(&s_alpf_2k3_inst, ALPF_2K3_STAGES, ALPF_2K3_COEFFS, s_alpf_2k3_state);
    arm_biquad_cascade_df1_init_f32(&s_alpf_1k8_inst, ALPF_1K8_STAGES, ALPF_1K8_COEFFS, s_alpf_1k8_state);
    arm_biquad_cascade_df1_init_f32(&s_wfm_alpf_inst, WFM_ALPF_STAGES, WFM_ALPF_COEFFS, s_wfm_alpf_state);
    arm_biquad_cascade_df1_init_f32(&s_nfm_chf_i_inst, NFM_CHF_STAGES, NFM_CHF_COEFFS, s_nfm_chf_i_state);
    arm_biquad_cascade_df1_init_f32(&s_nfm_chf_q_inst, NFM_CHF_STAGES, NFM_CHF_COEFFS, s_nfm_chf_q_state);

    /* WFM discriminator/de-emphasis state - see the field comments
     * above. A stale s_fm_i_prev/s_fm_q_prev only matters for the
     * very first sample after boot or after switching INTO WFM mode
     * (one glitched sample from a bogus delta), same tradeoff the SSB
     * delay-match history below makes; not worth guarding with a
     * mode-switch reset. */
    s_fm_i_prev = 0.0f;
    s_fm_q_prev = 0.0f;
    s_wfm_deemph_y1 = 0.0f;
    /* WFM's own DC blocker/AGC state (05/08/2026, separate processing
     * path - see demod_wfm_process_raw()'s comment) - same
     * "not worth a mode-switch reset" reasoning as above. */
    s_wfm_dcb_x1 = 0.0f;
    s_wfm_dcb_y1 = 0.0f;
    s_wfm_agc_peak = WFM_AGC_PEAK_MIN;

    /* SSB decimated chain. The decimate/interpolate inits VALIDATE
     * their arguments (blockSize%M, numTaps%L) and return a status -
     * check it and shout over UART if it ever fails, because a failed
     * init here leaves the instance unusable and the SSB path would
     * crash or garbage out with no other clue. The plain FIR init has
     * no failure mode (returns void). All three zero their state
     * buffers internally. */
    if (arm_fir_decimate_init_f32(&s_decim_i_inst, DECIM_COEFFS_TAPS, DECIM_FACTOR,
                                    DECIM_COEFFS, s_decim_i_state,
                                    SDR_RX_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("demod_am: *** decimator I init FAILED (blockSize %% M != 0?) ***\n");
    }
    if (arm_fir_decimate_init_f32(&s_decim_q_inst, DECIM_COEFFS_TAPS, DECIM_FACTOR,
                                    DECIM_COEFFS, s_decim_q_state,
                                    SDR_RX_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("demod_am: *** decimator Q init FAILED ***\n");
    }
    arm_fir_init_f32(&s_hilbert_inst, HILBERT_COEFFS_TAPS, HILBERT_COEFFS,
                      s_hilbert_state, DEC_BLOCK_SAMPLES);
    if (arm_fir_interpolate_init_f32(&s_interp_inst, DECIM_FACTOR, INTERP_COEFFS_TAPS,
                                       INTERP_COEFFS, s_interp_state,
                                       DEC_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("demod_am: *** interpolator init FAILED (numTaps %% L != 0?) ***\n");
    }

    /* NR (Spectral Subtraction, AM/USB/LSB only) - see this file's NR
     * INTEGRATION comment above s_nr_decim_inst. Own instances, SAME
     * coefficient tables as the SSB chain just above. */
    if (arm_fir_decimate_init_f32(&s_nr_decim_inst, DECIM_COEFFS_TAPS, DECIM_FACTOR,
                                    DECIM_COEFFS, s_nr_decim_state,
                                    SDR_RX_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("demod_am: *** NR decimator init FAILED (blockSize %% M != 0?) ***\n");
    }
    if (arm_fir_interpolate_init_f32(&s_nr_interp_inst, DECIM_FACTOR, INTERP_COEFFS_TAPS,
                                       INTERP_COEFFS, s_nr_interp_state,
                                       DEC_BLOCK_SAMPLES) != ARM_MATH_SUCCESS) {
        debug_print("demod_am: *** NR interpolator init FAILED (numTaps %% L != 0?) ***\n");
    }
    nr_ss_init();
    rtty_init();
    rtty_scope_init();
    {
        uint32_t k;
        for (k = 0; k < HILBERT_GROUP_DELAY_DEC; k++) {
            s_i_delay_hist[k] = 0.0f;
        }
    }

    s_dcb_x1 = 0.0f;
    s_dcb_y1 = 0.0f;
    s_agc_peak = AGC_TARGET; /* start at unity-ish gain, settles fast */
    s_squelch_level_db = s_squelch_threshold_db; /* no false "signal detected" blip before the first real block */
    s_squelch_open = 1U; /* matches the OFF-by-default threshold - see demod_am.h */

    debug_print("demod_am: PB7 speaker enable high, CMSIS-DSP biquads + decimated "
                "SSB chain init, state reset\n");
}

/*
 * Shared FM discriminator (delay-and-conjugate-multiply -> atan2f(),
 * see demod_am.h's WFM note for the full derivation) - factored out
 * 31/07/2026 when NFM was added, so WFM and NFM don't carry two
 * copies of the same loop that differ only in which I/Q buffers feed
 * it and what gain scales the output. Uses/updates the SHARED
 * s_fm_i_prev/s_fm_q_prev state (see its own comment - safe since WFM
 * and NFM are never active at once). `i_buf`/`q_buf` are read only;
 * `env` is written, and may safely alias `i_buf`/`q_buf`'s underlying
 * storage in the caller since this never reads env[n] after writing
 * it (matches this file's other in-place filter calls' aliasing
 * reasoning).
 */
static void fm_discriminate(const float32_t *i_buf, const float32_t *q_buf,
                             float32_t *env, float32_t gain, uint32_t n_samples)
{
    float32_t i_prev = s_fm_i_prev;
    float32_t q_prev = s_fm_q_prev;
    uint32_t n;

    for (n = 0; n < n_samples; n++) {
        float32_t i_now = i_buf[n];
        float32_t q_now = q_buf[n];
        /* z[n]*conj(z[n-1]): re = I.I'+Q.Q', im = Q.I'-I.Q' */
        float32_t re = i_now * i_prev + q_now * q_prev;
        float32_t im = q_now * i_prev - i_now * q_prev;

        env[n] = atan2f(im, re) * gain;

        i_prev = i_now;
        q_prev = q_now;
    }
    s_fm_i_prev = i_prev;
    s_fm_q_prev = q_prev;
}

/*
 * WFM's OWN, SEPARATE processing path - added 05/08/2026, per the
 * project owner's "ruta separada" decision, when WFM was reactivated
 * at its native 192kHz/512-sample rate while AM/SSB/NFM stayed at
 * 48kHz/128 samples in demod_am_process_raw() above. Registered as
 * the sdr_rx block hook (sdr_rx_set_block_hook()) ONLY while WFM is
 * the active mode - main.c swaps the hook (and the codec's own clock
 * registers, and sdr_rx's/gd32_i2s's block-size configuration) on
 * every mode transition into or out of WFM. demod_am_process_raw()
 * above is NEVER called while WFM is selected, and this function is
 * NEVER called otherwise - the two do not interleave.
 *
 * Mirrors the WFM-specific steps demod_am_process_raw() used to
 * contain before this split (deinterleave -> discriminate -> de-
 * emphasis -> audio LPF -> DC blocker -> AGC -> output), at
 * SDR_RX_BLOCK_SAMPLES_WFM (512) instead of SDR_RX_BLOCK_SAMPLES
 * (128), using WFM's own dedicated buffers/state (s_wfm_*) and its
 * own 192kHz-tuned constants (WFM_DCB_R, WFM_AGC_RELEASE_*) instead of
 * the 48kHz ones AM/SSB/NFM use. No down-mix, no channel filter, no
 * squelch, no NR - see demod_am.h's WFM note for why WFM never used
 * the first two even before this split, and squelch/NR were never
 * wired up for WFM either (see their own comments).
 */
void demod_wfm_process_raw(const int16_t *raw_interleaved)
{
    uint32_t n;
    uint8_t muted;
    uint8_t do_log;

    /*
     * *** 05/08/2026, mitigation added after exhausting the practical
     * root-cause investigation (see sdr_rx_last_block_corrupted()'s own
     * comment) *** - a block that arrived with SPI_STAT_FERR set may
     * have samples misassigned between channels; skip it entirely
     * rather than feed it through this phase-sensitive discriminator.
     * Deliberately returns before touching ANY state below (mute
     * countdown, diag counters, AGC) so the next good block picks up
     * exactly where the last good one left off - this block's slot in
     * the TX ping-pong buffer simply keeps whatever it already had
     * from last time (gd32_i2s_stream_write_half() never gets called
     * for this block), which repeats ~2.67ms of already-played audio
     * rather than passing a corrupted one through - inaudible as a
     * glitch, unlike letting the raw discriminator output through.
     */
    if (sdr_rx_last_block_corrupted()) {
        return;
    }

    /* Clipping scan on the RAW block, before anything reinterprets
     * these samples - see rf_clip_scan()'s comment. After the
     * corrupted-block guard on purpose: a FERR block's samples may be
     * channel-misassigned garbage, not a meaningful clipping read. */
    rf_clip_scan(raw_interleaved, SDR_RX_BLOCK_SAMPLES_WFM);

    muted = (s_wfm_mute_remaining > 0U) ? 1U : 0U;
    do_log = wfm_should_log_diag(); /* decide BEFORE incrementing below */

    if (muted) {
        s_wfm_mute_remaining--;
    }
    if (do_log) {
        debug_print_dec("demod_wfm: block# (muted=1/0 follows)", s_wfm_diag_count);
        debug_print_dec("demod_wfm: muted", (uint32_t)muted);
    }
    s_wfm_diag_count++; /* unconditional - see wfm_should_log_diag()'s comment */

    /*
     * Raw sample dump - added 05/08/2026 after AGC data looked fully
     * converged/reasonable on both sides of a switch (WFM and AM)
     * while the project owner still heard the exact same "ruido/
     * pitido fuerte, sin voz reconocible" as before any of the AGC/
     * mute fixes - meaning those fixes aren't the issue. min/max
     * alone can't distinguish real signal from scrambled-but-
     * plausible-magnitude noise (e.g. an I2S word-select/frame
     * alignment glitch on re-enable would produce exactly that: bit-
     * shifted samples that still fall in a "normal" numeric range but
     * carry no real waveform). Dumping the actual sequential raw I/Q
     * values - only at block 0 of each entry, only when logging - is
     * the next thing needed to tell "genuinely bad samples" apart
     * from "fine samples, something else is wrong downstream".
     */
    if (do_log && s_wfm_diag_count == 1U) {
        for (n = 0; n < 8U; n++) {
            debug_print_dec_signed_local("demod_wfm: raw I", (int32_t)raw_interleaved[2U * n]);
            debug_print_dec_signed_local("demod_wfm: raw Q", (int32_t)raw_interleaved[2U * n + 1U]);
        }
    }

    {
    float dcb_x1 = s_wfm_dcb_x1;
    float dcb_y1 = s_wfm_dcb_y1;
    float peak = s_wfm_agc_peak;
    uint32_t cyc_start = DWT->CYCCNT;

    /* 0a. Deinterleave - same plain-cast, no-normalization convention
     * as demod_am_process_raw()'s own step 0a. */
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
        s_wfm_i_buf[n] = (float32_t)raw_interleaved[2U * n];
        s_wfm_q_buf[n] = (float32_t)raw_interleaved[2U * n + 1U];
    }

    /* 1. Discriminate - delay-and-conjugate-multiply, straight on the
     * RAW (unfiltered, un-down-mixed) I/Q, at the full 192kHz rate -
     * see demod_am.h's WFM note for the full derivation and why
     * atan2f() rather than arm_atan2_f32() here. */
    fm_discriminate(s_wfm_i_buf, s_wfm_q_buf, s_wfm_env, WFM_DISC_GAIN, SDR_RX_BLOCK_SAMPLES_WFM);

    if (do_log) {
        float mn = s_wfm_env[0], mx = s_wfm_env[0];
        for (n = 1; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            if (s_wfm_env[n] < mn) { mn = s_wfm_env[n]; }
            if (s_wfm_env[n] > mx) { mx = s_wfm_env[n]; }
        }
        debug_print_dec_signed_local("demod_wfm: [1] raw discriminator min", (int32_t)mn);
        debug_print_dec_signed_local("demod_wfm: [1] raw discriminator max", (int32_t)mx);
    }

    /* 2. DC blocker - own state (s_wfm_dcb_x1/y1), own coefficient
     * (WFM_DCB_R, tuned for 192kHz - see its comment). *** FIX
     * 05/08/2026 ***: this MUST run right here, immediately after the
     * discriminator and BEFORE de-emphasis/the audio LPF below - this
     * function's first draft had it AFTER those two stages instead,
     * which doesn't match the original, already-validated pre-
     * migration WFM chain (discriminate -> DC-block -> de-emphasis ->
     * ALPF) and is the leading suspect for the project owner's "suena
     * a NFM" report once the clock itself was confirmed correct
     * (BCLK=6.144MHz/WCLK=192kHz, measured) - de-emphasis is itself a
     * lowpass, so running it before the DC blocker lets whatever DC
     * bias the discriminator outputs sit in the signal (and interact
     * with two more filter stages' internal state) for a full extra
     * block before ever getting removed, instead of being stripped
     * immediately the way the validated design does it. */
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
        float y = s_wfm_env[n] - dcb_x1 + WFM_DCB_R * dcb_y1;
        dcb_x1 = s_wfm_env[n];
        dcb_y1 = y;
        s_wfm_env[n] = y;
    }

    /* 3a. De-emphasis, single-pole LPF (see WFM_DEEMPH_ALPHA's
     * comment) - scalar, same reasoning as the DC blocker above for
     * not using a CMSIS block call. */
    {
        float y1 = s_wfm_deemph_y1;

        for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            y1 = WFM_DEEMPH_ALPHA * y1 + (1.0f - WFM_DEEMPH_ALPHA) * s_wfm_env[n];
            s_wfm_env[n] = y1;
        }
        s_wfm_deemph_y1 = y1;
    }

    /* 3b. WFM audio LPF, 4th-order Butterworth ~15kHz (see
     * WFM_ALPF_COEFFS' comment) - in-place CMSIS call. */
    arm_biquad_cascade_df1_f32(&s_wfm_alpf_inst, s_wfm_env, s_wfm_env, SDR_RX_BLOCK_SAMPLES_WFM);

    if (do_log) {
        float mn = s_wfm_env[0], mx = s_wfm_env[0];
        for (n = 1; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            if (s_wfm_env[n] < mn) { mn = s_wfm_env[n]; }
            if (s_wfm_env[n] > mx) { mx = s_wfm_env[n]; }
        }
        debug_print_dec_signed_local("demod_wfm: [2] post-filter (pre-AGC) min", (int32_t)mn);
        debug_print_dec_signed_local("demod_wfm: [2] post-filter (pre-AGC) max", (int32_t)mx);
        debug_print_dec("demod_wfm: [2] s_wfm_agc_peak going in", (uint32_t)s_wfm_agc_peak);
        debug_print_dec("demod_wfm: [2] s_agc_profile (0=MAN,1=SLOW,2=MED,3=FAST)", (uint32_t)s_agc_profile);
    }

    /* 4. AGC (instant attack, slow release) + 5. clamp/duplicate to
     * L/R int16 - own state (s_wfm_agc_peak), own release coefficient
     * (s_wfm_agc_release, resolved from the SAME s_agc_profile
     * selector as AM/SSB/NFM's s_agc_release - see
     * demod_am_set_agc_profile()'s comment). Same MANUAL-bypass
     * structure as demod_am_process_raw()'s own step 4. */
    if (s_agc_profile == AGC_PROFILE_MANUAL) {
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            int32_t out = (int32_t)(s_wfm_env[n] * DEMOD_AM_GAIN);

            if (out > 32767)  { out = 32767; }
            if (out < -32768) { out = -32768; }
            s_wfm_audio_out[2U * n]      = (int16_t)out;
            s_wfm_audio_out[2U * n + 1U] = (int16_t)out;
        }
    } else {
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            float mag = (s_wfm_env[n] < 0.0f) ? -s_wfm_env[n] : s_wfm_env[n];
            float y;
            int32_t out;

            peak *= s_wfm_agc_release;
            if (mag > peak) { peak = mag; }
            {
                float pk = (peak < WFM_AGC_PEAK_MIN) ? WFM_AGC_PEAK_MIN : peak;
                float gain = AGC_TARGET / pk;
                if (gain > AGC_GAIN_MAX) { gain = AGC_GAIN_MAX; }
                y = s_wfm_env[n] * gain * DEMOD_AM_GAIN;
            }

            out = (int32_t)y;
            if (out > 32767)  { out = 32767; }
            if (out < -32768) { out = -32768; }
            s_wfm_audio_out[2U * n]      = (int16_t)out;
            s_wfm_audio_out[2U * n + 1U] = (int16_t)out;
        }
        s_wfm_agc_peak = peak;
    }

    s_wfm_dcb_x1 = dcb_x1;
    s_wfm_dcb_y1 = dcb_y1;

    if (do_log) {
        int16_t mn = s_wfm_audio_out[0], mx = s_wfm_audio_out[0];
        for (n = 1; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            if (s_wfm_audio_out[2U * n] < mn) { mn = s_wfm_audio_out[2U * n]; }
            if (s_wfm_audio_out[2U * n] > mx) { mx = s_wfm_audio_out[2U * n]; }
        }
        debug_print_dec_signed_local("demod_wfm: [3] final int16 output min", (int32_t)mn);
        debug_print_dec_signed_local("demod_wfm: [3] final int16 output max", (int32_t)mx);
    }

    if (muted) {
        /* Silence the OUTPUT only - everything above (discriminator,
         * DC blocker, de-emphasis, ALPF, AGC) already ran normally on
         * the real data and updated its state accordingly, exactly as
         * it would with the mute removed entirely - see
         * WFM_SETTLE_MUTE_BLOCKS' comment for why that's the whole
         * point. s_wfm_audio_out's real (pre-mute) values were already
         * captured by the diagnostic block just above, so the debug
         * log still shows what the AGC actually computed even while
         * this zeroes out what's actually heard. */
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES_WFM; n++) {
            s_wfm_audio_out[2U * n]      = 0;
            s_wfm_audio_out[2U * n + 1U] = 0;
        }
    }

    gd32_i2s_stream_write_half(s_wfm_audio_out);

    s_wfm_last_cycles = DWT->CYCCNT - cyc_start;
    }
}

void demod_am_process_raw(const int16_t *raw_interleaved)
{
    uint32_t n;
    float dcb_x1;
    float dcb_y1;
    float peak;
    uint32_t cyc_start;
    uint8_t do_log;
    uint8_t muted;
    uint32_t nr_ssb_cycles = 0U;

    /*
     * *** 05/08/2026, mitigation - see demod_wfm_process_raw()'s
     * identical guard and sdr_rx_last_block_corrupted()'s comment for
     * the full reasoning *** - skip a block that arrived with
     * SPI_STAT_FERR before touching any state, so the TX buffer slot
     * just keeps its previous contents (a brief, inaudible repeat)
     * instead of playing out possibly channel-misassigned samples.
     */
    if (sdr_rx_last_block_corrupted()) {
        return;
    }

    /* Clipping scan on the RAW block, before anything reinterprets
     * these samples - see rf_clip_scan()'s comment. After the
     * corrupted-block guard on purpose: a FERR block's samples may be
     * channel-misassigned garbage, not a meaningful clipping read. */
    rf_clip_scan(raw_interleaved, SDR_RX_BLOCK_SAMPLES);

    dcb_x1 = s_dcb_x1;
    dcb_y1 = s_dcb_y1;
    peak = s_agc_peak;
    cyc_start = DWT->CYCCNT;
    do_log = am_should_log_diag(); /* decide BEFORE incrementing below */
    muted = (s_am_mute_remaining > 0U) ? 1U : 0U; /* see AM_SETTLE_MUTE_BLOCKS' comment */
    /* NR cost when done INLINE on s_ssb_dec for USB/LSB (see step 1's
     * SSB branch, and the 3b. NR block's comment below for why SSB
     * doesn't go through the separate decimate/interpolate path AM
     * uses). Stays 0 for every other case (AM, WFM, NFM, or SSB with
     * NR off) - see the 3b. NR block for how it's actually used. */

    if (do_log) {
        debug_print_dec("demod_am: block# (mode 0=AM,1=USB,2=LSB,3=NFM follows)",
                         s_am_diag_count);
        debug_print_dec("demod_am: mode", (uint32_t)s_mode);
    }
    s_am_diag_count++; /* unconditional - see am_should_log_diag()'s comment */
    if (muted) {
        s_am_mute_remaining--;
    }

    /* Raw sample dump - see demod_wfm_process_raw()'s identical block
     * for the full reasoning (same suspicion, same test, both sides
     * of every switch). */
    if (do_log && s_am_diag_count == 1U) {
        for (n = 0; n < 8U; n++) {
            debug_print_dec_signed_local("demod_am: raw I", (int32_t)raw_interleaved[2U * n]);
            debug_print_dec_signed_local("demod_am: raw Q", (int32_t)raw_interleaved[2U * n + 1U]);
        }
    }

    /* 0a. Deinterleave raw int16 L/R (I/Q) into separate float rails.
     * Plain cast, same scale as before (no -1..1 normalization) so
     * the AGC's int16-scale targeting downstream doesn't change. */
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
        s_i_buf[n] = (float32_t)raw_interleaved[2U * n];
        s_q_buf[n] = (float32_t)raw_interleaved[2U * n + 1U];
    }

    /* 0b/0c: LOW-IF DOWN-MIX + CHANNEL FILTER - unconditional now
     * (05/08/2026): WFM moved to its OWN separate processing path
     * (demod_wfm_process_raw(), see its comment) with its own hook,
     * registered/unregistered via sdr_rx_set_block_hook() on mode
     * switch (see main.c) - this function is never even CALLED while
     * WFM is selected anymore, so the old "if (s_mode != WFM)" guard
     * here was becoming dead-but-misleading code, not a real
     * conditional. NFM stays in this branch (it needs the down-mix
     * just as much as AM/SSB do) - it just gets its OWN, wider channel
     * filter instead of CHF_COEFFS, selected inside 0c below - see
     * demod_am.h's NFM note. */
    {
        /* 0b. LOW-IF DOWN-MIX (SR/4 rotation, see the block comment
         * above s_if_offset_active): brings the wanted signal back to DC
         * so CHF_COEFFS (designed for a DC-centered signal) still applies
         * unchanged. Only runs while the LO is actually tuned with the
         * offset - see demod_am.h's LOW-IF TUNING note on why this must
         * stay in sync with the real LO instead of always-on. No
         * multiplications, just sign flips and swaps, in place. */
        if (s_if_offset_active) {
            for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n += 4U) {
                float32_t hh1, hh2;

                /* n+0: identity, leave as-is. */

                /* n+1: (I,Q) -> (Q, -I), i.e. x -j */
                hh1 =  s_q_buf[n + 1U];
                hh2 = -s_i_buf[n + 1U];
                s_i_buf[n + 1U] = hh1;
                s_q_buf[n + 1U] = hh2;

                /* n+2: (I,Q) -> (-I, -Q), i.e. x -1 */
                hh1 = -s_i_buf[n + 2U];
                hh2 = -s_q_buf[n + 2U];
                s_i_buf[n + 2U] = hh1;
                s_q_buf[n + 2U] = hh2;

                /* n+3: (I,Q) -> (-Q, I), i.e. x +j */
                hh1 = -s_q_buf[n + 3U];
                hh2 =  s_i_buf[n + 3U];
                s_i_buf[n + 3U] = hh1;
                s_q_buf[n + 3U] = hh2;
            }
        }

        /* 0c. CHANNEL FILTER: identical 4th-order Butterworth LPF on I
         * and Q via CMSIS-DSP (in-place: pSrc==pDst is safe here, see
         * arm_biquad_cascade_df1_f32.c - each sample is read before it's
         * overwritten, no lookahead). Complex LPF centered on the VFO.
         * NFM uses its OWN, wider filter (NFM_CHF_COEFFS) instead of
         * CHF_COEFFS - see demod_am.h's NFM note on why AM/SSB's ~4kHz
         * corner would clip narrowband FM's modulated sidebands. */
        if (s_mode == (uint8_t)DEMOD_MODE_NFM) {
            arm_biquad_cascade_df1_f32(&s_nfm_chf_i_inst, s_i_buf, s_i_buf, SDR_RX_BLOCK_SAMPLES);
            arm_biquad_cascade_df1_f32(&s_nfm_chf_q_inst, s_q_buf, s_q_buf, SDR_RX_BLOCK_SAMPLES);
        } else {
            arm_biquad_cascade_df1_f32(&s_chf_i_inst, s_i_buf, s_i_buf, SDR_RX_BLOCK_SAMPLES);
            arm_biquad_cascade_df1_f32(&s_chf_q_inst, s_q_buf, s_q_buf, SDR_RX_BLOCK_SAMPLES);
        }
    }

    s_last_cycles_frontend = DWT->CYCCNT - cyc_start;

    /* 1. Audio extraction - branches by mode. All paths write into
     * s_env[], which steps 2-5 below (DC blocker, audio LPF, AGC,
     * output) consume - WFM's LPF sub-step differs, see step 3. */
    if (s_mode == (uint8_t)DEMOD_MODE_AM) {
        /* |z| = |I + jQ| via arm_cmplx_mag_f32 (hardware VSQRT.F32).
         * Needs the filtered I/Q re-interleaved into CMSIS's complex
         * format first. */
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            s_iq_cplx[2U * n]      = s_i_buf[n];
            s_iq_cplx[2U * n + 1U] = s_q_buf[n];
        }
        arm_cmplx_mag_f32(s_iq_cplx, s_env, SDR_RX_BLOCK_SAMPLES);
    } else if (s_mode == (uint8_t)DEMOD_MODE_NFM) {
        /* NFM: same discriminator as WFM (see fm_discriminate()'s
         * comment), but on the down-mixed + NFM_CHF_COEFFS-filtered
         * I/Q from steps 0b/0c above, not the raw signal - see
         * demod_am.h's NFM note on why NFM keeps that narrowband
         * front-end instead of skipping it like WFM does. */
        fm_discriminate(s_i_buf, s_q_buf, s_env, NFM_DISC_GAIN, SDR_RX_BLOCK_SAMPLES);
    } else {
        /* SSB (USB/LSB), phasing method at a DECIMATED rate - see the
         * PIPELINE comment above DECIM_COEFFS and demod_am.h's SSB
         * note. */
        float32_t sign = (s_mode == (uint8_t)DEMOD_MODE_USB) ? SSB_USB_SIGN : SSB_LSB_SIGN;
        uint32_t k;

        /* 1. Decimate channel-filtered I and Q by DECIM_FACTOR (4): 128
         * samples @ 48kHz -> 32 samples @ 12kHz (was by 16: 512 @
         * 192kHz -> 32 @ 12kHz before 04/08/2026 - see sdr_rx.h's
         * SDR_RX_BLOCK_SAMPLES comment), anti-aliased by DECIM_COEFFS
         * inside the same call. */
        arm_fir_decimate_f32(&s_decim_i_inst, s_i_buf, s_i_dec, SDR_RX_BLOCK_SAMPLES);
        arm_fir_decimate_f32(&s_decim_q_inst, s_q_buf, s_q_dec, SDR_RX_BLOCK_SAMPLES);

        /* 2a. Hilbert-shift the decimated Q (90 degrees across the
         * audio band, now with proper coverage down to ~300Hz - see
         * the design numbers above HILBERT_COEFFS). */
        arm_fir_f32(&s_hilbert_inst, s_q_dec, s_q_hilbert_out, DEC_BLOCK_SAMPLES);

        /* 2b. Delay-match the decimated I by the Hilbert's group
         * delay: history (previous block's tail) followed by the
         * current block, so s_i_delayed[k] for k=0..DEC_BLOCK_SAMPLES-1
         * equals I_dec[k - HILBERT_GROUP_DELAY_DEC] across the block
         * boundary. */
        for (k = 0; k < HILBERT_GROUP_DELAY_DEC; k++) {
            s_i_delayed[k] = s_i_delay_hist[k];
        }
        for (k = 0; k < DEC_BLOCK_SAMPLES; k++) {
            s_i_delayed[HILBERT_GROUP_DELAY_DEC + k] = s_i_dec[k];
        }
        for (k = 0; k < HILBERT_GROUP_DELAY_DEC; k++) {
            s_i_delay_hist[k] = s_i_dec[DEC_BLOCK_SAMPLES - HILBERT_GROUP_DELAY_DEC + k];
        }

        /* 2c. Combine: one sideband adds, the other cancels. */
        for (k = 0; k < DEC_BLOCK_SAMPLES; k++) {
            s_ssb_dec[k] = s_i_delayed[k] + sign * s_q_hilbert_out[k];
        }

        /*
         * --- RTTY INTEGRATION (added 08/08/2026, first draft) ---
         *
         * USB/LSB only, per the project owner - RTTY is two audio
         * tones inside an SSB passband, there's no meaning to it in
         * AM/NFM/WFM. Reads s_ssb_dec BEFORE NR (2d, right below)
         * touches it - deliberately RAW SSB audio, not NR'd: spectral
         * subtraction is tuned for voice intelligibility, and could
         * smear or distort two narrow FSK tones in ways that hurt
         * Goertzel tone detection (rtty.c's whole decode depends on
         * accurately telling two specific frequencies apart) more
         * than it helps. Same "s_ssb_dec is ALREADY at the rate this
         * wants, no extra decimation needed" reuse as NR just above -
         * see nr_ss_get_enabled()'s comment for that reasoning, it
         * applies here identically. Gated on rtty_get_enabled() the
         * same way NR is: skipped entirely while off, not just a
         * transparent no-op - zero extra ISR cycles (see rtty.h's
         * RTTY_ENABLED comment for how this defaults to on/off).
         *
         * READ-ONLY - rtty_process() never modifies s_ssb_dec, unlike
         * nr_ss_process() right below, so call order relative to NR
         * only matters for what RTTY sees, not for correctness of
         * what NR sees afterward.
         */
        if (rtty_get_enabled()) {
            rtty_process(s_ssb_dec, DEC_BLOCK_SAMPLES);
            /* Same buffer, same reasoning as rtty_process() just above
             * (raw pre-NR audio) - feeds the tuning scope's own
             * accumulator. See rtty_scope.h for why this is a
             * SEPARATE FFT from both fft.c's real-time one and
             * rtty.c's own Goertzel detectors. Cheap: just an
             * accumulate-into-a-ring, the actual FFT runs later from
             * the main loop (rtty_scope_poll()), never here. */
            rtty_scope_feed(s_ssb_dec, DEC_BLOCK_SAMPLES);
        }

        /* 2d. NR (Spectral Subtraction), USB/LSB - IN PLACE on
         * s_ssb_dec, which is ALREADY at the 12kHz rate nr_ss_process()
         * wants (see nr_ss.h's NR_SS_BLOCK_SAMPLES comment) - unlike
         * AM below, SSB needs NO separate decimate/interpolate pair
         * around it: piggybacking on this chain's own existing
         * decimation avoids interpolating this audio up to 192kHz
         * (step 3, right below) only to immediately decimate it back
         * down again for NR and interpolate AGAIN - a wasted round
         * trip discovered 04/08/2026 once NR made the ISR "reach the
         * controller's limit" in SSB specifically (the project owner's
         * own words) - this is that fix. AM has no equivalent
         * already-decimated buffer to reuse (its envelope only ever
         * exists at the full 512-sample/192kHz s_env[]), so it still
         * goes through its own dedicated decimate/interpolate pair -
         * see the 3b. NR block below. */
        if (nr_ss_get_enabled()) {
            uint32_t nr_t0 = DWT->CYCCNT;
            nr_ss_process(s_ssb_dec, DEC_BLOCK_SAMPLES);
            nr_ssb_cycles = DWT->CYCCNT - nr_t0;
        }

        /* 3. Interpolate the 12kHz SSB audio (NR'd above, if it was on)
         * back up to 192kHz (32 samples -> 512), straight into s_env[] -
         * everything downstream (DC blocker, ALPF, AGC, output) is
         * shared with AM, unchanged. INTERP_COEFFS carries the x16
         * gain that compensates the zero-stuffing loss (see the
         * PIPELINE comment - without it this comes out at 1/16
         * volume). */
        arm_fir_interpolate_f32(&s_interp_inst, s_ssb_dec, s_env, DEC_BLOCK_SAMPLES);
    }

    /*
     * --- Squelch (AM + NFM only - see demod_am.h's squelch block
     * comment) --- RF-level metric: average magnitude of the SAME
     * down-mixed, channel-filtered I/Q that just fed AM's envelope
     * detector or NFM's discriminator above (s_i_buf/s_q_buf are
     * read-only in both, still valid here) - a hardware-VSQRT
     * sqrtf() per sample, same cost class as AM's own
     * arm_cmplx_mag_f32() call, just done as a scalar accumulation
     * since only the block's average is needed here, not a full
     * per-sample magnitude array.
     *
     * WFM and SSB don't get a squelch: WFM's front-end skips the
     * channel filter entirely (see demod_am.h's WFM note), so there's
     * no equivalent "in-channel RF level" to measure the same way;
     * SSB could in principle reuse this exact metric (it shares the
     * same down-mix + CHF_COEFFS front-end AM does), but wasn't asked
     * for and isn't wired up - straightforward to add the same way if
     * it's ever wanted.
     */
    if (s_mode == (uint8_t)DEMOD_MODE_AM || s_mode == (uint8_t)DEMOD_MODE_NFM) {
        float32_t sum = 0.0f;
        float mag_db;
        float half_hyst = SQUELCH_HYSTERESIS_DB * 0.5f;

        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            sum += sqrtf(s_i_buf[n] * s_i_buf[n] + s_q_buf[n] * s_q_buf[n]);
        }
        /* +1.0f before the log so true silence (sum=0) gives
         * 20*log10(1)=0dB instead of log10(0)=-inf - a harmless
         * floor, not a meaningful "0dB" reading. */
        mag_db = 20.0f * log10f((sum / (float)SDR_RX_BLOCK_SAMPLES) + 1.0f);

        s_squelch_level_db = SQUELCH_LEVEL_ALPHA * s_squelch_level_db
                              + (1.0f - SQUELCH_LEVEL_ALPHA) * mag_db;

        /* Schmitt-trigger hysteresis: different trip points for
         * opening vs closing so a signal sitting right at the
         * threshold doesn't chatter block-to-block. */
        if (s_squelch_open) {
            if (s_squelch_level_db < s_squelch_threshold_db - half_hyst) {
                s_squelch_open = 0U;
            }
        } else {
            if (s_squelch_level_db > s_squelch_threshold_db + half_hyst) {
                s_squelch_open = 1U;
            }
        }

        if (!s_squelch_open) {
            /* Muted: silence this block's audio before it reaches the
             * DC blocker/audio LPF/AGC below - cheaper and simpler
             * than threading a mute flag through every downstream
             * stage, and those stages handle a silent block
             * gracefully already (the AGC peak just decays toward
             * AGC_PEAK_MIN, same as any quiet signal). */
            for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
                s_env[n] = 0.0f;
            }
        }
    }

    {
        uint32_t cyc_now = DWT->CYCCNT;
        /* nr_ssb_cycles subtracted out - see this function's top
         * comment on it, and the 2d. NR block above - otherwise SSB's
         * NR cost would silently hide inside "extract" instead of
         * showing up in its own "nr" bucket the way AM's does. */
        s_last_cycles_extract = cyc_now - cyc_start - s_last_cycles_frontend - nr_ssb_cycles;
    }

    /* 2. DC blocker (removes the carrier level in AM; a harmless
     * small-DC-bias cleanup in SSB, which has no carrier). Scalar -
     * cheap, stateful single-pole HPF, not worth a CMSIS block call. */
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
        float y = s_env[n] - dcb_x1 + DCB_R * dcb_y1;
        dcb_x1 = s_env[n];
        dcb_y1 = y;
        s_env[n] = y;
    }

    /* 3. Audio LPF, 4th-order Butterworth via CMSIS-DSP (in-place,
     * same reasoning as the channel filter above). WFM no longer
     * reaches this function at all as of 05/08/2026 (see
     * demod_wfm_process_raw()) - the old "if WFM {...} else {...}"
     * split here collapsed to just this AM/SSB/NFM path.
     *
     * NFM: unconditionally s_alpf_inst (ALPF_COEFFS, ~6kHz) -
     * unaffected by the AM/SSB selector below even though it lands
     * in this same branch. NFM's own channel filter (NFM_CHF_COEFFS)
     * already sets its bandwidth upstream, and letting the AM/SSB
     * selector also touch NFM's audio would narrow narrowband FM voice
     * every time someone picks a tighter AM/SSB filter, which the
     * project owner never asked for - see AUDIO_BW_4K0's comment
     * in demod_am.h for why these are kept as fully separate
     * instances instead of one shared "current audio filter".
     *
     * AM/USB/LSB: whichever of the three s_audio_bw picked - see
     * demod_am_set_audio_bw()'s comment in demod_am.h. */
    {
        if (s_mode == (uint8_t)DEMOD_MODE_NFM) {
            arm_biquad_cascade_df1_f32(&s_alpf_inst, s_env, s_env, SDR_RX_BLOCK_SAMPLES);
        } else {
            arm_biquad_casd_df1_inst_f32 *alpf;

            switch (s_audio_bw) {
            case AUDIO_BW_2K3: alpf = &s_alpf_2k3_inst; break;
            case AUDIO_BW_1K8: alpf = &s_alpf_1k8_inst; break;
            case AUDIO_BW_4K0:
            default:            alpf = &s_alpf_4k0_inst; break;
            }
            arm_biquad_cascade_df1_f32(alpf, s_env, s_env, SDR_RX_BLOCK_SAMPLES);
        }
    }

    {
        uint32_t cyc_now = DWT->CYCCNT;
        s_last_cycles_audio = cyc_now - cyc_start - s_last_cycles_frontend - s_last_cycles_extract;
    }

    /* 3b. NR (Spectral Subtraction), AM ONLY here - USB/LSB already got
     * theirs done back in step 2d, inline on the already-12kHz
     * s_ssb_dec, specifically to AVOID this decimate/interpolate pair
     * (see step 2d's comment for why - the project owner hit the
     * controller's cycle limit in SSB+NR before this split existed).
     * AM has no equivalent already-decimated buffer (its envelope only
     * ever exists at the full s_env[]), so it still needs its own
     * dedicated decimate-down/NR/interpolate-back-up here. Gated the
     * same way as before: only while the bottom bar's NR button has
     * actually turned it on (nr_ss_get_enabled() - a genuine on/off
     * switch, independent of the strength value - see
     * nr_ss_set_enabled()'s comment in nr_ss.h). Placed AFTER the
     * audio LPF (so NR works on already band-limited, DC-free audio,
     * not raw envelope content) and BEFORE the AGC (so the NR strength
     * control's fixed threshold sees a consistent, un-normalized
     * signal level rather than chasing the AGC's own gain changes -
     * the operator tunes the NR strength by ear anyway, the same way
     * they'd tune an RF gain or squelch control, so this ordering
     * isn't load-bearing the way it would be for an automatic/
     * adaptive threshold). */
    if (s_mode == (uint8_t)DEMOD_MODE_AM && nr_ss_get_enabled()) {
        arm_fir_decimate_f32(&s_nr_decim_inst, s_env, s_nr_buf, SDR_RX_BLOCK_SAMPLES);
        nr_ss_process(s_nr_buf, NR_SS_BLOCK_SAMPLES);
        arm_fir_interpolate_f32(&s_nr_interp_inst, s_nr_buf, s_env, DEC_BLOCK_SAMPLES);

        {
            uint32_t cyc_now = DWT->CYCCNT;
            s_last_cycles_nr = cyc_now - cyc_start - s_last_cycles_frontend
                                - s_last_cycles_extract - s_last_cycles_audio;
        }
    } else {
        /* nr_ssb_cycles carries USB/LSB's already-measured step-2d
         * cost here (0 for every other case: AM with NR off, WFM,
         * NFM) - see this function's top comment on that variable. */
        s_last_cycles_nr = nr_ssb_cycles;
    }

    /* 4. AGC (instant attack, slow release) + 5. clamp/duplicate to
     * L/R int16. Scalar - a branchy control loop, not a filter.
     * MANUAL profile bypasses the peak-tracking entirely (fixed unity
     * gain) - see agc_profile_t's MANUAL note in demod_am.h. Branching
     * on s_agc_profile ONCE outside the loop (not per-sample) keeps
     * both paths as cheap as they were before profiles existed. */
    if (s_agc_profile == AGC_PROFILE_MANUAL) {
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            int32_t out = (int32_t)(s_env[n] * DEMOD_AM_GAIN);

            if (out > 32767)  { out = 32767; }
            if (out < -32768) { out = -32768; }
            s_audio_out[2U * n]      = (int16_t)out;
            s_audio_out[2U * n + 1U] = (int16_t)out;
        }
        /* peak/s_agc_peak deliberately NOT updated here - so the
         * S-meter (demod_am_get_signal_peak()) and the automatic
         * profiles both pick up cleanly wherever they left off if you
         * switch back to SLW/MED/FST later, rather than resuming from
         * a stale or zeroed peak. */
    } else {
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            float mag = (s_env[n] < 0.0f) ? -s_env[n] : s_env[n];
            float y;
            int32_t out;

            peak *= s_agc_release;
            if (mag > peak) { peak = mag; }
            {
                float pk = (peak < AGC_PEAK_MIN) ? AGC_PEAK_MIN : peak;
                float gain = AGC_TARGET / pk;
                if (gain > AGC_GAIN_MAX) { gain = AGC_GAIN_MAX; }
                y = s_env[n] * gain * DEMOD_AM_GAIN;
            }

            out = (int32_t)y;
            if (out > 32767)  { out = 32767; }
            if (out < -32768) { out = -32768; }
            s_audio_out[2U * n]      = (int16_t)out;
            s_audio_out[2U * n + 1U] = (int16_t)out;
        }
        s_agc_peak = peak;
    }

    s_dcb_x1 = dcb_x1;
    s_dcb_y1 = dcb_y1;

    if (do_log) {
        int16_t mn = s_audio_out[0], mx = s_audio_out[0];
        for (n = 1; n < SDR_RX_BLOCK_SAMPLES; n++) {
            if (s_audio_out[2U * n] < mn) { mn = s_audio_out[2U * n]; }
            if (s_audio_out[2U * n] > mx) { mx = s_audio_out[2U * n]; }
        }
        debug_print_dec("demod_am: s_agc_peak going out", (uint32_t)s_agc_peak);
        debug_print_dec_signed_local("demod_am: [3] final int16 output min", (int32_t)mn);
        debug_print_dec_signed_local("demod_am: [3] final int16 output max", (int32_t)mx);
    }

    if (muted) {
        /* Silence the OUTPUT only - everything above (DC blocker,
         * audio LPF, AGC, NR) already ran normally on the real data
         * and updated its state accordingly - see AM_SETTLE_MUTE_BLOCKS'
         * comment for why that's the whole point. The diagnostic
         * block just above already captured the real (pre-mute)
         * output values, so the log still shows what the AGC actually
         * computed even while this zeroes out what's actually heard. */
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            s_audio_out[2U * n]      = 0;
            s_audio_out[2U * n + 1U] = 0;
        }
    }

    gd32_i2s_stream_write_half(s_audio_out);

    s_last_cycles = DWT->CYCCNT - cyc_start;
    s_last_cycles_agc_out = s_last_cycles - s_last_cycles_frontend
                             - s_last_cycles_extract - s_last_cycles_audio
                             - s_last_cycles_nr;
}
