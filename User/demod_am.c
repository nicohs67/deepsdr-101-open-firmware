#include "demod_am.h"
#include "sdr_rx.h"
#include "gd32_i2s.h"
#include "gd32f4xx.h"
#include "debug_uart.h"
#include "arm_math.h"
#include <math.h> /* atan2f() for the WFM discriminator - see demod_am.h's
                    * WFM note on why this uses libm instead of
                    * arm_atan2_f32() (not present in this project's
                    * pruned CMSIS-DSP tree). */

#define SPK_EN_PORT GPIOB
#define SPK_EN_PIN  GPIO_PIN_7

/*
 * CHANNEL FILTER + AUDIO LPF, both 4th-order Butterworth low-pass
 * (2 cascaded CMSIS biquad DF1 stages each), designed offline via
 * bilinear transform with prewarping at fs=192kHz:
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
 * design corners and at +/-20kHz/Nyquist) before being embedded here
 * - see the project's filter design notes. To retune either corner,
 * regenerate the full 10-value array; don't hand-edit individual
 * coefficients, the 5 values per stage are coupled.
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
    0.0040740687f, 0.0081481374f, 0.0040740687f,  1.8885559539f, -0.9048522288f,
    0.0038172458f, 0.0076344916f, 0.0038172458f,  1.7695043485f, -0.7847733318f
};

#define ALPF_STAGES 2U
static const float32_t ALPF_COEFFS[ALPF_STAGES * 5U] = {
    0.0089399244f, 0.0178798488f, 0.0089399244f,  1.8252977819f, -0.8610574795f,
    0.0081401750f, 0.0162803500f, 0.0081401750f,  1.6620099596f, -0.6945706597f
};

/*
 * AM/SSB AUDIO FILTER SET - 3 selectable widths, added 02/08/2026 (the
 * 2.3kHz one first, then extended to 3-way the same day per the
 * project owner - see AUDIO_BW_4K0's comment in demod_am.h). Same
 * design method throughout (4th-order Butterworth, 2 cascaded CMSIS
 * biquad DF1 stages, bilinear transform via scipy.signal.butter at
 * fs=192kHz, gain split evenly across both stages - matching
 * ALPF_COEFFS' own style rather than scipy's default of dumping the
 * whole gain into one section; cascaded stages give an identical
 * aggregate response either way, this is purely cosmetic/numerical-
 * precision consistency). Low-pass only (no high-pass/low-cut) - same
 * shape as ALPF_COEFFS, not a true bandpass SSB filter; a low-cut
 * around 300Hz could be added the same way later if wanted.
 *
 * ALPF_4K0_COEFFS happens to land on the same -3dB corner as
 * CHF_COEFFS above - purely coincidental (CHF_COEFFS runs on the
 * complex RF I/Q pre-demod, this runs on the real audio post-demod;
 * same corner, completely different stage/purpose), not a shared
 * design or a typo. Verified numerically:
 *   flat to 1.5kHz, -3.02dB at 4.0kHz (corner), -18.58dB at 6.8kHz,
 *   -33.48dB at 10.4kHz
 *
 * ALPF_2K3_COEFFS (originally ALPF_NARROW_COEFFS, renamed when the
 * 3-way selector replaced the plain WIDE/NARROW toggle):
 *   flat to 1kHz, -3.04dB at 2.3kHz (corner), -19.28dB at 4.0kHz,
 *   -33.41dB at 6.0kHz, -51.34dB at 10kHz
 *
 * ALPF_1K8_COEFFS - the narrowest option, verified numerically:
 *   flat to 1kHz, -3.01dB at 1.8kHz (corner), -18.52dB at 3.06kHz,
 *   -33.26dB at 4.68kHz, -59.89dB at 10kHz
 */
#define ALPF_4K0_STAGES 2U
static const float32_t ALPF_4K0_COEFFS[ALPF_4K0_STAGES * 5U] = {
    0.0039435671f, 0.0078871343f, 0.0039435671f,  1.7695043485f, -0.7847733318f,
    0.0039435671f, 0.0078871343f, 0.0039435671f,  1.8885559539f, -0.9048522288f
};

#define ALPF_2K3_STAGES 2U
static const float32_t ALPF_2K3_COEFFS[ALPF_2K3_STAGES * 5U] = {
    0.0013495925f, 0.0026991850f, 0.0013495925f,  1.8647864947f, -0.8700811583f,
    0.0013495925f, 0.0026991850f, 0.0013495925f,  1.9385529871f, -0.9440570949f
};

#define ALPF_1K8_STAGES 2U
static const float32_t ALPF_1K8_COEFFS[ALPF_1K8_STAGES * 5U] = {
    0.0008351767f, 0.0016703535f, 0.0008351767f,  1.8935423413f, -0.8968321878f,
    0.0008351767f, 0.0016703535f, 0.0008351767f,  1.9525426196f, -0.9559349733f
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
 */
#define WFM_DEEMPH_ALPHA 0.9010751057f

/*
 * WFM DISCRIMINATOR GAIN: scales atan2f()'s raw radians-per-sample
 * output before it enters the shared DC-blocker/AGC/output chain.
 * Picked so a full +/-75kHz deviation (phase step ~2.454 rad, see
 * demod_am.h) lands close to AGC_TARGET below - not that the exact
 * value matters much, since the AGC peak-normalizes downstream
 * regardless (step 4), this just keeps its gain excursion sane
 * instead of starting from a near-silent or heavily-clipped signal.
 */
#define WFM_DISC_GAIN 7300.0f

/*
 * NFM CHANNEL FILTER, 4th-order Butterworth (2 cascaded biquad DF1
 * stages, same shape/convention as CHF_COEFFS/WFM_ALPF_COEFFS), -3dB
 * at 6.25kHz @ fs=192kHz - applied to I and Q post-down-mix, same
 * spot CHF_COEFFS occupies for AM/SSB, just wider. Designed offline
 * via scipy.signal.butter (same method as the other filters here) and
 * verified numerically:
 *
 *   500Hz-2.5kHz: essentially flat (0 to -0.0dB)
 *   5kHz:         -0.67dB
 *   6.25kHz:      -3.01dB (the design corner)
 *   8kHz:         -9.21dB
 *   10kHz:        -16.62dB
 *   12.5kHz:      -24.47dB (adjacent 12.5kHz-spaced channel, well
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
    0.0000848574f, 0.0001697149f, 0.0000848574f,  1.6489012835f, -0.6840019530f,
    1.0000000000f, 2.0000000000f, 1.0000000000f,  1.8170786045f, -0.8557593164f
};

/*
 * NFM DISCRIMINATOR GAIN: same reasoning as WFM_DISC_GAIN, scaled for
 * NFM's much smaller deviation - worst case +/-5kHz (covers both the
 * 2.5kHz and 5kHz narrowband deviation conventions in use) gives a
 * phase step of only ~0.1636 rad @ 192kHz (vs WFM's ~2.454 rad), so
 * this needs a proportionally larger gain to land in the same
 * pre-AGC ballpark - ~110000 rather than WFM's ~7300. Again, the
 * exact value doesn't matter much since the AGC re-normalizes
 * downstream regardless.
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
 * SDR_RX_BLOCK_SAMPLES=512 samples/block @ 192kHz = ~2.667ms/block,
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
 * still at the full 192kHz rate), SSB diverges:
 *   1. DECIMATE I and Q by 16 (192kHz -> 12kHz, 512 samples/block ->
 *      32) via arm_fir_decimate_f32 with DECIM_COEFFS - a 161-tap
 *      anti-alias lowpass (-3dB ~5kHz, -42dB by 8kHz, -78dB by 10kHz;
 *      verified numerically, combined with the existing channel
 *      filter's own rolloff this comfortably covers everything that
 *      would otherwise fold into the 0-6kHz decimated Nyquist band).
 *      Despite 161 taps this is CHEAP: arm_fir_decimate_f32 only
 *      evaluates the FIR sum at the DECIMATED output rate (32
 *      samples), so cost is numTaps*32 = 5152 MACs per channel, not
 *      numTaps*512 - the whole point of doing it this way.
 *   2. Hilbert-shift the decimated Q and delay-match the decimated I
 *      (same technique as the old full-rate version, just at 12kHz
 *      now) - HILBERT_COEFFS is still 63 taps, but at 12kHz instead
 *      of 192kHz that covers 300Hz-6kHz far better (|H|=0.913 @
 *      300Hz, essentially flat 1.000 from 500Hz up - verified
 *      numerically; the old full-rate 63-tap version only reached
 *      0.164 @ 300Hz). Combine per demod_am.h's mechanics.
 *   3. INTERPOLATE the resulting 12kHz audio back up to 192kHz (32
 *      samples -> 512) via arm_fir_interpolate_f32 with
 *      INTERP_COEFFS, straight into s_env[] - everything downstream
 *      (DC blocker, audio LPF, AGC, output) is then IDENTICAL to AM,
 *      unchanged. INTERP_COEFFS is the SAME 5.5kHz lowpass design as
 *      DECIM_COEFFS but scaled to DC gain=16 instead of 1 - zero-
 *      stuffing L-1 zeros between samples divides energy by L, so the
 *      reconstruction filter must supply that L back or the audio
 *      comes out at 1/16 volume (confirmed with an explicit round-
 *      trip simulation before writing any firmware - see the design
 *      notes referenced above; would have been yet another "why is
 *      the volume so low" report otherwise). numTaps (160) must be a
 *      multiple of the interpolation factor (16) - arm_fir_interpolate_init_f32()
 *      enforces this and demod_am_init() checks its return status.
 */
#define DECIM_FACTOR 16U
#define DEC_BLOCK_SAMPLES (SDR_RX_BLOCK_SAMPLES / DECIM_FACTOR) /* 32, @ 12kHz */

#define DECIM_COEFFS_TAPS 161U
static const float32_t DECIM_COEFFS[DECIM_COEFFS_TAPS] = {
    -0.0000000000f, 0.0000005577f, 0.0000022598f, 0.0000049902f, 0.0000084158f, 0.0000119920f,
    0.0000149838f, 0.0000165018f, 0.0000155537f, 0.0000111088f, 0.0000021766f, -0.0000121062f,
    -0.0000323808f, -0.0000589708f, -0.0000917884f, -0.0001302509f, -0.0001732127f, -0.0002189192f,
    -0.0002649915f, -0.0003084446f, -0.0003457479f, -0.0003729285f, -0.0003857216f, -0.0003797646f,
    -0.0003508336f, -0.0002951149f, -0.0002095016f, -0.0000919033f, 0.0000584465f, 0.0002407026f,
    0.0004521586f, 0.0006880563f, 0.0009414709f, 0.0012032925f, 0.0014623173f, 0.0017054592f,
    0.0019180908f, 0.0020845117f, 0.0021885405f, 0.0022142165f, 0.0021465924f, 0.0019725910f,
    0.0016818946f, 0.0012678276f, 0.0007281925f, 0.0000660162f, -0.0007098389f, -0.0015842388f,
    -0.0025355542f, -0.0035356827f, -0.0045503370f, -0.0055396148f, -0.0064588500f, -0.0072597368f,
    -0.0078917018f, -0.0083034898f, -0.0084449146f, -0.0082687189f, -0.0077324750f, -0.0068004567f,
    -0.0054454043f, -0.0036501097f, -0.0014087478f, 0.0012721102f, 0.0043728593f, 0.0078606406f,
    0.0116895700f, 0.0158014247f, 0.0201267856f, 0.0245866089f, 0.0290941866f, 0.0335574384f,
    0.0378814610f, 0.0419712506f, 0.0457345059f, 0.0490844109f, 0.0519422995f, 0.0542401035f,
    0.0559224940f, 0.0569486370f, 0.0572934965f, 0.0569486370f, 0.0559224940f, 0.0542401035f,
    0.0519422995f, 0.0490844109f, 0.0457345059f, 0.0419712506f, 0.0378814610f, 0.0335574384f,
    0.0290941866f, 0.0245866089f, 0.0201267856f, 0.0158014247f, 0.0116895700f, 0.0078606406f,
    0.0043728593f, 0.0012721102f, -0.0014087478f, -0.0036501097f, -0.0054454043f, -0.0068004567f,
    -0.0077324750f, -0.0082687189f, -0.0084449146f, -0.0083034898f, -0.0078917018f, -0.0072597368f,
    -0.0064588500f, -0.0055396148f, -0.0045503370f, -0.0035356827f, -0.0025355542f, -0.0015842388f,
    -0.0007098389f, 0.0000660162f, 0.0007281925f, 0.0012678276f, 0.0016818946f, 0.0019725910f,
    0.0021465924f, 0.0022142165f, 0.0021885405f, 0.0020845117f, 0.0019180908f, 0.0017054592f,
    0.0014623173f, 0.0012032925f, 0.0009414709f, 0.0006880563f, 0.0004521586f, 0.0002407026f,
    0.0000584465f, -0.0000919033f, -0.0002095016f, -0.0002951149f, -0.0003508336f, -0.0003797646f,
    -0.0003857216f, -0.0003729285f, -0.0003457479f, -0.0003084446f, -0.0002649915f, -0.0002189192f,
    -0.0001732127f, -0.0001302509f, -0.0000917884f, -0.0000589708f, -0.0000323808f, -0.0000121062f,
    0.0000021766f, 0.0000111088f, 0.0000155537f, 0.0000165018f, 0.0000149838f, 0.0000119920f,
    0.0000084158f, 0.0000049902f, 0.0000022598f, 0.0000005577f, -0.0000000000f
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

#define INTERP_COEFFS_TAPS 160U /* must be a multiple of DECIM_FACTOR */
static const float32_t INTERP_COEFFS[INTERP_COEFFS_TAPS] = {
    -0.0000000000f, 0.0000091231f, 0.0000363733f, 0.0000789648f, 0.0001306281f, 0.0001817951f,
    0.0002200229f, 0.0002306613f, 0.0001977535f, 0.0001051537f, -0.0000621718f, -0.0003167055f,
    -0.0006668611f, -0.0011154649f, -0.0016583222f, -0.0022829698f, -0.0029677202f, -0.0036811060f,
    -0.0043818288f, -0.0050193049f, -0.0055348824f, -0.0058637752f, -0.0059377274f, -0.0056883796f,
    -0.0050512624f, -0.0039702950f, -0.0024026173f, -0.0003235368f, 0.0022686662f, 0.0053483659f,
    0.0088589592f, 0.0127102869f, 0.0167774100f, 0.0209010092f, 0.0248896291f, 0.0285239189f,
    0.0315629380f, 0.0337524981f, 0.0348354045f, 0.0345633479f, 0.0327100854f, 0.0290854458f,
    0.0235495988f, 0.0160269561f, 0.0065190223f, -0.0048845094f, -0.0179971074f, -0.0325298449f,
    -0.0480895005f, -0.0641807674f, -0.0802128752f, -0.0955107595f, -0.1093307218f, -0.1208803218f,
    -0.1293420387f, -0.1339000419f, -0.1337692291f, -0.1282255368f, -0.1166364072f, -0.0984902202f,
    -0.0734234673f, -0.0412444693f, -0.0019525150f, 0.0442485740f, 0.0969432701f, 0.1555041548f,
    0.2190990673f, 0.2867056483f, 0.3571330536f, 0.4290503169f, 0.5010205611f, 0.5715400049f,
    0.6390804907f, 0.7021340902f, 0.7592582305f, 0.8091197270f, 0.8505361256f, 0.8825128295f,
    0.9042746318f, 0.9152904702f, 0.9152904702f, 0.9042746318f, 0.8825128295f, 0.8505361256f,
    0.8091197270f, 0.7592582305f, 0.7021340902f, 0.6390804907f, 0.5715400049f, 0.5010205611f,
    0.4290503169f, 0.3571330536f, 0.2867056483f, 0.2190990673f, 0.1555041548f, 0.0969432701f,
    0.0442485740f, -0.0019525150f, -0.0412444693f, -0.0734234673f, -0.0984902202f, -0.1166364072f,
    -0.1282255368f, -0.1337692291f, -0.1339000419f, -0.1293420387f, -0.1208803218f, -0.1093307218f,
    -0.0955107595f, -0.0802128752f, -0.0641807674f, -0.0480895005f, -0.0325298449f, -0.0179971074f,
    -0.0048845094f, 0.0065190223f, 0.0160269561f, 0.0235495988f, 0.0290854458f, 0.0327100854f,
    0.0345633479f, 0.0348354045f, 0.0337524981f, 0.0315629380f, 0.0285239189f, 0.0248896291f,
    0.0209010092f, 0.0167774100f, 0.0127102869f, 0.0088589592f, 0.0053483659f, 0.0022686662f,
    -0.0003235368f, -0.0024026173f, -0.0039702950f, -0.0050512624f, -0.0056883796f, -0.0059377274f,
    -0.0058637752f, -0.0055348824f, -0.0050193049f, -0.0043818288f, -0.0036811060f, -0.0029677202f,
    -0.0022829698f, -0.0016583222f, -0.0011154649f, -0.0006668611f, -0.0003167055f, -0.0000621718f,
    0.0001051537f, 0.0001977535f, 0.0002306613f, 0.0002200229f, 0.0001817951f, 0.0001306281f,
    0.0000789648f, 0.0000363733f, 0.0000091231f, -0.0000000000f
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

/* DC blocker pole: y[n] = x[n] - x[n-1] + R*y[n-1]. R=0.9995 at
 * 192kHz puts the corner around 15Hz. Left as a scalar one-pole -
 * cheap enough (1 MAC/sample) that a CMSIS block call isn't worth
 * the extra buffer/call overhead. */
#define DCB_R 0.9995f

/* AGC: target output amplitude (of int16 full scale 32767), peak
 * release per sample, and gain bounds. GAIN_MAX bounds how far pure
 * band noise gets amplified. Attack is instant (unconditional in the
 * loop below) in every profile - only release varies. */
#define AGC_TARGET   18000.0f
#define AGC_PEAK_MIN 40.0f
#define AGC_GAIN_MAX 300.0f

/*
 * Per-profile release coefficients (peak *= this, per sample @
 * 192kHz) - see agc_profile_t's comment in demod_am.h for what each
 * one is for. Computed offline as exp(-1/(fs*tau)) for the target
 * time constant tau, same method as WFM_DEEMPH_ALPHA above:
 *
 *   AGC_RELEASE_SLOW   : tau=700ms -> 0.99999256f
 *   AGC_RELEASE_MEDIUM : tau=175ms -> 0.99997024f (this is, to 5
 *                         decimal places, the ORIGINAL single
 *                         AGC_RELEASE value this project always used
 *                         before profiles existed - MEDIUM is the
 *                         default specifically so existing behavior
 *                         doesn't change for anyone who never touches
 *                         the new badge)
 *   AGC_RELEASE_FAST   : tau=60ms  -> 0.99991320f
 *
 * AGC_PROFILE_MANUAL doesn't use any of these - see the loop in
 * demod_am_process_raw() and agc_profile_t's MANUAL note.
 */
#define AGC_RELEASE_SLOW   0.99999256f
#define AGC_RELEASE_MEDIUM 0.99997024f
#define AGC_RELEASE_FAST   0.99991320f

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

/* SSB decimated-chain instances + state (see the PIPELINE comment
 * above DECIM_COEFFS). CMSIS state sizes:
 *   decimate:    numTaps + blockSize(INPUT, 512) - 1
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
static uint8_t s_mode = (uint8_t)DEMOD_MODE_WFM;

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

void demod_am_set_agc_profile(agc_profile_t profile)
{
    s_agc_profile = profile;
    switch (profile) {
    case AGC_PROFILE_SLOW:   s_agc_release = AGC_RELEASE_SLOW;   break;
    case AGC_PROFILE_FAST:   s_agc_release = AGC_RELEASE_FAST;   break;
    case AGC_PROFILE_MANUAL: break; /* unused in MANUAL - see the AGC loop */
    case AGC_PROFILE_MEDIUM:
    default:                  s_agc_release = AGC_RELEASE_MEDIUM; break;
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

/* Per-stage breakdown (see demod_am.h's comment on
 * demod_am_get_last_cycles_breakdown()). Plain volatile uint32_t
 * words, same "not worth a critical section" reasoning as s_mode -
 * worst case the main loop reads one stage from the current block and
 * another from the previous one, which for a diagnostic readout is
 * irrelevant. */
static volatile uint32_t s_last_cycles_frontend = 0U;
static volatile uint32_t s_last_cycles_extract  = 0U;
static volatile uint32_t s_last_cycles_audio    = 0U;
static volatile uint32_t s_last_cycles_agc_out  = 0U;

demod_am_cycles_breakdown_t demod_am_get_last_cycles_breakdown(void)
{
    demod_am_cycles_breakdown_t b;
    b.frontend = s_last_cycles_frontend;
    b.extract  = s_last_cycles_extract;
    b.audio    = s_last_cycles_audio;
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
                             float32_t *env, float32_t gain)
{
    float32_t i_prev = s_fm_i_prev;
    float32_t q_prev = s_fm_q_prev;
    uint32_t n;

    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
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

void demod_am_process_raw(const int16_t *raw_interleaved)
{
    uint32_t n;
    float dcb_x1 = s_dcb_x1;
    float dcb_y1 = s_dcb_y1;
    float peak = s_agc_peak;
    uint32_t cyc_start = DWT->CYCCNT;

    /* 0a. Deinterleave raw int16 L/R (I/Q) into separate float rails.
     * Plain cast, same scale as before (no -1..1 normalization) so
     * the AGC's int16-scale targeting downstream doesn't change. */
    for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
        s_i_buf[n] = (float32_t)raw_interleaved[2U * n];
        s_q_buf[n] = (float32_t)raw_interleaved[2U * n + 1U];
    }

    /* 0b/0c: LOW-IF DOWN-MIX + CHANNEL FILTER - SKIPPED ENTIRELY for
     * WFM (see demod_am.h's WFM note: a ~200kHz broadcast FM channel
     * needs the full +/-96kHz of complex bandwidth this narrowband
     * front-end would otherwise throw away, and both the down-mix's
     * 48kHz shift and CHF_COEFFS' ~4kHz channel filter would make it
     * unrecoverable). AM/USB/LSB are unchanged. NFM stays in this
     * branch too (it needs the down-mix just as much as AM/SSB do) -
     * it just gets its OWN, wider channel filter instead of
     * CHF_COEFFS, selected inside 0c below - see demod_am.h's NFM
     * note. */
    if (s_mode != (uint8_t)DEMOD_MODE_WFM) {
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
    } else if (s_mode == (uint8_t)DEMOD_MODE_WFM) {
        /* WFM: delay-and-conjugate-multiply discriminator at the full
         * 192kHz rate, straight on the RAW (unfiltered, un-down-mixed)
         * I/Q - see demod_am.h's WFM note for the full derivation and
         * why atan2f() rather than arm_atan2_f32() here. */
        fm_discriminate(s_i_buf, s_q_buf, s_env, WFM_DISC_GAIN);
    } else if (s_mode == (uint8_t)DEMOD_MODE_NFM) {
        /* NFM: same discriminator as WFM (see fm_discriminate()'s
         * comment), but on the down-mixed + NFM_CHF_COEFFS-filtered
         * I/Q from steps 0b/0c above, not the raw signal - see
         * demod_am.h's NFM note on why NFM keeps that narrowband
         * front-end instead of skipping it like WFM does. */
        fm_discriminate(s_i_buf, s_q_buf, s_env, NFM_DISC_GAIN);
    } else {
        /* SSB (USB/LSB), phasing method at a DECIMATED rate - see the
         * PIPELINE comment above DECIM_COEFFS and demod_am.h's SSB
         * note. */
        float32_t sign = (s_mode == (uint8_t)DEMOD_MODE_USB) ? SSB_USB_SIGN : SSB_LSB_SIGN;
        uint32_t k;

        /* 1. Decimate channel-filtered I and Q by 16: 512 samples @
         * 192kHz -> 32 samples @ 12kHz, anti-aliased by DECIM_COEFFS
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

        /* 3. Interpolate the 12kHz SSB audio back up to 192kHz (32
         * samples -> 512), straight into s_env[] - everything
         * downstream (DC blocker, ALPF, AGC, output) is shared with
         * AM, unchanged. INTERP_COEFFS carries the x16 gain that
         * compensates the zero-stuffing loss (see the PIPELINE
         * comment - without it this comes out at 1/16 volume). */
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
        s_last_cycles_extract = cyc_now - cyc_start - s_last_cycles_frontend;
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

    /* 3. Audio LPF - WFM branches off here (de-emphasis + its own
     * wider LPF), AM/SSB keep the original ~6kHz path unchanged. */
    if (s_mode == (uint8_t)DEMOD_MODE_WFM) {
        /* 3a. De-emphasis, single-pole LPF (see WFM_DEEMPH_ALPHA's
         * comment) - scalar, same reasoning as the DC blocker above
         * for not using a CMSIS block call. */
        float y1 = s_wfm_deemph_y1;
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            y1 = WFM_DEEMPH_ALPHA * y1 + (1.0f - WFM_DEEMPH_ALPHA) * s_env[n];
            s_env[n] = y1;
        }
        s_wfm_deemph_y1 = y1;

        /* 3b. WFM audio LPF, 4th-order Butterworth ~15kHz (see
         * WFM_ALPF_COEFFS' comment) - same in-place CMSIS call
         * pattern as the channel filter/AM-SSB ALPF above. */
        arm_biquad_cascade_df1_f32(&s_wfm_alpf_inst, s_env, s_env, SDR_RX_BLOCK_SAMPLES);
    } else {
        /* 3. Audio LPF, 4th-order Butterworth via CMSIS-DSP (in-place,
         * same reasoning as the channel filter above).
         *
         * NFM: unconditionally s_alpf_inst (ALPF_COEFFS, ~6kHz) -
         * unaffected by the AM/SSB selector below even though it lands
         * in this same else branch (see this function's step 3 comment
         * above). NFM's own channel filter (NFM_CHF_COEFFS) already
         * sets its bandwidth upstream, and letting the AM/SSB selector
         * also touch NFM's audio would narrow narrowband FM voice
         * every time someone picks a tighter AM/SSB filter, which the
         * project owner never asked for - see AUDIO_BW_4K0's comment
         * in demod_am.h for why these are kept as fully separate
         * instances instead of one shared "current audio filter".
         *
         * AM/USB/LSB: whichever of the three s_audio_bw picked - see
         * demod_am_set_audio_bw()'s comment in demod_am.h. */
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

    gd32_i2s_stream_write_half(s_audio_out);

    s_last_cycles = DWT->CYCCNT - cyc_start;
    s_last_cycles_agc_out = s_last_cycles - s_last_cycles_frontend
                             - s_last_cycles_extract - s_last_cycles_audio;
}
