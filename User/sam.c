#include "sam.h"
#include <math.h>

#define SAM_TPI (2.0f * 3.14159265358979323846f)

void sam_init(sam_t *s, float32_t sample_rate_hz, float32_t pll_fmax_hz,
        float32_t omega_n_hz, float32_t zeta)
{
    s->pll_fmax = pll_fmax_hz;
    s->zeta = zeta;
    s->omegaN = omega_n_hz;

    /* Matches sam_variables_init()'s own omega_min/omega_max/g1/g2
     * formulas exactly, with DR=1 (this project calls sam_step()
     * directly on s_i_buf/s_q_buf, one call per real sample - no
     * extra internal decimation the way the original multi-mode SDR
     * project's SAM() had, hence no separate DR term needed here). */
    s->omega_min = -SAM_TPI * pll_fmax_hz / sample_rate_hz;
    s->omega_max =  SAM_TPI * pll_fmax_hz / sample_rate_hz;

    s->g1 = 1.0f - expf(-2.0f * omega_n_hz * zeta / sample_rate_hz);
    s->g2 = -s->g1 + 2.0f * (1.0f - expf(-omega_n_hz * zeta / sample_rate_hz) *
                                     cosf(omega_n_hz / sample_rate_hz *
                                          sqrtf(1.0f - zeta * zeta)));

    /* fade leveler - tauR/tauI values match sam_variables_init()'s own
     * comments ("original 0.02"/"original 1.4"). */
    s->tauR = 0.02f;
    s->tauI = 1.4f;
    s->mtauR = expf(-1.0f / (sample_rate_hz * s->tauR));
    s->onem_mtauR = 1.0f - s->mtauR;
    s->mtauI = expf(-1.0f / (sample_rate_hz * s->tauI));
    s->onem_mtauI = 1.0f - s->mtauI;
    s->dc = 0.0f;
    s->dc_insert = 0.0f;
    s->fade_leveler = 1u;

    sam_reset(s);
}

void sam_reset(sam_t *s)
{
    s->phzerror = 0.0f;
    s->det = 0.0f;
    s->fil_out = 0.0f;
    s->del_out = 0.0f;
    s->omega2 = 0.0f;
    s->corr0 = 0.0f;
    s->corr1 = 0.0f;
    s->audio = 0.0f;
    s->carrier_hz_raw = 0.0f;
    s->carrier_hz = 0.0f;
}

float32_t sam_step(sam_t *s, float32_t i_in, float32_t q_in, float32_t sample_rate_hz)
{
    float32_t sin_v = sinf(s->phzerror);
    float32_t cos_v = cosf(s->phzerror);

    /* Same quadrature mixing + phase-detector combination as the
     * original SAM()'s ai/bi/aq/bq/corr[0]/corr[1]/det - Hilbert-
     * derived ai_ps/bi_ps/aq_ps/bq_ps terms omitted (SAML/SAMU only,
     * not this mode - see sam.h's own top comment). */
    float32_t ai = cos_v * i_in;
    float32_t bi = sin_v * i_in;
    float32_t aq = cos_v * q_in;
    float32_t bq = sin_v * q_in;

    s->corr0 = ai + bq;
    s->corr1 = -bi + aq;

    /* DEMOD_SAM case only (both sidebands) - matches the original
     * switch's `case DEMOD_SAM: audio = corr[0];` exactly. */
    s->audio = s->corr0;

    if (s->fade_leveler) {
        s->dc = s->mtauR * s->dc + s->onem_mtauR * s->audio;
        s->dc_insert = s->mtauI * s->dc_insert + s->onem_mtauI * s->corr0;
        s->audio = s->audio + s->dc_insert - s->dc;
    }

    s->det = atan2f(s->corr1, s->corr0);

    /* Loop filter + phase accumulator - identical structure to the
     * original's del_out/omega2/fil_out/phzerror update. */
    s->del_out = s->fil_out;
    s->omega2 = s->omega2 + s->g2 * s->det;
    if (s->omega2 < s->omega_min) {
        s->omega2 = s->omega_min;
    } else if (s->omega2 > s->omega_max) {
        s->omega2 = s->omega_max;
    }
    s->fil_out = s->g1 * s->det + s->omega2;
    s->phzerror = s->phzerror + s->del_out;

    while (s->phzerror >= SAM_TPI) { s->phzerror -= SAM_TPI; }
    while (s->phzerror < 0.0f) { s->phzerror += SAM_TPI; }

    /* Hz conversion - corrected version of the original's own
     * commented-out calculation (see sam.h's own top comment for the
     * exact typo). omega2 is in rad/sample; multiplying by
     * sample_rate_hz (samples/sec) gives rad/sec, dividing by 2*pi
     * (rad/cycle) gives Hz. DF (an extra decimation factor in the
     * original multi-mode project) is 1 here for the same "no
     * internal decimation" reason noted in sam.h, so it's simply
     * omitted rather than carried as an always-1 multiply. The
     * 0.08/0.92 IIR smoothing weights are copied unchanged from the
     * original - purely a display/reading smoother, doesn't affect
     * the PLL's own lock dynamics above. */
    s->carrier_hz_raw = (s->omega2 * sample_rate_hz) / SAM_TPI;
    s->carrier_hz = 0.08f * s->carrier_hz_raw + 0.92f * s->carrier_hz;

    return s->audio;
}
