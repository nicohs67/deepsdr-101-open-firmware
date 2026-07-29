#include "fft.h"

/*
 * RAM BUDGET: re/im (512*4*2=4096B) + cos/sin twiddles (256*4*2=2048B)
 * + Hann window (512*4=2048B) + bit-reversal table (512*2=1024B) =
 * ~9.2KB static. Combined with the waterfall's own buffer and sdr_rx's
 * raw buffer, this still leaves margin within the main SRAM budget.
 */

static float s_re[FFT_SIZE];
static float s_im[FFT_SIZE];
static float s_twiddle_cos[FFT_SIZE / 2U];
static float s_twiddle_sin[FFT_SIZE / 2U];
static float s_hann[FFT_SIZE];
static uint16_t s_bitrev[FFT_SIZE];

/* Sine approximation with no libm dependency - same method (Bhaskara
 * I) as gd32_i2s.c, reimplemented locally to keep the two modules
 * decoupled. */
static float sinf_approx_local(float x)
{
    float pi = 3.14159265358979f;
    float x2;
    int negate = 0;
    while (x > pi) { x -= 2.0f * pi; }
    while (x < -pi) { x += 2.0f * pi; }
    if (x < 0.0f) { x = -x; negate = 1; }
    x2 = (16.0f * x * (pi - x)) / (5.0f * pi * pi - 4.0f * x * (pi - x));
    return negate ? -x2 : x2;
}

static float cosf_approx_local(float x)
{
    return sinf_approx_local(x + (3.14159265358979f / 2.0f));
}

/* Approximate log2(x) via IEEE754 bit reinterpretation (the classic
 * "fast log2" trick): not exact (typically a few percent error), but
 * more than good enough for scaling color in a waterfall display, and
 * avoids libm entirely. */
static float log2_approx(float x)
{
    union { float f; uint32_t i; } vx;
    float y;
    if (x <= 0.0f) {
        x = 1.0e-9f; /* avoid log(0)/negative input - arbitrary low floor */
    }
    vx.f = x;
    y = (float)vx.i;
    y *= 1.1920929e-7f; /* 1 / 2^23 */
    return y - 126.94269504f;
}

void fft_init(void)
{
    uint32_t n;
    uint32_t bits = 0U;
    uint32_t size = FFT_SIZE;

    /* Exact log2(FFT_SIZE) by counting shifts (FFT_SIZE is guaranteed
     * to be a power of two per the header's contract) */
    while (size > 1U) { size >>= 1U; bits++; }

    for (n = 0; n < FFT_SIZE / 2U; n++) {
        float angle = -2.0f * 3.14159265358979f * (float)n / (float)FFT_SIZE;
        s_twiddle_cos[n] = cosf_approx_local(angle);
        s_twiddle_sin[n] = sinf_approx_local(angle);
    }

    for (n = 0; n < FFT_SIZE; n++) {
        /* Hann window: 0.5 - 0.5*cos(2*pi*n/(N-1)) */
        s_hann[n] = 0.5f - 0.5f * cosf_approx_local(
            2.0f * 3.14159265358979f * (float)n / (float)(FFT_SIZE - 1U));
    }

    /* Bit-reversal table for `bits` bits, precomputed once */
    for (n = 0; n < FFT_SIZE; n++) {
        uint32_t v = n;
        uint32_t r = 0U;
        uint32_t b;
        for (b = 0; b < bits; b++) {
            r = (r << 1U) | (v & 1U);
            v >>= 1U;
        }
        s_bitrev[n] = (uint16_t)r;
    }
}

void fft_compute_db(const int16_t *samples, float *db_out)
{
    uint32_t n, stage, i, j;
    uint32_t half_size, step;

    /* Window + load, with bit-reversal reordering applied on load */
    for (n = 0; n < FFT_SIZE; n++) {
        uint16_t src = s_bitrev[n];
        s_re[n] = (float)samples[src] * s_hann[src];
        s_im[n] = 0.0f;
    }

    /* Iterative radix-2 DIT, in-place */
    for (half_size = 1U, step = FFT_SIZE / 2U; half_size < FFT_SIZE;
         half_size <<= 1U, step >>= 1U) {
        for (i = 0; i < FFT_SIZE; i += (half_size << 1U)) {
            for (j = 0; j < half_size; j++) {
                uint32_t tw_idx = j * step;
                float tre = s_twiddle_cos[tw_idx];
                float tim = s_twiddle_sin[tw_idx];
                uint32_t a = i + j;
                uint32_t b = a + half_size;
                float br = s_re[b] * tre - s_im[b] * tim;
                float bi = s_re[b] * tim + s_im[b] * tre;
                s_re[b] = s_re[a] - br;
                s_im[b] = s_im[a] - bi;
                s_re[a] = s_re[a] + br;
                s_im[a] = s_im[a] + bi;
            }
        }
    }

    /* Power -> "dB" (10*log10(x) = 3.0103*log2(x)), only the useful
     * bins 0..N/2-1 (the rest is the mirror image of a real input) */
    for (n = 0; n < FFT_BINS_USEFUL; n++) {
        float power = s_re[n] * s_re[n] + s_im[n] * s_im[n];
        db_out[n] = 3.0103f * log2_approx(power);
    }
}
