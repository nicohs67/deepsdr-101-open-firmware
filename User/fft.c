#include "fft.h"

/*
 * PRESUPUESTO DE RAM (ver criterio ya establecido en waterfall.h):
 * re/im (512*4*2=4096B) + twiddles cos/sin (256*4*2=2048B) + hann
 * (512*4=2048B) + tabla de bit-reversal (512*2=1024B) = ~9.2KB
 * estaticos. Sumado a los 96000B del waterfall y los 4096B del buffer
 * RAW de sdr_rx, deja margen dentro de los 192KB de RAM principal.
 */

static float s_re[FFT_SIZE];
static float s_im[FFT_SIZE];
static float s_twiddle_cos[FFT_SIZE / 2U];
static float s_twiddle_sin[FFT_SIZE / 2U];
static float s_hann[FFT_SIZE];
static uint16_t s_bitrev[FFT_SIZE];

/* Aproximacion de seno sin libm - mismo metodo (Bhaskara I) que en
 * gd32_i2s.c, reimplementada aqui para no acoplar los dos modulos. */
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

/* log2(x) aproximado via reinterpretacion de bits IEEE754 (truco
 * clasico "fast log2"): NO exacto (error tipico unos pocos %), pero de
 * sobra para escalar color en un waterfall, y evita libm por completo. */
static float log2_approx(float x)
{
    union { float f; uint32_t i; } vx;
    float y;
    if (x <= 0.0f) {
        x = 1.0e-9f; /* evitar log(0)/negativos - suelo arbitrario bajo */
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

    /* log2(FFT_SIZE) exacto por conteo de shifts (FFT_SIZE es potencia
     * de 2 por contrato de la cabecera) */
    while (size > 1U) { size >>= 1U; bits++; }

    for (n = 0; n < FFT_SIZE / 2U; n++) {
        float angle = -2.0f * 3.14159265358979f * (float)n / (float)FFT_SIZE;
        s_twiddle_cos[n] = cosf_approx_local(angle);
        s_twiddle_sin[n] = sinf_approx_local(angle);
    }

    for (n = 0; n < FFT_SIZE; n++) {
        /* ventana de Hann: 0.5 - 0.5*cos(2*pi*n/(N-1)) */
        s_hann[n] = 0.5f - 0.5f * cosf_approx_local(
            2.0f * 3.14159265358979f * (float)n / (float)(FFT_SIZE - 1U));
    }

    /* tabla de bit-reversal de `bits` bits, precalculada una vez */
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

    /* ventana + carga (con reordenacion bit-reversal ya aplicada) */
    for (n = 0; n < FFT_SIZE; n++) {
        uint16_t src = s_bitrev[n];
        s_re[n] = (float)samples[src] * s_hann[src];
        s_im[n] = 0.0f;
    }

    /* radix-2 DIT iterativo, in-place */
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

    /* potencia -> "dB" (10*log10(x) = 3.0103*log2(x)), solo bins utiles
     * 0..N/2-1 (el resto es simetria especular de la entrada real) */
    for (n = 0; n < FFT_BINS_USEFUL; n++) {
        float power = s_re[n] * s_re[n] + s_im[n] * s_im[n];
        db_out[n] = 3.0103f * log2_approx(power);
    }
}
