#ifndef FFT_H
#define FFT_H

#include <stdint.h>

/*
 * FFT real->espectro, autocontenida y SIN libm (el proyecto enlaza con
 * nano.specs sin -lm, ver Makefile - confirmado al escribir el tono de
 * prueba de I2S). Radix-2 Cooley-Tukey clasica, ventana de Hann,
 * potencia en "dB" via una aproximacion de log2 de bit-manipulation
 * (no exacta, pero de sobra para colorear un waterfall/espectro - no
 * pretende ser una medida de precision).
 */

#define FFT_SIZE        512U   /* debe ser potencia de 2 */
#define FFT_BINS_USEFUL (FFT_SIZE / 2U)  /* bins 0..Nyquist, el resto son espejo */

/* Debe llamarse una vez al arrancar (precalcula la tabla de twiddles). */
void fft_init(void);

/*
 * Entrada: `samples` (FFT_SIZE muestras reales, ej. int16_t ya
 * deintercalado de sdr_rx). Salida: `db_out` (FFT_BINS_USEFUL valores,
 * "dB" relativos sin calibrar, mayor = mas energia). Aplica ventana de
 * Hann internamente antes de transformar.
 */
void fft_compute_db(const int16_t *samples, float *db_out);

#endif /* FFT_H */
