#ifndef FFT_H
#define FFT_H

#include <stdint.h>

/*
 * Real-input FFT for the spectrum/waterfall display, self-contained
 * and with NO libm dependency (this project links with nano.specs and
 * no -lm - same constraint that shaped the I2S test tone code).
 * Classic radix-2 Cooley-Tukey, Hann window, power converted to "dB"
 * via a fast bit-manipulation log2 approximation (not exact, but more
 * than good enough for coloring a waterfall/spectrum display - not
 * intended as a calibrated measurement).
 */

#define FFT_SIZE        512U   /* must be a power of two */
#define FFT_BINS_USEFUL (FFT_SIZE / 2U)  /* bins 0..Nyquist; the rest mirror these */

/* Must be called once at startup (precomputes the twiddle table). */
void fft_init(void);

/*
 * Input: `samples` (FFT_SIZE real samples, e.g. int16_t already
 * de-interleaved from sdr_rx). Output: `db_out` (FFT_BINS_USEFUL
 * values, uncalibrated relative "dB" - higher means more energy).
 * Applies a Hann window internally before transforming.
 */
void fft_compute_db(const int16_t *samples, float *db_out);

#endif /* FFT_H */
