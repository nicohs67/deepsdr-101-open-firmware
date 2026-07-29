#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdint.h>

/*
 * Draws the instantaneous spectrum (live trace, not history - history
 * is what waterfall.c is for) inside a fixed screen rectangle, and
 * exposes the same dB->RGB565 colormap so gd32_i2s.c/main.c can reuse
 * it when coloring the row pushed to waterfall_push_line() - one
 * shared colormap for both, so the scale reads consistently.
 */

/* dB -> RGB565, fixed scale db_min..db_max (clamped outside that
 * range). Classic SDR waterfall palette: blue (cold/quiet) -> cyan ->
 * green -> yellow -> red (hot/strong signal). */
uint16_t spectrum_colormap(float db, float db_min, float db_max);

/*
 * Draws `n_bins` dB values as vertical bars filling the rectangle
 * (x,y,w,h). If n_bins != w, resamples (nearest neighbor) to stretch/
 * shrink to the actual screen width. db_min/max set the vertical
 * scale (and are also used for color, via spectrum_colormap).
 */
void spectrum_draw(const float *db, uint32_t n_bins,
                    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    float db_min, float db_max);

#endif /* SPECTRUM_H */
