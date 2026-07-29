#include "spectrum.h"
#include "gfx.h"

uint16_t spectrum_colormap(float db, float db_min, float db_max)
{
    float t;
    uint8_t r, g, b;

    if (db_max <= db_min) {
        return GFX_COLOR_BLACK; /* invalid range, avoid divide-by-zero */
    }
    t = (db - db_min) / (db_max - db_min);
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }

    /* 4 segments: black/blue -> cyan -> green/yellow -> red, each
     * covering 1/4 of the scale. Simple linear interpolation within
     * each segment. */
    if (t < 0.25f) {
        float u = t / 0.25f;
        r = 0U;
        g = 0U;
        b = (uint8_t)(u * 255.0f);
    } else if (t < 0.5f) {
        float u = (t - 0.25f) / 0.25f;
        r = 0U;
        g = (uint8_t)(u * 255.0f);
        b = 255U;
    } else if (t < 0.75f) {
        float u = (t - 0.5f) / 0.25f;
        r = (uint8_t)(u * 255.0f);
        g = 255U;
        b = (uint8_t)((1.0f - u) * 255.0f);
    } else {
        float u = (t - 0.75f) / 0.25f;
        r = 255U;
        g = (uint8_t)((1.0f - u) * 255.0f);
        b = 0U;
    }
    return gfx_rgb565(r, g, b);
}

/*
 * Row-stripe rendering: build one full-width row buffer per pixel row
 * of the rectangle and flush it with a single gfx_blit() call, instead
 * of calling gfx_vline() once per column. Each gfx_vline() call reopens
 * a full EXMC window (CASET+RASET+RAMWR), which is expensive - this
 * cuts the number of window opens from ~w (up to 800) down to ~h
 * (typically under 150), a significant reduction in EXMC transactions.
 */
static uint16_t s_bar_h[800];
static uint16_t s_row_buf[800];

void spectrum_draw(const float *db, uint32_t n_bins,
                    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    float db_min, float db_max)
{
    uint16_t col, row;

    if (db_max <= db_min || w > 800U) {
        return;
    }

    /* Pass 1: bar height per column, no EXMC access yet */
    for (col = 0; col < w; col++) {
        uint32_t bin = ((uint32_t)col * n_bins) / w;
        float v = db[bin];
        float t = (v - db_min) / (db_max - db_min);
        if (t < 0.0f) { t = 0.0f; }
        if (t > 1.0f) { t = 1.0f; }
        s_bar_h[col] = (uint16_t)(t * (float)h);
    }

    /* Pass 2: one full-width row stripe per row, one gfx_blit per row */
    for (row = 0; row < h; row++) {
        uint16_t level_from_bottom = (uint16_t)(h - row);
        for (col = 0; col < w; col++) {
            if (s_bar_h[col] >= level_from_bottom) {
                uint32_t bin = ((uint32_t)col * n_bins) / w;
                s_row_buf[col] = spectrum_colormap(db[bin], db_min, db_max);
            } else {
                s_row_buf[col] = GFX_COLOR_BLACK;
            }
        }
        gfx_blit(x, (uint16_t)(y + row), w, 1, s_row_buf);
    }
}
