#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdint.h>

/*
 * Spectrum trace rendering + shared dB->RGB565 colormap (the same
 * palette colors both the live spectrum and the waterfall rows, so
 * the scale reads consistently between them).
 *
 * PERFORMANCE MODEL (this is the hot path of the whole UI): the
 * original implementation called a float colormap function - with
 * divisions and branches - once per LIT PIXEL, up to ~80k calls per
 * frame, which dominated the frame time by an order of magnitude
 * over the actual EXMC pixel writes. This version moves ALL float
 * math out of the per-pixel loop:
 *
 *   - The palette is a 256-entry RGB565 lookup table built once by
 *     spectrum_init().
 *   - Per frame, dB values are folded to one value per screen COLUMN
 *     (averaging every FFT bin that lands in the column - cheap
 *     anti-aliasing, kills the nearest-neighbor shimmer), smoothed,
 *     and quantized to integer bar heights.
 *   - Per frame, one color per ROW is precomputed for the vertical
 *     gradient fill.
 *   - The per-pixel inner loop is then just an integer compare and a
 *     16-bit store into a row stripe that gfx_blit()s once per row.
 *
 * VISUALS:
 *   - Vertical gradient fill (each bar colored by height through the
 *     palette, classic "heat" look).
 *   - Asymmetric exponential smoothing per column: fast attack, slow
 *     decay - signals pop instantly, the floor calms down.
 *   - Optional decaying peak-hold markers.
 *   - Horizontal gridline every SPECTRUM_GRID_DB dB, drawn dim under
 *     the trace.
 *   - 1px bright trace on top of each bar.
 */

/* Grid spacing in dB (set 0 to disable gridlines). */
#define SPECTRUM_GRID_DB 20.0f

/* Smoothing factors (0..1): applied per displayed frame, not per FFT
 * block. Attack is how fast the trace RISES toward a stronger value,
 * decay how fast it FALLS. */
#define SPECTRUM_EMA_ATTACK 0.60f
#define SPECTRUM_EMA_DECAY  0.25f

/* Peak-hold: 0 disables. Decay is in dB per displayed frame. */
#define SPECTRUM_PEAK_HOLD  0
#define SPECTRUM_PEAK_DECAY 0.4f

/* Vertical marker at the horizontal center (the VFO, when fed the
 * fftshifted I/Q spectrum). Drawn UNDER signals, over the grid, so it
 * never obscures a real trace. 0 disables.
 *
 * When low-IF tuning is active (see demod_am.h's LOW-IF TUNING note),
 * the demodulated signal doesn't sit on the true LO/center bin
 * anymore - it's DEMOD_IF_OFFSET_HZ away from it. spectrum_draw()'s
 * center_mark_offset_px parameter shifts this line to track the
 * actual demod point instead of always drawing it dead-center. */
#define SPECTRUM_CENTER_MARK 1

/* Build the palette LUT. Call once at startup, before the first
 * spectrum_draw()/spectrum_colormap(). */
void spectrum_init(void);

/*
 * Two trace styles - added 31/07/2026 per the project owner, who
 * wanted the option to switch to a plain continuous-line look (single
 * flat fill color + bright outline on a dark navy background, like a
 * classic panadapter) as an alternative to the existing palette-
 * gradient "heat map" bars. Toggled live via spectrum_set_style() -
 * the NEXT spectrum_draw() call picks it up, no re-init needed. Only
 * affects the SPECTRUM panel's own fill/background/trace colors -
 * the waterfall keeps using spectrum_colormap() (the palette LUT)
 * regardless of this setting, since a flat-color waterfall would be
 * useless (it's the palette gradient that makes a waterfall readable
 * at a glance).
 */
typedef enum {
    SPECTRUM_STYLE_HEATMAP = 0, /* existing palette-gradient bars (default, unchanged) */
    SPECTRUM_STYLE_LINE = 1     /* flat fill + bright outline on a dark navy background */
} spectrum_style_t;

void spectrum_set_style(spectrum_style_t style);
spectrum_style_t spectrum_get_style(void);

/*
 * Spatial line smoothing - added 01/08/2026, exposed live through the
 * repurposed NB tile/button/badge in main.c (see s_spec_smooth_passes'
 * comment there; the old NB noise-blanker flag never drove any real
 * DSP, so its UI slot was free). Runs a 3-tap (1,2,1)/4 box filter
 * across neighboring columns' bar heights, `passes` times, AFTER
 * quantization to pixels - it only softens the on-screen trace/fill
 * edge, it never touches the per-column EMA/peak-hold state that
 * spectrum_draw() keeps between frames (see its Pass 1 vs Pass 1.5
 * comments), so it can't accumulate or drift frame to frame the way
 * smoothing the EMA itself would.
 *
 * 0 disables it (bin-sharp edge, the original look). Each additional
 * pass costs one more ~w-iteration integer loop per frame (negligible
 * next to the w*h pixel loop) and rounds the trace a bit further;
 * 2-3 passes is the range the project owner found gives a visibly
 * smoother line without flattening real peaks. Clamped internally to
 * [0, SPECTRUM_LINE_SMOOTH_MAX].
 */
#define SPECTRUM_LINE_SMOOTH_MAX 5
void spectrum_set_line_smooth(uint8_t passes);
uint8_t spectrum_get_line_smooth(void);

/* dB -> RGB565 through the shared palette (LUT-backed; the float here
 * is fine for per-COLUMN use, e.g. coloring a waterfall line - just
 * never call it per pixel). */
uint16_t spectrum_colormap(float db, float db_min, float db_max);

/*
 * Draws `n_bins` dB values into the rectangle (x,y,w,h). Columns
 * average their bins; smoothing/peak state is kept internally per
 * column (it self-resets if w changes). db_min/max fix the vertical
 * scale and the palette range.
 *
 * center_mark_offset_px shifts the SPECTRUM_CENTER_MARK line from the
 * true horizontal center by this many pixels (positive = right, i.e.
 * higher frequency) - pass 0 to mark the true center (the LO) as
 * before. Clamped internally to stay inside the plot area.
 *
 * band_active/band_lo_offset_px/band_hi_offset_px - added 03/08/2026,
 * per the project owner: an optional tinted background region behind
 * the trace showing WHICH slice of the panadapter is actually being
 * demodulated right now (AM/USB/LSB's audio bandwidth, mapped back
 * onto the frequency axis) - so it's visually obvious what you're
 * listening to, not just where the LO/demod point sits. band_active=0
 * skips it entirely (draw exactly as before - used for NFM/WFM, which
 * don't have a caller-selectable audio bandwidth the way AM/SSB do).
 * When active, band_lo_offset_px/band_hi_offset_px are pixel offsets
 * from the panel's horizontal center, SAME origin and sign convention
 * as center_mark_offset_px (so the caller can build them directly on
 * top of that same value - see main.c's call site) - lo/hi order
 * doesn't matter, both get clamped and sorted internally. The tint
 * only ever replaces BACKGROUND pixels (gridline or plain fill,
 * whichever would've shown) - it never covers the trace, the bar
 * fill, the peak marker, or the center mark, all of which keep
 * drawing on top of it exactly as they would without it.
 */
void spectrum_draw(const float *db, uint32_t n_bins,
                    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    float db_min, float db_max,
                    int16_t center_mark_offset_px,
                    uint8_t band_active,
                    int16_t band_lo_offset_px, int16_t band_hi_offset_px);

#endif /* SPECTRUM_H */
