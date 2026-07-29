#include "spectrum.h"
#include "gfx.h"

uint16_t spectrum_colormap(float db, float db_min, float db_max)
{
    float t;
    uint8_t r, g, b;

    if (db_max <= db_min) {
        return GFX_COLOR_BLACK; /* rango invalido, evitar division por 0 */
    }
    t = (db - db_min) / (db_max - db_min);
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }

    /* 4 tramos: negro/azul -> cian -> verde/amarillo -> rojo, cada uno
     * ocupando 1/4 de la escala. Interpolacion lineal simple dentro de
     * cada tramo (sin floats caros, todo con la t ya en [0,1]). */
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
 * OPTIMIZACION (28/07/2026): la version anterior llamaba a gfx_vline()
 * una vez POR COLUMNA (hasta 800 veces por frame), y cada gfx_vline
 * reabre una ventana EXMC completa (CASET+RASET+RAMWR) - carisimo
 * comparado con un unico gfx_blit. Reescrito para construir una
 * franja de una fila (ancho completo) por cada fila de pixeles del
 * rectangulo y volcarla con un unico gfx_blit por fila: pasamos de
 * ~w aperturas de ventana (~800) a ~h (~100), casi un orden de
 * magnitud menos. El calculo de que columnas estan "encendidas" en
 * cada fila se hace primero en un array pequeño en RAM (bar_h[w],
 * 800*2=1.6KB de pila - aceptable, no es estatico ni persistente).
 */
/* estaticos (.bss, RAM principal) y NO locales de pila: con solo 2KB
 * de _Min_Stack_Size en el linker script y la TCM ya ajustada (ver
 * presupuesto en waterfall.h), 2*800*2=3200 bytes en la pila era
 * arriesgado - aqui hay mucho mas margen. */
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

    /* pasada 1: altura de barra por columna, sin tocar EXMC */
    for (col = 0; col < w; col++) {
        uint32_t bin = ((uint32_t)col * n_bins) / w;
        float v = db[bin];
        float t = (v - db_min) / (db_max - db_min);
        if (t < 0.0f) { t = 0.0f; }
        if (t > 1.0f) { t = 1.0f; }
        s_bar_h[col] = (uint16_t)(t * (float)h);
    }

    /* pasada 2: una franja horizontal completa por fila, un solo
     * gfx_blit por fila (h llamadas en vez de w) */
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
