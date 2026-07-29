#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdint.h>

/*
 * Dibuja el espectro instantaneo (traza en vivo, no historico - la
 * historia es el waterfall.c ya existente) en un rectangulo fijo de
 * pantalla, y expone el mismo mapa de color dB->RGB565 para que
 * gd32_i2s.c/main.c lo reutilicen al colorear la fila que se empuja a
 * waterfall_push_line() - un unico mapa de color para los dos, para
 * que la escala se lea igual en ambos.
 */

/* dB -> RGB565, escala fija db_min..db_max (clamp fuera de rango).
 * Paleta clasica de waterfall SDR: azul (frio/silencio) -> cian ->
 * verde -> amarillo -> rojo (caliente/señal fuerte). */
uint16_t spectrum_colormap(float db, float db_min, float db_max);

/*
 * Pinta `n_bins` valores de dB como barras verticales que llenan el
 * rectangulo (x,y,w,h). Si n_bins != w, se resamplea (vecino mas
 * cercano) para estirar/encoger al ancho real de pantalla. db_min/max
 * fijan la escala vertical (y tambien se usan para el color, via
 * spectrum_colormap).
 */
void spectrum_draw(const float *db, uint32_t n_bins,
                    uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    float db_min, float db_max);

#endif /* SPECTRUM_H */
