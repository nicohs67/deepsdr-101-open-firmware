#ifndef WATERFALL_H
#define WATERFALL_H

#include <stdint.h>

/*
 * Framebuffer parcial en RAM para la zona de waterfall (y, mas adelante,
 * espectro si se decide usar el mismo mecanismo para su traza).
 *
 * POR QUE UN BUFFER PARCIAL Y NO UN FRAMEBUFFER COMPLETO:
 *   480 * 800 * 2 bytes = 750KB, muy por encima de los 192KB de RAM
 *   principal (mas 64KB de TCM, ya casi agotados por pila/heap segun el
 *   linker script). Un framebuffer parcial de solo la altura visible del
 *   waterfall si cabe, y es la unica forma de hacer scroll vertical barato
 *   (mover filas en RAM es memmove; hacerlo directo en GRAM via EXMC
 *   implicaria releer+reescribir toda la ventana en cada frame).
 *
 * PRESUPUESTO DE RAM (ajustar WATERFALL_ROWS segun lo que quede libre
 * una vez se sumen los buffers de DSP/FFT, que aun no estan
 * dimensionados):
 *   tamano_buffer = WATERFALL_WIDTH * WATERFALL_ROWS * 2 bytes
 *   Con los valores por defecto (800 x 60): 96000 bytes (~93.75KB, igual
 *   presupuesto que la version anterior 480x100 - mismo tamano total,
 *   solo cambia la forma del buffer para que quepa el ancho real del
 *   panel, ver gfx.h). Sigue siendo la mitad larga de los 192KB
 *   disponibles - probablemente haya que BAJAR WATERFALL_ROWS en cuanto
 *   se sepa cuanta RAM piden los buffers de FFT/IQ.
 *
 * MODELO DE SCROLL: el buffer se trata como un anillo logico simple:
 *   - waterfall_push_line() desplaza todas las filas una posicion hacia
 *     "abajo" (memmove) y escribe la fila nueva en la posicion 0 (arriba),
 *     de forma que las señales nuevas entran por arriba y bajan, que es
 *     la convencion habitual en waterfalls de SDR.
 *   - waterfall_blit() vuelca el buffer completo a la GRAM en una sola
 *     ventana EXMC (gfx_blit), en la posicion de pantalla indicada.
 *
 * El "colormap" (dB -> RGB565) NO esta aqui: waterfall_push_line() recibe
 * ya un array de uint16_t en RGB565, para no acoplar este modulo a una
 * escala de color concreta. La conversion FFT bin -> RGB565 se hara en la
 * capa de SDR/DSP mas adelante.
 */

#define WATERFALL_WIDTH  800   /* ancho de pantalla completo (horizontal, ver gfx.h) */
#define WATERFALL_ROWS   60    /* filas visibles simultaneamente, ver presupuesto de RAM arriba */

/* Reserva el buffer (en .bss, RAM principal) y lo pone a negro. Llamar
 * una vez al arrancar, antes del primer waterfall_push_line(). */
void waterfall_init(void);

/* Desplaza el buffer una fila hacia abajo y mete `line` (WATERFALL_WIDTH
 * pixeles RGB565) como fila superior nueva. O(WATERFALL_WIDTH*WATERFALL_ROWS)
 * por el memmove - a tener en cuenta si se llama a alta frecuencia. */
void waterfall_push_line(const uint16_t *line);

/* Vuelca el buffer completo a la GRAM en (x,y). No hace falta llamarlo en
 * cada push_line si se prefiere desacoplar tasa de actualizacion de datos
 * vs. tasa de refresco de pantalla. */
void waterfall_blit(uint16_t x, uint16_t y);

/* Acceso directo a una fila del buffer (0 = mas reciente/arriba), por si
 * se necesita pintar encima (cursores, marcadores de frecuencia, etc.)
 * sin pasar por waterfall_push_line(). NULL si row fuera de rango. */
uint16_t *waterfall_row(uint16_t row);

#endif /* WATERFALL_H */
