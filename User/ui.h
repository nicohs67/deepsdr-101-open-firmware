#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "gfx.h"

/*
 * Base de widgets, construida encima de gfx.c (sin framebuffer, redibuja
 * directo a GRAM). Preparada para tactil resistivo aunque todavia no
 * este soportado en hardware:
 *
 *   - Los widgets interactivos (por ahora solo ui_button_t) tienen
 *     estado propio (pressed) y un callback de eventos, en vez de ser
 *     solo "algo que se dibuja".
 *   - ui_screen_t es un registro de widgets de una pantalla. Cuando
 *     exista el driver de tactil (touch.c, pendiente - primero hay que
 *     ver si el panel tiene pin de IRQ o hay que hacer polling puro),
 *     lo unico que hara falta es llamar a ui_screen_touch(x, y, pressed)
 *     con cada lectura/evento. Esta funcion no asume nada sobre COMO se
 *     obtienen esas lecturas (polling en el bucle principal, timer, o
 *     flag desde una ISR de pin de IRQ que se consume fuera de la ISR).
 *   - El dispatcher solo redibuja el widget cuyo estado visual cambia
 *     (no la pantalla entera) porque cada redibujado es una ventana EXMC
 *     nueva - repintar todo en cada evento de touch seria caro.
 *
 * Todo con arrays estaticos de tamano fijo (UI_SCREEN_MAX_WIDGETS) para
 * no meter malloc/heap dinamico en un sistema con la RAM ya ajustada.
 */

#define UI_SCREEN_MAX_WIDGETS 16

/* Eventos que puede recibir el callback de un widget interactivo.
 * Modelo de un boton tactil tipico:
 *   - dedo baja dentro del boton      -> UI_EVENT_PRESS
 *   - dedo sube SIN haber salido      -> UI_EVENT_RELEASE (esto es "el click")
 *   - dedo sale del area sin soltar,
 *     o suelta fuera del area         -> UI_EVENT_CANCEL (no cuenta como click)
 */
typedef enum {
    UI_EVENT_PRESS,
    UI_EVENT_RELEASE,
    UI_EVENT_CANCEL,
} ui_event_t;

/* widget: puntero al ui_button_t (u otro widget interactivo futuro) que
 * genero el evento, para que un callback compartido entre varios botones
 * pueda distinguir cual fue. user_data es lo que se paso al registrar el
 * widget (p.ej. un id de pantalla o un puntero a estado de la app). */
typedef void (*ui_callback_t)(void *widget, ui_event_t event, void *user_data);

typedef struct {
    uint16_t x, y, w, h;
    uint16_t bg;
    uint16_t border; /* usar bg para "sin borde" */
} ui_panel_t;

typedef struct {
    uint16_t x, y;
    const char *text;
    uint16_t fg;
    uint16_t bg;
    uint8_t  text_scale;
} ui_label_t;

typedef struct ui_button_s {
    uint16_t x, y, w, h;
    const char *label;
    uint16_t fg;
    uint16_t bg;
    uint16_t border;
    uint8_t  text_scale;
    uint8_t  pressed;  /* estado visual actual, lo gestiona ui_screen_touch() */
    uint8_t  enabled;  /* 0 = no reacciona a toques (se sigue pintando, ver ui_button_draw) */
    ui_callback_t on_event; /* NULL si no se quiere callback (widget solo visual) */
    void *user_data;
} ui_button_t;

/* --- Dibujo de widgets sueltos (sin pasar por ui_screen_t) --- */
void ui_panel_draw(const ui_panel_t *panel);
void ui_button_draw(const ui_button_t *btn);
void ui_label_draw(const ui_label_t *label);
uint8_t ui_button_hit(const ui_button_t *btn, uint16_t px, uint16_t py);

/* --- Registro de pantalla + despacho de toques --- */
typedef enum {
    UI_WIDGET_PANEL,
    UI_WIDGET_LABEL,
    UI_WIDGET_BUTTON,
} ui_widget_type_t;

typedef struct {
    ui_widget_type_t type;
    void *widget; /* ui_panel_t* / ui_label_t* / ui_button_t* segun type */
} ui_widget_ref_t;

typedef struct {
    ui_widget_ref_t widgets[UI_SCREEN_MAX_WIDGETS];
    uint8_t count;
    int8_t  active_index; /* indice en widgets[] del boton bajo el dedo desde
                              el ultimo PRESS, o -1 si no hay ninguno */
} ui_screen_t;

void ui_screen_init(ui_screen_t *screen);

/* Registran el widget (por puntero, el screen NO se queda con una copia)
 * y lo pintan en orden de insercion (los añadidos despues quedan
 * "encima" a la hora de hacer hit-test si se solapan). Devuelven 1 si se
 * pudo añadir, 0 si el screen ya esta lleno (UI_SCREEN_MAX_WIDGETS). */
uint8_t ui_screen_add_panel(ui_screen_t *screen, ui_panel_t *panel);
uint8_t ui_screen_add_label(ui_screen_t *screen, ui_label_t *label);
uint8_t ui_screen_add_button(ui_screen_t *screen, ui_button_t *button);

/* Dibuja todos los widgets registrados, en orden de insercion. Llamar
 * una vez al construir la pantalla (equivalente a lo que hacia
 * demo_screen_draw() pintando cada widget a mano). */
void ui_screen_draw(ui_screen_t *screen);

/* Despacha un evento de toque a los botones registrados. x,y en
 * coordenadas de pantalla (mismas unidades que gfx.c). pressed=1
 * mientras el dedo sigue en contacto (llamar en cada lectura, no solo en
 * flancos), pressed=0 en el evento de "dedo levantado". Gestiona el
 * estado PRESS/RELEASE/CANCEL descrito arriba y solo redibuja el boton
 * cuyo estado visual cambia. */
void ui_screen_touch(ui_screen_t *screen, uint16_t x, uint16_t y, uint8_t pressed);

#endif /* UI_H */

