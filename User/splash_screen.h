#ifndef SPLASH_SCREEN_H
#define SPLASH_SCREEN_H

/*
 * Splash screen de arranque: 3 líneas de texto centradas + número de
 * versión abajo. Ver splash_screen.c para el contenido de las líneas,
 * el tiempo en pantalla (SPLASH_HOLD_MS) y el aviso sobre la fuente
 * (solo mayúsculas).
 *
 * Llamar UNA VEZ en main(), después de que la pantalla esté
 * inicializada y ANTES de radio_screen_draw() - ver el comentario de
 * cabecera de splash_screen_draw() para el orden exacto.
 */
void splash_screen_draw(void);

#endif /* SPLASH_SCREEN_H */
