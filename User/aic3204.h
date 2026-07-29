#ifndef AIC3204_H
#define AIC3204_H

#include <stdint.h>

/*
 * Driver del codec TLV320AIC3204 - FASE 1: solo comunicacion por I2C
 * (deteccion de direccion, reset por software, lectura/escritura de
 * registros paginados). NO configura todavia arbol de reloj, PLL,
 * NADC/MADC, ni encendido de ADC/PGA - eso es la fase 2, una vez esto
 * este confirmado en hardware real (asi lo pidio Jorge: por fases,
 * probando cada una antes de seguir).
 *
 * El AIC3204 organiza sus registros en "paginas": el registro 0x00 de
 * CUALQUIER pagina es el "Page Control Register" - escribirlo selecciona
 * la pagina activa, y leerlo siempre devuelve la pagina actualmente
 * seleccionada. Esto da una prueba de bucle-cerrado gratuita para
 * validar el enlace I2C sin tocar ningun registro de audio: escribes una
 * pagina, la relees, y tiene que coincidir.
 *
 * Direccion I2C: por defecto 0x18 con el pin MODE del AIC3204 a masa
 * (la configuracion mas habitual en breakouts), o 0x19 si esta a VDD.
 * Si aic3204_init() no detecta 0x18, usa aic3204_scan_bus() para
 * encontrar la direccion real antes de asumir que el chip no responde.
 */

#define AIC3204_ADDR_DEFAULT 0x18U

/* nRESET del AIC3204 - PB11, activo a nivel bajo (confirmado en
 * hardware real, Jorge 27/07/2026). aic3204_init() lo saca de reset
 * antes de tocar el bus I2C - sin esto el chip nunca llega a responder
 * de forma fiable. Definido dentro de aic3204.c, no aqui, para no
 * arrastrar gd32f4xx.h a este header. */

/* Llamar una vez al arrancar: inicializa el bus I2C (i2c_bitbang) y deja
 * el driver listo para usar la direccion indicada (normalmente
 * AIC3204_ADDR_DEFAULT). No toca el chip todavia - eso lo hace
 * aic3204_probe_and_reset(). */
void aic3204_init(uint8_t i2c_addr);

/* Selecciona `page` y escribe `value` en `reg` dentro de esa pagina.
 * Devuelve 1 si ambas escrituras (seleccion de pagina + registro) dieron
 * ACK, 0 si alguna fallo. */
uint8_t aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t value);

/* Selecciona `page` y lee `reg` dentro de esa pagina en `*value`.
 * Devuelve 1 si la operacion completa (escritura de pagina + lectura)
 * dio ACK, 0 si fallo (en ese caso *value no es valido). */
uint8_t aic3204_read_reg(uint8_t page, uint8_t reg, uint8_t *value);

/*
 * Prueba de bring-up completa, pensada para ejecutarse una vez al
 * arrancar y volcar el resultado por UART (usa debug_print internamente):
 *   1. Bucle-cerrado del Page Control Register (escribe pagina 1, relee,
 *      escribe pagina 0, relee) - confirma que el enlace I2C funciona de
 *      verdad, sin asumir nada sobre el resto del chip.
 *   2. Software reset (pagina 0, registro 0x01 = 0x01) - el chip vuelve
 *      a sus valores de reset; no hay forma de leer confirmacion directa
 *      de esto (el reset es autolimpiante), pero si el bucle-cerrado del
 *      paso 1 ya funciono, el reset practicamente seguro que tambien.
 * Devuelve 1 si el paso 1 (la unica parte verificable) tuvo exito.
 */
uint8_t aic3204_probe_and_reset(void);

/* Escanea el bus I2C (direcciones 0x08-0x77) e imprime por UART
 * (debug_print) cada direccion que responda con ACK. Util si
 * aic3204_probe_and_reset() falla y hay que confirmar la direccion real
 * en vez de asumir 0x18. */
void aic3204_scan_bus(void);

/*
 * FASE 2 (28/07/2026): reloj (MCLK directo a CODEC_CLKIN, sin PLL) +
 * ruteo diferencial de entrada (I=IN2_L/IN2_R ya confirmado por Jorge
 * en hardware real; Q=IN3_R/IN3_L, extrapolado del patron de bits de
 * su propio driver Arduino y confirmado a nivel arquitectura contra
 * SLAA557 - Application Reference Guide de TI, PENDIENTE de verificar
 * el valor exacto de registro) + power-up del ADC.
 *
 * CONFIRMADO (decimotercera vuelta): MCLK=12.288MHz, BCLK=1.536MHz,
 * WCLK=48kHz, medido con osciloscopio - ya NO depende de ninguna
 * prediccion de formula. NADC=1/MADC=2/AOSR=128 estan calculados sobre
 * estos valores reales (formula: CODEC_CLKIN = NADC x MADC x AOSR x
 * ADC_FS = 1x2x128x48000 = 12.288.000, exacto).
 *
 * Llamar despues de aic3204_probe_and_reset() Y despues de que el I2S
 * ya este generando reloj real hacia el codec (gd32_i2s_init_master_48k
 * ya ejecutada) - la fase 2 configura registros de reloj que dependen
 * de que MCLK/BCLK/WCLK ya esten corriendo de verdad.
 */
void aic3204_phase2_init(void);

#endif /* AIC3204_H */
