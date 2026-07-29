#ifndef GD32_I2S_H
#define GD32_I2S_H

#include <stdint.h>


/*
 * Configuracion del periferico I2S1 (SPI1) del GD32F450 en modo
 * maestro, full-duplex (bloque principal I2S1 + extension I2S1_ADD),
 * para 48kHz estereo I/Q con el AIC3204.
 *
 * CORRECCION IMPORTANTE (27/07/2026): los pines que Jorge cablea
 * pertenecen a SPI1/I2S1, NO a SPI2/I2S2 - error inicial por copiar el
 * mapeo de un ejemplo de STM32F4 (chip distinto, numeracion de
 * periferico I2S distinta en estos pines concretos). Confirmado contra
 * la tabla oficial de pin definitions del datasheet GD32F450xx
 * (Rev2.3): PB12/13/14/15 y PC6 listan funciones I2S1_*, nunca I2S2_*.
 *
 * FASE 2 (parcial): esto SOLO genera los relojes (WS/BCLK/MCLK) y deja
 * el periferico listo para transmitir/recibir - todavia NO hay DMA
 * capturando muestras (eso es la siguiente fase, una vez se confirme
 * con el osciloscopio que las frecuencias salen bien).
 *
 * Pines (confirmados en hardware real, Jorge 27/07/2026):
 *   PB12 = WS   (I2S1_WS)        AF5 (sin verificar contra la tabla de
 *   PB13 = BCLK (I2S1_CK)        AF5  AFs especifica del GD32F450 - la
 *   PB15 = DOUT (I2S1_SD, TX)    AF5  extraccion del PDF no dio columnas
 *   PB14 = DIN  (I2S1_ADD_SD,RX) AF6  fiables. Si no hay señal pese al
 *   PC6  = MCLK (I2S1_MCK)       AF5  fix de periferico, este es el
 *                                     siguiente sospechoso.
 *
 * MCLK (28/07/2026, duodecima vuelta - CONFIRMADO): Jorge verifico
 * contra el esquematico que PC6 SI esta cableado al MCLK del AIC3204.
 * El .c activa MCKOUT y configura PC6 como AF5. OJO: la formula nativa
 * de i2s_psc_config() fija MCK=256xFs cuando MCKOUT esta activo; con
 * Fs=192kHz eso predice MCK~=49MHz, muy lejos del 1.536MHz medido en
 * el firmware original (que implica MCLK=BCLK/4, no BCLK/8). Esa
 * discrepancia esta SIN RESOLVER todavia - medir con el osciloscopio
 * que sale realmente antes de dar esto por bueno.
 */
void gd32_i2s_init_master_48k(void);

/*
 * PRUEBA DE AISLAMIENTO (Jorge, 27/07/2026): configura el OTRO bloque
 * I2S del chip (SPI2/I2S2) en pines libres sin relacion con el AIC3204
 * (PA4=WS, PC10=CK, PC12=SD, sin MCK), solo para ver si ESE arranca el
 * reloj. Si funciona, confirma que hay algo especifico de SPI1/I2S1 en
 * este chip/librería que se nos escapa; si tampoco, es un paso que nos
 * falta en general (no especifico de I2S1). AF5 sin verificar contra
 * datasheet para estos pines concretos - mismo patron que I2S1 (que si
 * se confirmo AF5), pero no dado por seguro aqui.
 */
void gd32_i2s2_isolation_test(void);

/* DIAGNOSTICO TEMPORAL - ver comentario en gd32_i2s.c. Parpadea los 5
 * pines como GPIO llano (sin I2S) para aislar si el problema es del
 * periferico o del wiring fisico. `cycles` = numero de parpadeos
 * (~100Hz cada uno, visible con osciloscopio o multimetro en AC). */
void gd32_i2s_pins_gpio_toggle_test(uint32_t cycles);

/*
 * FASE 3 (28/07/2026): arranca un canal DMA circular que alimenta
 * SPI1/I2S1 con un tono de prueba de 1kHz de forma indefinida - ya NO
 * se para como el bucle manual de la fase anterior. Se llama
 * automaticamente al final de gd32_i2s_init_master_48k(), no hace
 * falta invocarla aparte.
 */
void gd32_i2s_dma_start_test_tone(void);

#endif /* GD32_I2S_H */
