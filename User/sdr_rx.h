#ifndef SDR_RX_H
#define SDR_RX_H

#include <stdint.h>

/*
 * Captura de muestras RX del AIC3204 via I2S1_ADD (extension full-duplex
 * de SPI1/I2S1, PB14=SDext), por DMA en modo circular.
 *
 * MAPEO DMA (28/07/2026, confirmado contra datasheet real - Jorge aporto
 * la foto de la Tabla 10-2 "Peripheral requests to DMA0" del GD32F4xx
 * User Manual, la misma tabla que ya nos saco del atolladero con
 * SPI1_TX): I2S1_ADD_RX -> DMA0, Channel 3, PERIEN[2:0]=011
 * (DMA_SUBPERI3). Esta vez SI viene de la tabla, no de memoria.
 *
 * MODELO PING-PONG: un unico buffer circular de 2*SDR_RX_BLOCK_SAMPLES
 * (estereo, 16 bits). Cuando el DMA lleva la primera mitad (flag HTF) la
 * primera mitad del buffer ya esta completa y estable (el DMA esta
 * escribiendo en la segunda); cuando completa la vuelta (flag FTF) es
 * la segunda mitad la que esta lista. sdr_rx_poll_block() hace ese
 * seguimiento y devuelve NULL si no hay bloque nuevo listo todavia
 * (no bloqueante, pensado para llamarse desde el bucle principal igual
 * que el resto del proyecto).
 */

#define SDR_RX_BLOCK_SAMPLES   512U   /* muestras MONO por bloque entregado (= tamano de FFT) */

/* Arranca el DMA de captura. Llamar DESPUES de gd32_i2s_init_master_48k()
 * (necesita que SPI1/I2S1_ADD ya esten inicializados y habilitados). */
void sdr_rx_init(void);

/*
 * Si hay un bloque nuevo completo desde la ultima llamada, copia
 * SDR_RX_BLOCK_SAMPLES muestras de I (izquierdo, IN2_L/IN2_R) a
 * `i_out` y de Q (derecho, IN3_R/IN3_L) a `q_out`, ya deintercaladas,
 * y devuelve 1. Si todavia no hay bloque nuevo, devuelve 0 y no toca
 * ninguno de los dos buffers. No bloqueante.
 */
uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out);

#endif /* SDR_RX_H */
