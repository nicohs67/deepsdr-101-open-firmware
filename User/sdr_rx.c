#include "sdr_rx.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

/*
 * Buffer circular RAW (estereo, L/R intercalado), doble del tamano de
 * bloque entregado: primera mitad + segunda mitad, ping-pong via las
 * flags HTF/FTF del propio canal DMA (mismo patron de poll no
 * bloqueante que usamos en gd32_i2s_dma_start_test_tone(), coherente
 * con el resto del proyecto).
 */
#define RAW_HALF_WORDS   (SDR_RX_BLOCK_SAMPLES * 2U) /* palabras de 16 bits por mitad (L+R intercalado) */
#define RAW_TOTAL_WORDS  (RAW_HALF_WORDS * 2U)

static int16_t s_raw_buf[RAW_TOTAL_WORDS];

/* 0 = todavia no se ha entregado ninguna mitad; 1 = la ultima entregada
 * fue la primera mitad (toca esperar FTF para la segunda); 2 = la
 * ultima entregada fue la segunda (toca esperar HTF para la primera). */
static uint8_t s_last_half_delivered;

void sdr_rx_init(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    s_last_half_delivered = 0U;

    rcu_periph_clock_enable(RCU_DMA0);

    /*
     * I2S1_ADD_RX -> DMA0, Channel 3, DMA_SUBPERI3 - confirmado contra
     * la Tabla 10-2 del GD32F4xx User Manual (misma fuente que resolvio
     * el canal de TX). Direccion PERIPH_TO_MEMORY (leemos del
     * periferico hacia la RAM), al contrario que en el tono de prueba.
     */
    dma_deinit(DMA0, DMA_CH3);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(I2S1_ADD);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_raw_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.number              = RAW_TOTAL_WORDS;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH3, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH3, DMA_SUBPERI3);

    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_HTF);
    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_FTF);
    dma_channel_enable(DMA0, DMA_CH3);

    /* Habilitar la peticion DMA de recepcion en I2S1_ADD - sin esto el
     * canal esta armado pero nadie lo dispara (mismo fallo que ya
     * tuvimos en TX, aqui aplicado al lado RX). */
    spi_dma_enable(I2S1_ADD, SPI_DMA_RECEIVE);

    debug_print("sdr_rx: DMA0 CH3 (I2S1_ADD_RX) armado en modo circular ping-pong, "
                "bloque = ");
    debug_print_dec("sdr_rx: SDR_RX_BLOCK_SAMPLES", SDR_RX_BLOCK_SAMPLES);
}

uint32_t sdr_rx_poll_block_iq(int16_t *i_out, int16_t *q_out)
{
    uint32_t n;

    /* Mitad 1 (offset 0): lista cuando salta HTF y no era ya la ultima
     * entregada. */
    if ((s_last_half_delivered != 1U) &&
        (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_HTF) == SET)) {
        dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_HTF);
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            /* deintercalar: par=izquierdo(I, IN2), impar=derecho(Q, IN3) */
            i_out[n] = s_raw_buf[2U * n];
            q_out[n] = s_raw_buf[2U * n + 1U];
        }
        s_last_half_delivered = 1U;
        return 1U;
    }

    /* Mitad 2 (offset RAW_HALF_WORDS): lista cuando salta FTF. */
    if ((s_last_half_delivered != 2U) &&
        (dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_FTF) == SET)) {
        dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_FTF);
        for (n = 0; n < SDR_RX_BLOCK_SAMPLES; n++) {
            i_out[n] = s_raw_buf[RAW_HALF_WORDS + 2U * n];
            q_out[n] = s_raw_buf[RAW_HALF_WORDS + 2U * n + 1U];
        }
        s_last_half_delivered = 2U;
        return 1U;
    }

    return 0U;
}
