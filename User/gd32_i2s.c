#include "gd32_i2s.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

static void tone_buf_fill_1khz(void);
static float sinf_approx(float x);

/*
 * DIAGNOSTICO TEMPORAL: antes de configurar I2S para nada, forzamos los
 * 5 pines implicados a GPIO de salida normal y los hacemos parpadear a
 * mano. Objetivo: aislar si el problema es especifico del periferico
 * I2S (registros bien pero la señal no sale por AF) o si es un
 * problema mas basico de wiring/pin fisico (ni siquiera un GPIO llano
 * se ve en el osciloscopio). Quitar esta funcion y su llamada en
 * main.c una vez diagnosticado.
 */
void gd32_i2s_pins_gpio_toggle_test(uint32_t cycles)
{
    uint32_t i;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    {
        uint32_t pinsB = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pinsB);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pinsB);
        gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    }

    debug_print("gd32_i2s: prueba de parpadeo GPIO en PB12/13/14/15 y PC6 "
                "(sin I2S) - mide con el osciloscopio ahora\n");

    for (i = 0; i < cycles; i++) {
        gpio_bit_set(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_bit_set(GPIOC, GPIO_PIN_6);
        {
            volatile uint32_t d;
            for (d = 0; d < 500000U; d++) {
                __NOP();
            }
        }
        gpio_bit_reset(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        gpio_bit_reset(GPIOC, GPIO_PIN_6);
        {
            volatile uint32_t d;
            for (d = 0; d < 500000U; d++) {
                __NOP();
            }
        }
    }

    debug_print("gd32_i2s: prueba de parpadeo GPIO terminada\n");
}

void gd32_i2s2_isolation_test(void)
{
    /*
     * Antes de tocar I2S2 para nada: parpadeo GPIO llano en PA4/PC10/PC12,
     * igual que ya hicimos con los pines de I2S1, para descartar un
     * problema basico de wiring/acceso en estos pines concretos antes
     * de interpretar el resultado de la prueba de aislamiento.
     */
    {
        uint32_t i;

        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_GPIOC);

        gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10 | GPIO_PIN_12);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_12);

        debug_print("gd32_i2s2_test: parpadeo GPIO en PA4/PC10/PC12 (sin I2S) - mide ahora\n");

        for (i = 0; i < 15; i++) {
            gpio_bit_set(GPIOA, GPIO_PIN_4);
            gpio_bit_set(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
            {
                volatile uint32_t d;
                for (d = 0; d < 500000U; d++) { __NOP(); }
            }
            gpio_bit_reset(GPIOA, GPIO_PIN_4);
            gpio_bit_reset(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
            {
                volatile uint32_t d;
                for (d = 0; d < 500000U; d++) { __NOP(); }
            }
        }
        debug_print("gd32_i2s2_test: parpadeo GPIO terminado, configurando I2S2 ahora\n");
    }

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI2);

    spi_i2s_deinit(SPI2);

    /* PA4=WS, PC10=CK, PC12=SD - pines libres, sin relacion con el AIC3204 */
    {
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_4);
    }
    {
        uint32_t pinsC = GPIO_PIN_10 | GPIO_PIN_12;
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, pinsC);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pinsC);
        gpio_af_set(GPIOC, GPIO_AF_5, pinsC);
    }

    /* Ya NO reconfiguramos PLLI2S (ver aviso en gd32_i2s_init_master_48k) -
     * dejamos el valor de reset (N=192/R=4), igual que el firmware
     * original confirmado funcionando. */
    i2s_init(SPI2, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI2, I2S_AUDIOSAMPLE_48K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_DISABLE);
    i2s_enable(SPI2);

    /*
     * FIX (28/07/2026, cuarta vuelta): una unica palabra dummy produce
     * ~16 ciclos de reloj (~2-3us a 6MHz) - invisible salvo con trigger
     * single. Alimentacion sostenida (igual que en I2S1) para que el
     * resultado del test de aislamiento sea interpretable.
     */
    {
        uint32_t i;
        uint32_t timeout_hits = 0;
        debug_print("gd32_i2s2_test: alimentando datos dummy en continuo ~3s - mide PC10 AHORA\n");
        for (i = 0; i < 1200000U; i++) {
            uint32_t wait_cycles = 0;
            while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET) {
                wait_cycles++;
                if (wait_cycles > 100000U) { timeout_hits++; break; }
            }
            if (timeout_hits > 3) {
                debug_print("gd32_i2s2_test: TBE nunca se libera - abandonando bucle\n");
                break;
            }
            spi_i2s_data_transmit(SPI2, (i & 1U) ? 0xAAAAU : 0x5555U);
        }
        debug_print("gd32_i2s2_test: alimentacion continua terminada\n");
    }

    debug_print_hex32("gd32_i2s2_test: SPI_I2SCTL(SPI2) tras enable", SPI_I2SCTL(SPI2));
    debug_print_hex32("gd32_i2s2_test: SPI_I2SPSC(SPI2)", SPI_I2SPSC(SPI2));
    debug_print("gd32_i2s2_test: SPI2/I2S2 configurado - WS=PA4 CK=PC10 SD=PC12 "
                "(sin MCK). Verificar con osciloscopio en PC10 (CK, deberia dar "
                "la misma BCLK ~1.536MHz que esperabamos en I2S1).\n");
}

void gd32_i2s_init_master_48k(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_SPI1);

    /*
     * DIAGNOSTICO (28/07/2026, undecima vuelta) - RESUELTO: aquello era
     * un corto de la propia sonda del osciloscopio, no un problema de
     * firmware ni de PCB. Jorge confirmo despues contra el esquematico
     * que PC6 SI va cableado al MCLK del AIC3204 de verdad - ver el
     * cambio de la duodecima vuelta un poco mas abajo, donde se activa
     * MCKOUT y se configura PC6 como AF5.
     */
    debug_print_hex32("gd32_i2s: RCU_PLLI2S ANTES de spi_i2s_deinit", RCU_PLLI2S);

    /*
     * Reset limpio de SPI1/I2S1/I2S1_ADD por si quedaba algun estado
     * heredado (del bootloader, o de una config previa) interfiriendo -
     * probando esto porque los registros de control salian perfectos
     * por lectura pero los pines se comportaban como flotantes en el
     * osciloscopio, algo que no encaja con "todo bien configurado".
     */
    spi_i2s_deinit(SPI1);
    debug_print("gd32_i2s: SPI1/I2S1 reseteado via RCU antes de configurar\n");
    debug_print_hex32("gd32_i2s: RCU_PLLI2S DESPUES de spi_i2s_deinit", RCU_PLLI2S);

    /*
     * CAMBIO (28/07/2026, segunda vuelta): confiar en "no tocar PLLI2S"
     * NO resulto fiable - con ciclo de alimentacion real (confirmado con
     * Jorge que asi lo hace siempre, no solo reset por SWD), nuestro
     * firmware arranca con RCU_PLLI2S=0x24003000 (N=192,R=2) mientras
     * que el firmware original arranca con 0x40003000 (N=192,R=4) -
     * mismo N, R distinto, SIN que ninguno de los dos binarios toque
     * este registro por software (confirmado por desensamblado). La
     * causa exacta de esa discrepancia entre arranques sigue sin
     * explicacion clara, pero la solucion practica es forzar los
     * valores verificados DOS VECES con el firmware original
     * (osciloscopio + lectura de registro en vivo, ambas con ciclo de
     * alimentacion real) en vez de depender de un estado heredado que
     * no resulta reproducible.
     */
    /*
     * CAMBIO (28/07/2026, quinta vuelta): N=192/R=4 era INVALIDO con
     * nuestro arbol de relojes real. system_gd32f4xx.c usa la rama
     * 200M_PLL_25M_HXTAL con PLLM=25 hardcodeado (pensada para cristal
     * de 25MHz) pero el cristal real es 12.288MHz -> frecuencia de
     * comparacion = 12.288/25 = 491.5kHz. Con N=192 el VCO del PLLI2S
     * quedaba en 94.4MHz, POR DEBAJO del minimo de 100MHz del rango del
     * VCO -> el PLL probablemente nunca enganchaba -> sin i2sclock ->
     * TBE clavado y cero señal en los pines (el sintoma exacto).
     * Con N=400/R=4: VCO=196.6MHz (en spec), i2sclock=49.152MHz, y
     * 49.152/(32x192k)=8 exacto -> BCLK=6.144MHz y WS=192kHz CLAVADOS,
     * lo mismo que se midio en el firmware original, sin tocar el
     * reloj de sistema (EXMC/pantalla intactos).
     *
     * ACTUALIZADO (28/07/2026, decimocuarta vuelta): PSC del PLL
     * principal corregido de 25 a 8 en system_gd32f4xx.c (arreglo del
     * HCLK/reloj de sistema, ver ese archivo). Como PSC es el MISMO
     * campo que usa plli2sm aqui, N=400/R=4 ya no es valido con PSC=8
     * (comparacion=1.536MHz en vez de 491.5kHz -> VCO se dispararia a
     * 614MHz, muy por encima del maximo de 432MHz). Recalculado a
     * N=128/R=4 para el MISMO i2sclock=49.152MHz exacto
     * (1.536MHz*128/4=49.152MHz) -> BCLK/MCLK/WCLK identicos a los ya
     * confirmados con el osciloscopio, sin necesidad de re-medir.
     */
    debug_print_hex32("gd32_i2s: RCU_PLLI2S ANTES de forzar N=128/R=4", RCU_PLLI2S);
    rcu_osci_off(RCU_PLLI2S_CK);
    if (rcu_plli2s_config(128U, 4U) != SUCCESS) {
        debug_print("gd32_i2s: rcu_plli2s_config(128,4) FALLO\n");
    }
    rcu_osci_on(RCU_PLLI2S_CK);
    if (rcu_osci_stab_wait(RCU_PLLI2S_CK) != SUCCESS) {
        debug_print("gd32_i2s: *** PLLI2S NO ENGANCHA (stab_wait ERROR) - sin i2sclock ***\n");
    } else {
        debug_print("gd32_i2s: PLLI2S enganchado OK (stab_wait SUCCESS)\n");
    }
    debug_print_hex32("gd32_i2s: RCU_PLLI2S tras forzar N=128/R=4", RCU_PLLI2S);

    /*
     * Medido con osciloscopio en el firmware original real (funcionando):
     * WCLK=192kHz (NO 48kHz como asumiamos), BCLK=6.14MHz (=32xWCLK,
     * formato estandar 16bit x 2 canales), PC6=1.536MHz (=BCLK/4 -
     * probablemente NO es la salida MCK del periferico I2S sino algo
     * derivado de otra forma, ya que MCKOUT esta deshabilitado segun el
     * desensamblado).
     */
    i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    /*
     * CONFIRMADO (28/07/2026, decimotercera vuelta): Jorge midio con el
     * osciloscopio MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz - la
     * relacion nativa fija de MCKOUT (256xFs MCK, 32xFs BCLK para
     * palabras de 16 bits). El WCLK=192kHz que dabamos por bueno en
     * vueltas anteriores era real SOLO mientras MCKOUT estaba
     * desactivado (ahi el divisor calculado caia dentro de rango); al
     * activar MCKOUT pedir Fs=192kHz desbordaba el calculo y la
     * libreria caia en silencio al valor de reset - la tasa real
     * pasaba a ser 48kHz sin avisar. En vez de pelear contra el
     * MCKOUT nativo para forzar 192kHz, aceptamos 48kHz como tasa de
     * trabajo real: es la combinacion "de libro" que recomienda TI
     * (Filter A, AOSR=128, MCLK=256xfs) y ya no depende de ninguna
     * prediccion - MCLK=12.288MHz esta confirmado con osciloscopio,
     * no calculado.
     */
    i2s_psc_config(SPI1, I2S_AUDIOSAMPLE_48K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);

    debug_print_hex32("gd32_i2s: SPI_I2SCTL tras i2s_init (SPI1)", SPI_I2SCTL(SPI1));
    debug_print_hex32("gd32_i2s: SPI_I2SPSC tras i2s_psc_config (SPI1)", SPI_I2SPSC(SPI1));

    /* Extension I2S1_ADD: se le pasa el MISMO modo que el bloque
     * principal (I2S_MODE_MASTERTX) - la funcion deduce internamente
     * que la extension debe ser I2S_MODE_SLAVERX (comparte reloj con el
     * maestro, solo recibe). Esta es la ruta por la que llegaran las
     * muestras I/Q del AIC3204 (PB14) en la siguiente fase con DMA. */
    i2s_full_duplex_mode_config(I2S1_ADD, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS,
                                 I2S_CKPL_LOW, I2S_FRAMEFORMAT_DT16B_CH16B);

    /*
     * FIX (28/07/2026, cuarta vuelta): la configuracion GPIO/AF estaba
     * DESPUES de i2s_enable y del bucle de alimentacion - durante toda
     * la ventana de medida los pines seguian siendo GPIO de salida
     * llano a nivel bajo (el estado en que los dejaba
     * gd32_i2s_pins_gpio_toggle_test), asi que aunque el generador
     * interno hubiese arrancado, la señal nunca habria llegado a los
     * pines fisicos. Movida aqui, ANTES de habilitar el periferico.
     *
     * WS, CK, SD(TX): AF5. Pines PB12/13/14/15, bloque SPI1/I2S1 del
     * GD32F450. PC6 (MCLK) - CONFIRMADO por Jorge contra el esquematico
     * (duodecima vuelta) que SI va cableado al AIC3204 de verdad - se
     * configura tambien como AF5 aqui, junto con el resto.
     */
    {
        uint32_t pins = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);
        gpio_af_set(GPIOB, GPIO_AF_5, pins);
    }
    {
        gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
        gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
        gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_14);
    }
    {
        gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
        gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_6);
    }

    debug_print_hex32("gd32_i2s: GPIOB_CTL crudo (pines 12-15 en bits[31:24], AF=10b)", GPIO_CTL(GPIOB));
    debug_print_hex32("gd32_i2s: GPIOB_AFSEL1 crudo (deberia ser 0x55550000 en nibbles 12-15)", GPIO_AFSEL1(GPIOB));
    debug_print_hex32("gd32_i2s: GPIOC_CTL crudo (pin6 = bits[13:12], AF=10b)", GPIO_CTL(GPIOC));
    debug_print_hex32("gd32_i2s: GPIOC_AFSEL0 crudo (pin6 = bits[27:24], deberia ser 0x5)",
                       GPIO_AFSEL0(GPIOC));
    debug_print("gd32_i2s: PC6/MCLK activado (MCKOUT enable). CONFIRMADO por Jorge con "
                "osciloscopio: MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz (256x/32x fijo "
                "de MCKOUT). Fs real de trabajo = 48kHz, no 192kHz.\n");

    i2s_enable(SPI1);
    i2s_enable(I2S1_ADD);

    /*
     * DIAGNOSTICO: comportamiento conocido (y mal documentado) en
     * perifericos I2S estilo STM32/GD32 en modo maestro-transmisor - el
     * generador de reloj interno puede no arrancar hasta que se escribe
     * al menos una palabra de datos al registro de datos, aunque I2SEN
     * ya este a 1.
     */
    /*
     * CAMBIO (28/07/2026, tercera vuelta): una sola palabra dummy
     * (spi_i2s_data_transmit una vez) probablemente solo produce un
     * pulso de reloj de unos pocos microsegundos - practicamente
     * invisible en un osciloscopio esperando una señal periodica
     * estable. Probando con alimentacion CONTINUA de datos (esperando a
     * TBE antes de cada palabra) durante un rato, para tener una señal
     * sostenida de verdad que se pueda medir. Si esto muestra reloj
     * donde antes no habia nada, confirma que el generador esta
     * gateado por disponibilidad de datos (no libre-corriente), y la
     * siguiente fase (DMA circular) sera indispensable, no opcional.
     *
     * FIX (28/07/2026, cuarta vuelta): el bucle de la tercera vuelta
     * esperaba a TBE pero NUNCA escribia nada en el registro de datos
     * (faltaba el spi_i2s_data_transmit dentro del bucle). Como no se
     * escribia nada, TBE estaba permanentemente a 1, las 200000 vueltas
     * se completaban en microsegundos y el generador de reloj jamas
     * arrancaba - por eso el osciloscopio no mostraba NADA en WS/CK/SD.
     * Ahora se escribe de verdad, con patron alternante 0xAAAA/0x5555
     * para que ademas haya actividad visible en SD (PB15), no solo en
     * los relojes. A 192kHz x 16bit x 2ch, 200000 palabras ~= 0.5s de
     * señal sostenida; subido a 1200000 palabras para tener ~3s reales
     * de ventana de medida.
     */
    {
        uint32_t i;
        uint32_t timeout_hits = 0;
        uint32_t wait_events = 0;   /* iteraciones en las que hubo que esperar a TBE */
        uint32_t ch_left = 0, ch_right = 0; /* muestras del flag de canal I2SCH */
        uint32_t t0, t1, elapsed_ms;

        /* DWT cycle counter para medir la duracion real del bucle */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        t0 = DWT->CYCCNT;

        debug_print("gd32_i2s: alimentando datos dummy en continuo durante ~3s - mide el osciloscopio AHORA\n");
        for (i = 0; i < 1200000U; i++) {
            uint32_t wait_cycles = 0;
            if (spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET) {
                wait_events++;
            }
            while (spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET) {
                wait_cycles++;
                if (wait_cycles > 100000U) {
                    timeout_hits++;
                    break;
                }
            }
            if (timeout_hits > 3) {
                debug_print("gd32_i2s: TBE nunca se libera (timeout repetido) - abandonando "
                            "el bucle de alimentacion para no bloquear el resto del sistema\n");
                break;
            }
            spi_i2s_data_transmit(SPI1, (i & 1U) ? 0xAAAAU : 0x5555U);
            /* muestrear el flag de canal de vez en cuando: si el contador
             * de trama corre, se veran AMBOS valores (izq y dcha).
             * FIX (28/07/2026, octava vuelta): el paso original (0x3FF,
             * multiplo de 2) siempre caia en la misma fase par/impar del
             * canal - aliasing propio de la medida, no un dato real
             * sobre el hardware. Con un paso IMPAR (1023) se recorren
             * las dos fases con el tiempo. */
            if ((i % 1023U) == 0U) {
                if (spi_i2s_flag_get(SPI1, I2S_FLAG_CH) == SET) { ch_right++; }
                else { ch_left++; }
            }
        }
        t1 = DWT->CYCCNT;
        elapsed_ms = (uint32_t)(((uint64_t)(t1 - t0)) * 1000U / SystemCoreClock);

        debug_print("gd32_i2s: alimentacion continua terminada (o abandonada por timeout)\n");
        /*
         * INTERPRETACION (28/07/2026, sexta vuelta):
         * - duracion ~3100ms + wait_events cerca de 1200000 + ch_left y
         *   ch_right AMBOS > 0  -> el reloj interno CORRE y el shifter
         *   transmite; el problema esta entre el periferico y el pin
         *   (mapeo AF / routing).
         * - duracion de pocos ms + wait_events = 0 -> TBE nunca se
         *   limpio: las escrituras al registro de datos NO cuajan; el
         *   periferico no esta aceptando datos de verdad.
         */
        debug_print_dec("gd32_i2s: duracion bucle (ms)", elapsed_ms);
        debug_print_dec("gd32_i2s: palabras escritas", i);
        debug_print_dec("gd32_i2s: wait_events (esperas a TBE)", wait_events);
        debug_print_dec("gd32_i2s: muestras canal IZQ", ch_left);
        debug_print_dec("gd32_i2s: muestras canal DCHA", ch_right);
    }

    /*
     * FASE 3 (28/07/2026, septima vuelta): DMA circular con un tono de
     * prueba de 1kHz, para tener BCLK/WS corriendo de forma permanente
     * (ya confirmado por osciloscopio que el bloque de alimentacion
     * manual solo dura mientras el bucle escribe) y ademas verificar de
     * un tiron toda la cadena hasta el altavoz/auriculares del AIC3204.
     *
     * *** CORREGIDO (28/07/2026, decima vuelta) contra el datasheet real
     * (Jorge aporto la tabla 10-2 "Peripheral requests to DMA0" del
     * GD32F4xx User Manual): SPI1_TX esta en DMA0 (NO DMA1 como yo habia
     * puesto de memoria), Channel 4, PERIEN[2:0]=000 (DMA_SUBPERI0). El
     * canal(4)/subperipheral(0) que puse ya eran correctos - el fallo
     * era el CONTROLADOR DMA entero (DMA1 en vez de DMA0), por eso el
     * canal se armaba perfecto (bit a bit) y aun asi se quedaba inerte:
     * la peticion de SPI1_TX ni siquiera llega a DMA1. ***
     */

    debug_print_hex32("gd32_i2s: SPI_I2SCTL tras i2s_enable (SPI1, bit10=I2SEN deberia ser 1)",
                       SPI_I2SCTL(SPI1));
    debug_print_hex32("gd32_i2s: RCU_CFG0 crudo (bit toggle I2SSEL en bit 23)", RCU_CFG0);

    debug_print("gd32_i2s: I2S1 (SPI1) maestro configurado - WS=PB12 CK=PB13 SD=PB15 "
                "SDext=PB14, MCK=PC6 (activado y confirmado). "
                "i2sclock=49.152MHz (N=400/R=4). Con MCKOUT activo: Fs real=48kHz, "
                "BCLK=1.536MHz, MCLK=12.288MHz (CONFIRMADO por Jorge con osciloscopio).\n");

    gd32_i2s_dma_start_test_tone();
}

/*
 * Tabla de un tono senoidal de 1kHz, estereo (L=R), 16 bits, a la tasa
 * de trama real CONFIRMADA (48kHz, no 192kHz - ver aviso de la
 * decimotercera vuelta). 48000/1000 = 48 muestras por ciclo exacto ->
 * tabla corta y sin error de redondeo de frecuencia.
 * Amplitud moderada (no full-scale) para no arriesgar el altavoz/oido
 * en la primera prueba.
 */
#define TONE_SAMPLES_PER_CYCLE   48U
static int16_t s_tone_buf[TONE_SAMPLES_PER_CYCLE * 2U]; /* *2: L y R intercalados */

static void tone_buf_fill_1khz(void)
{
    uint32_t n;
    for (n = 0; n < TONE_SAMPLES_PER_CYCLE; n++) {
        /* seno de amplitud ~+-8000 (25% de full-scale 16 bits) via tabla
         * precalculada a mano no hace falta: usamos aproximacion con
         * math.h en tiempo de arranque no esta disponible aqui, asi que
         * se rellena con una LUT generada por el propio compilador
         * mediante float una sola vez al init (no en el hot path). */
        float angle = (2.0f * 3.14159265358979f * (float)n) / (float)TONE_SAMPLES_PER_CYCLE;
        int16_t sample = (int16_t)(8000.0f * sinf_approx(angle));
        s_tone_buf[2U * n]      = sample; /* izquierdo */
        s_tone_buf[2U * n + 1U] = sample; /* derecho (mismo tono en ambos) */
    }
}

/* Aproximacion de seno sin depender de libm (evita tirar de -lm en el
 * link del firmware si no estaba ya incluido): Bhaskara I, error tipico
 * <0.2% en todo el rango, mas que suficiente para una senoide audible
 * de prueba. */
static float sinf_approx(float x)
{
    float pi = 3.14159265358979f;
    float x2;
    int negate = 0;
    while (x > pi) { x -= 2.0f * pi; }
    while (x < -pi) { x += 2.0f * pi; }
    if (x < 0.0f) { x = -x; negate = 1; }
    x2 = (16.0f * x * (pi - x)) / (5.0f * pi * pi - 4.0f * x * (pi - x));
    return negate ? -x2 : x2;
}

void gd32_i2s_dma_start_test_tone(void)
{
    dma_single_data_parameter_struct dma_init_struct;

    tone_buf_fill_1khz();

    rcu_periph_clock_enable(RCU_DMA0);

    /* DIAGNOSTICO (28/07/2026, novena vuelta): estado crudo de SPI_STAT
     * justo antes de tocar nada de DMA, para ver si el bucle manual dejo
     * algun flag de error (overrun, etc) que pudiera estar bloqueando
     * nuevas peticiones. */
    debug_print_hex32("gd32_i2s: SPI_STAT(SPI1) justo antes de armar DMA "
                       "(bit1=TBE deberia ser 1 si esta listo para escribir)",
                       SPI_STAT(SPI1));

    /* DMA0, Channel4, PERIEN=000 (SUBPERI0) para SPI1_TX - confirmado
     * contra la tabla 10-2 del GD32F4xx User Manual (Jorge, 28/07/2026). */
    dma_deinit(DMA0, DMA_CH4);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr         = (uint32_t)&SPI_DATA(SPI1);
    dma_init_struct.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr        = (uint32_t)s_tone_buf;
    dma_init_struct.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction           = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number              = TONE_SAMPLES_PER_CYCLE * 2U;
    dma_init_struct.priority            = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA0, DMA_CH4, &dma_init_struct);

    dma_channel_subperipheral_select(DMA0, DMA_CH4, DMA_SUBPERI0);

    dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);

    /* Habilitar la peticion DMA de transmision en el propio SPI1/I2S1 -
     * sin esto el DMA esta armado pero el periferico nunca la dispara. */
    spi_dma_enable(SPI1, SPI_DMA_TRANSMIT);

    debug_print("gd32_i2s: DMA circular armado, tono de prueba 1kHz deberia sonar/medirse "
                "de forma CONTINUA (ya no se para) en PB13/PB12, y en la salida del AIC3204 "
                "si la fase 1 (I2C) dejo el codec des-muteado\n");

    /*
     * DIAGNOSTICO (28/07/2026, octava vuelta): Jorge reporta que la
     * señal aparece y se para en el MISMO punto que antes del DMA -
     * la firma tipica de "el canal DMA nunca dispara ni una peticion"
     * (el canal/subperipheral SPI1_TX no esta verificado contra el
     * datasheet, avisado en la vuelta anterior). Leemos el contador de
     * transferencias restantes DOS VECES separadas por un delay: si NO
     * cambia, el canal esta armado pero inerte (canal/subperipheral
     * equivocado, o CTL1.DMATEN no llego a cuajar); si SI cambia, el
     * canal si dispara y el problema esta en otro sitio (p.ej. el
     * propio SPI1 dejo de aceptar datos por algun motivo distinto).
     * Tambien se comprueba el bit DMATEN releido y las flags de error
     * del canal (FEE/SDE/TAE), que delatarian un canal/subperipheral
     * mal seleccionado de otra forma (acceso invalido, etc).
     */
    {
        uint32_t cnt_a, cnt_b;
        volatile uint32_t d;

        debug_print_hex32("gd32_i2s: SPI_CTL1(SPI1) tras spi_dma_enable (bit1=DMATEN deberia ser 1)",
                           SPI_CTL1(SPI1));
        debug_print_hex32("gd32_i2s: DMA_CHCTL(DMA0,CH4) crudo (bit0=CHEN deberia ser 1)",
                           DMA_CHCTL(DMA0, DMA_CH4));

        cnt_a = dma_transfer_number_get(DMA0, DMA_CH4);
        for (d = 0; d < 2000000U; d++) { __NOP(); }
        cnt_b = dma_transfer_number_get(DMA0, DMA_CH4);

        debug_print_dec("gd32_i2s: DMA_CHCNT restante, lectura A", cnt_a);
        debug_print_dec("gd32_i2s: DMA_CHCNT restante, lectura B (tras delay)", cnt_b);
        if (cnt_a == cnt_b) {
            debug_print("gd32_i2s: *** DMA_CHCNT NO SE MUEVE - el canal esta armado pero "
                        "INERTE, nunca dispara peticion (canal/subperipheral SPI1_TX "
                        "probablemente equivocado - revisar tabla DMA request mapping) ***\n");
        } else {
            debug_print("gd32_i2s: DMA_CHCNT SI decrementa - el canal dispara peticiones, "
                        "el problema esta en otro punto de la cadena\n");
        }

        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FEE) == SET) {
            debug_print("gd32_i2s: *** DMA flag FEE (FIFO error/exception) activa ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_SDE) == SET) {
            debug_print("gd32_i2s: *** DMA flag SDE (single data mode exception) activa ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_TAE) == SET) {
            debug_print("gd32_i2s: *** DMA flag TAE (transfer access error) activa - canal/"
                        "subperipheral con acceso invalido, casi seguro mal seleccionado ***\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_HTF) == SET) {
            debug_print("gd32_i2s: DMA flag HTF (media transferencia) activa - "
                        "el canal SI ha transferido al menos la mitad del buffer alguna vez\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF) == SET) {
            debug_print("gd32_i2s: DMA flag FTF (transferencia completa) activa - "
                        "el canal SI ha completado al menos una vuelta del circular\n");
        }
        if (dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_HTF) == RESET &&
            dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF) == RESET &&
            cnt_a == cnt_b) {
            debug_print("gd32_i2s: ni HTF ni FTF se han activado NUNCA - si esto sigue "
                        "saliendo tras el fix de DMA0/DMA1, hay algo mas por revisar\n");
        }
    }
}
