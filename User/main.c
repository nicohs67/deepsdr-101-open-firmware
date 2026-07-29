#include "gd32f4xx.h"
#include "rm68120_exmc.h"
#include "debug_uart.h"
#include "gfx.h"
#include "ui.h"
#include "waterfall.h"
#include "touch.h"
#include "aic3204.h"
#include "gd32_i2s.h"
#include "sdr_rx.h"
#include "fft.h"
#include "spectrum.h"

static void led_gpio_init(void);
static void systick_delay_init(void);
static void demo_screen_draw(void);
static void sdr_spectrum_waterfall_tick(void);
static void demo_touch_poll(void);

/* Cambiar a 0 para volver a la demo normal una vez calibrada la altura real. */
#define CALIB_HEIGHT_TEST 0
#if CALIB_HEIGHT_TEST
static void calib_height_ruler_draw(void);
#endif

volatile uint32_t g_msticks = 0; /* incrementado en SysTick_Handler, 1 tick = 1ms real */
volatile uint16_t g_last_rddpm  = 0; /* ultimo valor leido de RDDPM (0x0A00) */
volatile uint16_t g_last_rddsdr = 0; /* ultimo valor leido de RDDSDR (0x0F00) */
volatile uint32_t g_fill_count  = 0; /* cuantos rellenos completos se han hecho */
volatile uint16_t g_panel_id_check1 = 0; /* respuesta a comando 0x000A del panel */
volatile uint16_t g_panel_id_check2 = 0; /* respuesta a comando 0x3A00 del panel */
volatile uint32_t g_system_clock_snapshot = 0; /* copia de SystemCoreClock, para verificar si el reloj arranca bien en frio */

int main(void)
{
    /* CRITICO al vivir encadenados tras el bootloader: nuestra tabla de
     * vectores ya no esta en 0x08000000 (esa es la del bootloader), sino
     * en 0x08020000. Sin esto, CUALQUIER interrupcion (incluido nuestro
     * propio SysTick) buscaria su manejador en la tabla de vectores DEL
     * BOOTLOADER, no en la nuestra - debe ser lo PRIMERO que hagamos. */
    SCB->VTOR = 0x08020000;

    /*
     * Limpiar el NVIC heredado del bootloader ANTES de reactivar
     * interrupciones globales. El bootloader pudo dejar habilitadas a
     * nivel de NVIC interrupciones suyas (su propio SysTick, algun DMA,
     * etc.) que, con VTOR ya apuntando a NUESTRA tabla de vectores
     * (posiciones sin usar = Default_Handler, un bucle infinito mudo),
     * saltarian a un cuelgue silencioso en cuanto se desenmascararan -
     * exactamente lo que paso al mover __enable_irq() demasiado pronto
     * sin este paso antes. Deshabilitar todo + limpiar pendientes deja
     * un estado limpio; nuestro propio codigo reactiva selectivamente
     * solo lo que configura (SysTick_Config mas abajo, EXTI2 en
     * touch_init() mas adelante).
     */
    {
        uint8_t i;
        for (i = 0; i < 8U; i++) {
            NVIC->ICER[i] = 0xFFFFFFFFU;
            NVIC->ICPR[i] = 0xFFFFFFFFU;
        }
    }

    /*
     * Tambien critico al vivir encadenados tras el bootloader: es
     * practica habitual que un bootloader deje las interrupciones
     * globalmente deshabilitadas (PRIMASK) antes de saltar a la
     * aplicacion, para no traspasar estado de IRQ a medias. Sin este
     * __enable_irq() aqui, NINGUNA interrupcion (SysTick incluido, EXTI
     * de touch.c mas adelante) llegaria a dispararse nunca, aunque toda
     * la configuracion de NVIC/EXTI sea correcta - exactamente lo que se
     * vio en pruebas reales: PENIRQ cambia de nivel bien (eso es lectura
     * directa de GPIO, no depende de interrupciones), pero
     * EXTI2_IRQHandler nunca se ejecutaba ni una sola vez.
     */
    __enable_irq();

    /*
     * SystemInit() ya se ha llamado desde el startup antes de main():
     * configura PLL/HXTAL segun system_gd32f4xx.c. Revisa ese fichero
     * si necesitas otra frecuencia de reloj distinta a la de fabrica.
     *
     * IMPORTANTE: la funcion de reloj activa (system_clock_200m_25m_hxtal)
     * asume 25MHz de HXTAL en sus divisores de PLL fijos (PSC=25), pero
     * el cristal real de esta placa es de 12.288MHz (confirmado con
     * osciloscopio en PH0). HXTAL_VALUE ya esta corregido via -D en el
     * Makefile, pero eso NO cambia los divisores PSC/PLLN/PLLP ya
     * programados en el hardware (son fijos, calculados para 25MHz) -
     * solo corrige el CALCULO de SystemCoreClock para que refleje la
     * realidad: el nucleo esta corriendo a ~98.3MHz de verdad, no a
     * 200MHz. SystemCoreClockUpdate() releé los registros reales del PLL
     * y recalcula SystemCoreClock con el HXTAL_VALUE correcto - sin
     * llamar a esto, SystemCoreClock se queda en el literal 200000000
     * incorrecto con el que arranca el fichero del fabricante.
     */
    SystemCoreClockUpdate();

    systick_delay_init();
    led_gpio_init();
    debug_uart_init();

    debug_print("\n\n=== ARRANQUE (encadenado tras bootloader) ===\n");
    debug_print_hex32("VTOR leido de vuelta", SCB->VTOR);

    /* Snapshot de SystemCoreClock lo antes posible, para verificar por
     * depurador si el reloj arranco a los 200MHz esperados o si el
     * cristal HXTAL fallo y quedo en un estado distinto (silenciosamente). */
    g_system_clock_snapshot = SystemCoreClock;
    debug_print_hex32("SystemCoreClock", g_system_clock_snapshot);

    /* DIAGNOSTICO: RCU_PLLI2S lo antes posible, para acotar en que punto
     * exacto del arranque cambia de 0x40003000 (valor visto con el
     * firmware original) a 0x24003000 (visto con el nuestro justo antes
     * de gd32_i2s_init_master_48k) - quitar una vez localizada la causa. */
    debug_print_hex32("RCU_PLLI2S lo antes posible en main()", RCU_PLLI2S);

    /*
     * Hasta ahora esta sesion completa confiaba en que el bootloader
     * dejara GPIO/EXMC/panel ya inicializados, y dibujabamos encima sin
     * tocar nada (por eso funciono todo el rato sin llamar a
     * rm68120_init()). Con el reloj ya corregido (HXTAL=12.288MHz real,
     * afecta a la precision de los tiempos del bus EXMC) y una
     * secuencia de init nueva confirmada en hardware real (ver
     * rm68120_exmc.h/.c), toca probar rm68120_init() por primera vez en
     * toda la sesion. RIESGO: si esto deja la pantalla peor de lo que
     * estaba (en vez de mejor), cambiar TRY_RM68120_INIT a 0 para volver
     * al comportamiento anterior (confiar en el bootloader, no tocar
     * GPIO/EXMC nosotros).
     */
#define TRY_RM68120_INIT 1
#if TRY_RM68120_INIT
    debug_print("Llamando a rm68120_init() por primera vez esta sesion...\n");
    rm68120_init();
    debug_print("rm68120_init() completado\n");
    debug_print_hex32("RCU_PLLI2S tras rm68120_init", RCU_PLLI2S);
#else
    debug_print("PRUEBA: NO llamamos a rm68120_init(). Confiamos en que el bootloader\n"
                "ya dejo el GPIO/EXMC/panel correctamente inicializados, y dibujamos\n"
                "directamente encima.\n");
#endif

    waterfall_init();
    touch_init();
    debug_print_hex32("RCU_PLLI2S tras touch_init", RCU_PLLI2S);

    debug_print("\n--- AIC3204: fase 1 (solo comunicacion I2C) ---\n");
    aic3204_init(AIC3204_ADDR_DEFAULT);
    if (!aic3204_probe_and_reset()) {
        debug_print("aic3204: probando con 0x19 (pin MODE a VDD)...\n");
        aic3204_init(0x19);
        if (!aic3204_probe_and_reset()) {
            aic3204_scan_bus();
        }
    }
    debug_print_hex32("RCU_PLLI2S tras aic3204", RCU_PLLI2S);

    debug_print("\n--- I2S1: fase 3 (relojes + DMA circular, tono de prueba) ---\n");
    gd32_i2s_init_master_48k();

    debug_print("\n--- AIC3204: fase 2 (reloj + ADC diferencial I/Q + power-up) ---\n");
    aic3204_phase2_init();

    debug_print("\n--- SDR: fase 4 (captura RX real + FFT + espectro/cascada) ---\n");
    fft_init();
    sdr_rx_init();

#if CALIB_HEIGHT_TEST
    calib_height_ruler_draw();
#else
    demo_screen_draw();
#endif

    debug_print("main: entrando en el bucle principal\n");

    while (1) {
        gpio_bit_toggle(GPIOA, GPIO_PIN_8);

#if !CALIB_HEIGHT_TEST
        sdr_spectrum_waterfall_tick();
        demo_touch_poll();
#endif

        g_fill_count++;

        if ((g_fill_count % 50) == 0) {
            debug_print_dec("waterfall ticks", g_fill_count);
#if !CALIB_HEIGHT_TEST
            debug_print_dec("PENIRQ nivel crudo (1=activo/bajo)", touch_is_pressed());
            debug_print_dec("EXTI2 disparos reales desde el arranque", touch_irq_count());
#endif
        }
    }
}

/*
 * Build de calibracion: dibuja una regla horizontal (marca + etiqueta de
 * texto con el valor de Y) cada 40px desde 0 hasta GFX_SCREEN_HEIGHT-1,
 * mas un borde exacto en (0,0,GFX_SCREEN_WIDTH-1,GFX_SCREEN_HEIGHT-1).
 * Fotografiar el panel y comparar: la ultima etiqueta que se lea
 * COMPLETA (no cortada) antes del borde inferior real del panel indica
 * la altura util de verdad. Si el borde inferior dibujado no llega a
 * verse, GFX_SCREEN_HEIGHT sigue siendo mayor que la altura fisica real.
 */
#if CALIB_HEIGHT_TEST
static void calib_height_ruler_draw(void)
{
    char label[8];
    uint16_t y;

    gfx_fill_screen(GFX_COLOR_BLACK);

    /* Borde exacto en los limites que asumimos ahora mismo */
    gfx_rect(0, 0, GFX_SCREEN_WIDTH, GFX_SCREEN_HEIGHT, GFX_COLOR_RED);

    for (y = 0; y < GFX_SCREEN_HEIGHT; y += 40) {
        uint8_t i = 0;
        uint16_t v = y;
        char tmp[8];
        uint8_t n = 0;

        /* itoa manual (sin sprintf, para no arrastrar mas newlib) */
        if (v == 0) {
            tmp[n++] = '0';
        } else {
            while (v > 0 && n < sizeof(tmp)) {
                tmp[n++] = (char)('0' + (v % 10));
                v /= 10;
            }
        }
        while (n > 0) {
            label[i++] = tmp[--n];
        }
        label[i] = '\0';

        gfx_hline(0, y, 20, GFX_COLOR_YELLOW);
        gfx_text(24, (uint16_t)((y >= 3) ? (y - 3) : 0), label,
                  GFX_COLOR_CYAN, GFX_COLOR_BLACK, 1);
    }
}
#endif /* CALIB_HEIGHT_TEST */

/*
 * Pantalla de demo fija: valida gfx.c/ui.c en hardware real (panel,
 * bus EXMC, orientacion) y sirve de plantilla para como registrar
 * widgets en un ui_screen_t. Se pinta una sola vez; el waterfall se va
 * actualizando aparte en demo_waterfall_tick().
 *
 * Layout (horizontal 800x480 - confirmado con hardware real, ver
 * comentario de GFX_SCREEN_WIDTH/HEIGHT en gfx.h):
 *   - Barra de titulo arriba (24px)
 *   - Marco de "espectro" (placeholder, aun sin datos de FFT reales)
 *   - Zona de waterfall (WATERFALL_ROWS filas de WATERFALL_WIDTH px)
 *   - Fila de botones de ejemplo abajo
 *
 * Las Y de cada zona se definen como constantes con enlazado interno
 * (static const, no macros) para que demo_waterfall_tick() use
 * exactamente el mismo valor de waterfall_y que demo_screen_draw(), sin
 * duplicar el calculo a mano.
 */
static const uint16_t DEMO_TITLE_H     = 24;
static const uint16_t DEMO_SPECTRUM_Y  = 24;
static const uint16_t DEMO_SPECTRUM_H  = 140;
static const uint16_t DEMO_WATERFALL_Y = 24 + 140 + 2;
static const uint16_t DEMO_BTN_Y       = 24 + 140 + 2 + WATERFALL_ROWS + 10;

/*
 * IMPORTANTE: estos widgets son estaticos (no locales de
 * demo_screen_draw()) a proposito. ui_screen_t solo guarda PUNTEROS a
 * ellos (para no duplicar datos ni depender de malloc), asi que tienen
 * que seguir vivos mientras exista la pantalla - si fueran variables de
 * pila de una funcion que ya ha retornado, ui_screen_touch() estaria
 * leyendo memoria de stack ya reutilizada por otra llamada. Esto es
 * justo el tipo de bug que no da error de compilacion pero corrompe
 * memoria silenciosamente en tiempo de ejecucion.
 */
static ui_screen_t s_demo_screen;
static ui_panel_t  s_title_panel;
static ui_panel_t  s_spectrum_panel;
static ui_panel_t  s_waterfall_panel;
static ui_button_t s_btn_menu;
static ui_button_t s_btn_tune;
static ui_button_t s_btn_mode;

/*
 * Callback de ejemplo compartido por los 3 botones de la demo. Cuando
 * exista el driver de tactil, cada lectura de (x,y,pressed) que llegue a
 * ui_screen_touch() puede terminar disparando esto en RELEASE (el
 * "click" real) - de momento no hay nada que lo invoque todavia, es solo
 * la plantilla de como conectar la logica de la app a los eventos de UI.
 */
static void demo_button_callback(void *widget, ui_event_t event, void *user_data)
{
    ui_button_t *btn = (ui_button_t *)widget;
    (void)user_data;

    if (event == UI_EVENT_RELEASE) {
        debug_print("boton pulsado: ");
        debug_print(btn->label);
        debug_print("\n");
    }
}

static void demo_screen_draw(void)
{
    const uint16_t btn_w = 120, btn_h = 44, btn_gap = 12;

    gfx_fill_screen(GFX_COLOR_BLACK);
    ui_screen_init(&s_demo_screen);

    s_title_panel = (ui_panel_t){0, 0, GFX_SCREEN_WIDTH, DEMO_TITLE_H,
                                  GFX_COLOR_DARKGRAY, GFX_COLOR_DARKGRAY};
    ui_screen_add_panel(&s_demo_screen, &s_title_panel);

    s_spectrum_panel = (ui_panel_t){0, DEMO_SPECTRUM_Y, GFX_SCREEN_WIDTH, DEMO_SPECTRUM_H,
                                     GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_spectrum_panel);

    s_waterfall_panel = (ui_panel_t){0, (uint16_t)(DEMO_WATERFALL_Y - 2), GFX_SCREEN_WIDTH,
                                      (uint16_t)(WATERFALL_ROWS + 4), GFX_COLOR_BLACK, GFX_COLOR_GRAY};
    ui_screen_add_panel(&s_demo_screen, &s_waterfall_panel);

    /* enabled=1 puesto explicitamente - si se omite, un ui_button_t
     * recien declarado queda con enabled=0 (inicializacion a cero de C)
     * y ui_screen_touch() lo ignorara siempre aunque se dibuje bien. */
    s_btn_menu = (ui_button_t){btn_gap, DEMO_BTN_Y, btn_w, btn_h, "MENU",
                                GFX_COLOR_WHITE, GFX_COLOR_BLUE, GFX_COLOR_WHITE,
                                1, 0, 1, demo_button_callback, NULL};
    s_btn_tune = (ui_button_t){(uint16_t)(btn_gap * 2 + btn_w), DEMO_BTN_Y, btn_w, btn_h, "TUNE",
                                GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, GFX_COLOR_WHITE,
                                1, 0, 1, demo_button_callback, NULL};
    s_btn_mode = (ui_button_t){(uint16_t)(btn_gap * 3 + btn_w * 2), DEMO_BTN_Y, btn_w, btn_h, "MODE",
                                GFX_COLOR_BLACK, GFX_COLOR_YELLOW, GFX_COLOR_BLACK,
                                1, 1, 1, demo_button_callback, NULL};
    ui_screen_add_button(&s_demo_screen, &s_btn_menu);
    ui_screen_add_button(&s_demo_screen, &s_btn_tune);
    ui_screen_add_button(&s_demo_screen, &s_btn_mode);

    ui_screen_draw(&s_demo_screen);

    /* Texto suelto que no necesita ser interactivo ni sobrevivir despues
     * de pintarse: se sigue pudiendo llamar a gfx_text()/ui_label_draw()
     * directo, sin pasar por el screen, para cosas que no cambian ni
     * reciben toques. */
    gfx_text(4, 4, "DEEPSDR - DEMO GFX/UI", GFX_COLOR_WHITE, GFX_COLOR_DARKGRAY, 2);
    gfx_text(4, (uint16_t)(DEMO_SPECTRUM_Y + 4), "ESPECTRO (PLACEHOLDER)",
              GFX_COLOR_GREEN, GFX_COLOR_BLACK, 1);
    gfx_line(0, (uint16_t)(DEMO_SPECTRUM_Y + DEMO_SPECTRUM_H - 20),
              GFX_SCREEN_WIDTH - 1, (uint16_t)(DEMO_SPECTRUM_Y + DEMO_SPECTRUM_H - 20),
              GFX_COLOR_DARKGRAY);
}

/*
 * Punto de enganche con touch.c, ya activo. touch_read() hace el trabajo
 * pesado (transacciones SPI bit-banged + promediado) SOLO cuando
 * touch_is_pressed() ya dio positivo (barato, un simple gpio_input_bit_get),
 * asi que llamarlo cada iteracion del bucle principal no es costoso
 * mientras no haya contacto.
 *
 * OJO CALIBRACION: sin haber llamado a touch_set_calibration(), touch.c
 * usa una correspondencia identidad (raw 0-4095 -> pantalla 0-800/0-480)
 * que casi seguro NO coincide con la orientacion/escala real del panel
 * resistivo. Sirve para validar que el pipeline entero funciona (que un
 * toque llega a mover/pulsar un boton), pero antes de dar esto por
 * terminado hay que calibrar: tocar las 4 esquinas conocidas del panel,
 * anotar los raw_x/raw_y que da touch_debug_raw() en cada una, y rellenar
 * un touch_calibration_t real via touch_set_calibration().
 */
static void demo_touch_poll(void)
{
    uint16_t x = 0, y = 0;
    uint8_t pressed = touch_read(&x, &y);

    ui_screen_touch(&s_demo_screen, x, y, pressed);

    if (touch_irq_pending()) {
        debug_print("touch: flanco PENIRQ (nuevo contacto) -> ");
        touch_debug_raw();
    }
}

/*
 * DIAGNOSTICO/CALIBRACION PENDIENTE (28/07/2026): SDR_DB_MIN/MAX son un
 * rango de trabajo INVENTADO, no calibrado - nuestra "dB" sale de una
 * aproximacion de log2 por bit-manipulation (ver fft.c), no de una
 * medida real referenciada. Sirve para que se vea algo razonable en
 * pantalla desde el primer momento, pero UNA VEZ que Jorge tenga una
 * señal real de referencia (tono conocido, ruido de fondo del AIC3204
 * sin señal, etc.) hay que reajustar estos dos numeros mirando que
 * valores de s_db salen realmente en la practica - no dar esta escala
 * por buena todavia.
 */
static const float SDR_DB_MIN = -10.0f;
static const float SDR_DB_MAX = 90.0f;

/* sdr_rx.h y fft.h definen sus tamanos de forma independiente - si
 * algun dia cambia uno sin el otro, mejor un error de compilacion claro
 * que un desbordamiento silencioso de s_rx_block. */
#if SDR_RX_BLOCK_SAMPLES != FFT_SIZE
#error "SDR_RX_BLOCK_SAMPLES (sdr_rx.h) y FFT_SIZE (fft.h) deben coincidir"
#endif

static int16_t s_rx_i[SDR_RX_BLOCK_SAMPLES];
static int16_t s_rx_q[SDR_RX_BLOCK_SAMPLES];
static float   s_db[FFT_BINS_USEFUL];

/*
 * VERIFICACION CANAL Q (28/07/2026, decimoquinta vuelta): I(IN2_L/R) ya
 * esta confirmado en hardware real por Jorge. Q(IN3_R/L) sigue siendo
 * una extrapolacion del registro (ver aic3204.c) sin verificar. Cambiar
 * este define a 1 para que el espectro/cascada muestren Q en vez de I -
 * si al inyectar algo conocido en IN3_R/IN3_L (y NADA en IN2) aparece
 * en pantalla igual que antes aparecia con IN2, confirma que el ruteo
 * de Q esta bien. Los min/max de AMBOS canales se imprimen siempre,
 * muestres el que muestres, para tener el dato numerico sin necesidad
 * de mirar la pantalla.
 */
#define SDR_SHOW_CHANNEL_Q   0   /* 0 = mostrar I (IN2), 1 = mostrar Q (IN3) */

/* debug_uart.h no trae una version con signo - los min/max de las
 * muestras I/Q son int16_t y pueden ser negativos, asi que un helper
 * minimo local en vez de tocar el modulo de UART para esto. */
static void debug_print_dec_signed(const char *label, int32_t val)
{
    if (val < 0) {
        debug_print(label);
        debug_print(" = -");
        debug_print_dec("", (uint32_t)(-val));
    } else {
        debug_print_dec(label, (uint32_t)val);
    }
}

/*
 * Reemplaza al antiguo demo_waterfall_tick() (gradiente sintetico) -
 * ver el comentario que quedaba en waterfall.h: "sustituir por datos
 * reales de FFT cuando lleguemos a las funciones de SDR". No bloqueante:
 * si sdr_rx_poll_block_iq() no tiene bloque nuevo todavia, no hace nada
 * este tick (el resto del bucle principal - touch, UI - sigue igual de
 * fluido).
 */
static void sdr_spectrum_waterfall_tick(void)
{
    static uint16_t line[WATERFALL_WIDTH];
    static uint32_t s_block_count = 0U;
    static uint32_t s_t_prev_block = 0U;
    static uint8_t  s_have_prev = 0U;
    uint32_t t_fft0, t_fft1, t_spec0, t_spec1, t_wf0, t_wf1, t_now;
    uint16_t x, n;
    int16_t i_min, i_max, q_min, q_max;
    const int16_t *show_block;

    if (sdr_rx_poll_block_iq(s_rx_i, s_rx_q) == 0U) {
        return;
    }

    /* min/max de AMBOS canales, cada bloque - es la prueba numerica de
     * si IN2 e IN3 estan viendo cosas distintas (senal real en cada
     * uno) o si Q esta "muerto" (min=max=0 o clavado en un valor fijo,
     * lo que delataria que el ruteo P1_R55/P1_R57 no esta llegando de
     * verdad al ADC derecho pese a que el registro acepto el ACK). */
    i_min = s_rx_i[0]; i_max = s_rx_i[0];
    q_min = s_rx_q[0]; q_max = s_rx_q[0];
    for (n = 1; n < SDR_RX_BLOCK_SAMPLES; n++) {
        if (s_rx_i[n] < i_min) { i_min = s_rx_i[n]; }
        if (s_rx_i[n] > i_max) { i_max = s_rx_i[n]; }
        if (s_rx_q[n] < q_min) { q_min = s_rx_q[n]; }
        if (s_rx_q[n] > q_max) { q_max = s_rx_q[n]; }
    }

#if SDR_SHOW_CHANNEL_Q
    show_block = s_rx_q;
#else
    show_block = s_rx_i;
#endif

    /* DIAGNOSTICO (28/07/2026): en vez de seguir optimizando a ciegas,
     * medimos con el DWT (mismo mecanismo que en gd32_i2s.c) cuanto
     * tarda cada tramo del pipeline, Y ADEMAS cuanto tiempo real pasa
     * entre bloque y bloque (periodo observado) - si el periodo total
     * es mucho mayor que fft+spectrum+waterfall sumados, el cuello de
     * botella esta en OTRA parte del bucle principal (touch, systick,
     * etc), no en este tick. Se imprime solo 1 de cada 20 bloques para
     * no saturar el UART ni afectar el tiempo medido con el propio
     * print. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    t_now = DWT->CYCCNT;

    t_fft0 = DWT->CYCCNT;
    fft_compute_db(show_block, s_db);
    t_fft1 = DWT->CYCCNT;

    t_spec0 = DWT->CYCCNT;

    /* espectro instantaneo, dentro del panel ya reservado en la UI
     * (ver DEMO_SPECTRUM_Y/H) - se deja un margen para no pisar el
     * texto "ESPECTRO" ni la linea separadora de abajo */
    spectrum_draw(s_db, FFT_BINS_USEFUL,
                  2, (uint16_t)(DEMO_SPECTRUM_Y + 18),
                  (uint16_t)(GFX_SCREEN_WIDTH - 4),
                  (uint16_t)(DEMO_SPECTRUM_H - 18 - 20 - 2),
                  SDR_DB_MIN, SDR_DB_MAX);
    t_spec1 = DWT->CYCCNT;

    t_wf0 = DWT->CYCCNT;
    /* cascada: una fila de WATERFALL_WIDTH px, cada columna mapeada
     * (vecino mas cercano) a su bin de FFT correspondiente */
    for (x = 0; x < WATERFALL_WIDTH; x++) {
        uint32_t bin = ((uint32_t)x * FFT_BINS_USEFUL) / WATERFALL_WIDTH;
        line[x] = spectrum_colormap(s_db[bin], SDR_DB_MIN, SDR_DB_MAX);
    }
    waterfall_push_line(line);
    waterfall_blit(0, DEMO_WATERFALL_Y);
    t_wf1 = DWT->CYCCNT;

    s_block_count++;
    if ((s_block_count % 20U) == 0U) {
        uint32_t fft_us  = (uint32_t)(((uint64_t)(t_fft1 - t_fft0)) * 1000000U / SystemCoreClock);
        uint32_t spec_us = (uint32_t)(((uint64_t)(t_spec1 - t_spec0)) * 1000000U / SystemCoreClock);
        uint32_t wf_us   = (uint32_t)(((uint64_t)(t_wf1 - t_wf0)) * 1000000U / SystemCoreClock);
        debug_print_dec("sdr_tick: fft (us)", fft_us);
        debug_print_dec("sdr_tick: spectrum_draw (us)", spec_us);
        debug_print_dec("sdr_tick: waterfall push+blit (us)", wf_us);
        debug_print_dec("sdr_tick: TOTAL tramo medido (us)", fft_us + spec_us + wf_us);
        if (s_have_prev) {
            uint32_t period_us = (uint32_t)(((uint64_t)(t_now - s_t_prev_block)) * 1000000U / SystemCoreClock);
            debug_print_dec("sdr_tick: PERIODO real entre bloques (us) - compara con el "
                            "TOTAL de arriba", period_us);
        }
        /* VERIFICACION CANAL Q: min/max de cada canal, cada 20 bloques.
         * Si Q(IN3) esta muerto (min=max, o clavado en un valor fijo
         * mientras I(IN2) se mueve), es la señal de que el ruteo
         * extrapolado de P1_R55/P1_R57 no esta llegando de verdad al
         * ADC derecho. Inyecta algo conocido en IN3_R/IN3_L (con IN2
         * quieto) y comprueba que estos numeros SI cambian. */
        debug_print_dec_signed("sdr_tick: I(IN2) min", i_min);
        debug_print_dec_signed("sdr_tick: I(IN2) max", i_max);
        debug_print_dec_signed("sdr_tick: Q(IN3) min", q_min);
        debug_print_dec_signed("sdr_tick: Q(IN3) max", q_max);
    }
    s_t_prev_block = t_now;
    s_have_prev = 1U;
}

static void led_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
}

static void systick_delay_init(void)
{
    /* SysTick a 1ms real, basado en SystemCoreClock (actualizado por
     * SystemInit()/system_gd32f4xx.c). Antes esto estaba vacio y
     * rm68120_exmc.c usaba un bucle sin calibrar para los delays -
     * sospechoso de dar tiempos de espera incorrectos en la secuencia
     * de arranque de tensiones del panel, que es sensible a timing. */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        while (1) {
            /* fallo configurando SysTick, no deberia pasar */
        }
    }
}

void SysTick_Handler(void)
{
    g_msticks++;
}

/*
 * El HardFault_Handler por defecto (Default_Handler, definido debil en
 * el startup) es un bucle infinito MUDO - exactamente el sintoma que
 * describiste (se para en seco sin avisar por UART). Esta version SI
 * avisa, para poder distinguir un HardFault real de cualquier otro tipo
 * de cuelgue (bucle infinito en codigo propio, por ejemplo) con solo
 * mirar si aparece este mensaje.
 *
 * No decodifica el registro CFSR/HFSR (para eso hace falta un depurador
 * conectado y mirar el stack frame) - de momento es solo la señal de
 * "hemos entrado aqui", suficiente para confirmar o descartar la hipotesis.
 */
void HardFault_Handler(void)
{
    debug_print("\n*** HARDFAULT_HANDLER: ha ocurrido un fallo de bus/acceso ***\n");
    while (1) {
        __NOP();
    }
}
