#include "aic3204.h"
#include "i2c_bitbang.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

#define AIC3204_NRST_PORT GPIOB
#define AIC3204_NRST_PIN  GPIO_PIN_11

static uint8_t s_addr = AIC3204_ADDR_DEFAULT;

/* extern del contador de ms de main.c, mismo mecanismo que ya usan
 * rm68120_exmc.c y touch.c, para los tiempos de reset. */
extern volatile uint32_t g_msticks;

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_msticks;
    while ((g_msticks - start) < ms) {
        __NOP();
    }
}

static void print_addr(uint8_t addr)
{
    debug_print_hex32("aic3204: dispositivo I2C encontrado en", addr);
}

/*
 * Saca al AIC3204 de reset por hardware antes de cualquier transaccion
 * I2C. Sin esto, el chip puede quedarse en reset indefinidamente (segun
 * como este cableado el pull del propio nRESET) y nunca responder de
 * forma fiable en el bus - encaja con el sintoma visto (bus I2C en
 * estado raro, todas las direcciones "respondiendo").
 *
 * Tiempos usados aqui son conservadores (10ms mantenido en reset, 10ms
 * de espera tras liberarlo) porque no se ha verificado el valor exacto
 * del datasheet contra este build en concreto - si hiciera falta mas
 * margen, este es el sitio a tocar.
 */
static void aic3204_hw_reset(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(AIC3204_NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AIC3204_NRST_PIN);
    gpio_output_options_set(AIC3204_NRST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, AIC3204_NRST_PIN);

    gpio_bit_reset(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* mantener en reset */
    delay_ms(10);
    gpio_bit_set(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* liberar */
    delay_ms(10);

    debug_print("aic3204: nRESET (PB11) liberado\n");
}

void aic3204_init(uint8_t i2c_addr)
{
    s_addr = i2c_addr;
    aic3204_hw_reset();
    i2c_bitbang_init();
}

uint8_t aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = 0x00; /* Page Control Register, siempre 0x00 en toda pagina */
    buf[1] = page;
    if (!i2c_write(s_addr, buf, 2)) {
        return 0;
    }

    buf[0] = reg;
    buf[1] = value;
    return i2c_write(s_addr, buf, 2);
}

uint8_t aic3204_read_reg(uint8_t page, uint8_t reg, uint8_t *value)
{
    uint8_t buf[2];

    buf[0] = 0x00;
    buf[1] = page;
    if (!i2c_write(s_addr, buf, 2)) {
        return 0;
    }

    /* Escritura de "registro a leer" sin repeated-start real entre medias:
     * i2c_write/i2c_read hacen START..STOP completos cada uno. El AIC3204
     * admite esto (repeated-start no es obligatorio, a diferencia de
     * algunos sensores). Si en hardware real diera problemas, cambiar a
     * un repeated-start real seria el primer sospechoso. */
    buf[0] = reg;
    if (!i2c_write(s_addr, buf, 1)) {
        return 0;
    }

    return i2c_read(s_addr, value, 1);
}

/* Lectura cruda de un registro SIN seleccionar pagina antes (a
 * diferencia de aic3204_read_reg, pensada para uso normal). Solo sirve
 * para verificar que un valor escrito antes (p.ej. la pagina activa)
 * persiste de verdad, sin que la propia lectura lo enmascare. */
static uint8_t read_reg_raw(uint8_t reg, uint8_t *value)
{
    uint8_t r = reg;

    if (!i2c_write(s_addr, &r, 1)) {
        return 0;
    }
    return i2c_read(s_addr, value, 1);
}

uint8_t aic3204_probe_and_reset(void)
{
    uint8_t buf[2];
    uint8_t readback;
    uint8_t ok = 1;

    debug_print("aic3204: bucle-cerrado del Page Control Register...\n");

    /* Selecciona pagina 1 (escritura directa, sin pasar por
     * aic3204_write_reg para no confundir "pagina" con "registro
     * destino dentro de esa pagina" - aqui ambos son 0x00). */
    buf[0] = 0x00;
    buf[1] = 1;
    if (!i2c_write(s_addr, buf, 2)) {
        debug_print("aic3204: NACK escribiendo pagina 1\n");
        ok = 0;
    } else if (!read_reg_raw(0x00, &readback) || readback != 1) {
        debug_print_dec("aic3204: pagina releida tras seleccionar 1", readback);
        ok = 0;
    } else {
        debug_print("aic3204: pagina 1 confirmada\n");
    }

    buf[0] = 0x00;
    buf[1] = 0;
    if (!i2c_write(s_addr, buf, 2)) {
        debug_print("aic3204: NACK escribiendo pagina 0\n");
        ok = 0;
    } else if (!read_reg_raw(0x00, &readback) || readback != 0) {
        debug_print_dec("aic3204: pagina releida tras seleccionar 0", readback);
        ok = 0;
    } else {
        debug_print("aic3204: pagina 0 confirmada\n");
    }

    if (!ok) {
        debug_print("aic3204: bucle-cerrado FALLIDO - revisar direccion I2C "
                     "con aic3204_scan_bus(), alimentacion del codec, o wiring SCL/SDA\n");
        return 0;
    }

    debug_print("aic3204: bucle-cerrado OK, enlace I2C confirmado\n");

    /* Software reset: pagina 0, registro 0x01, bit 0 = 1. Autolimpiante,
     * no hay registro que releer para confirmar - nos fiamos del
     * bucle-cerrado ya validado arriba. */
    if (!aic3204_write_reg(0, 0x01, 0x01)) {
        debug_print("aic3204: escritura de software reset fallo (sin ACK)\n");
        return 0;
    }
    debug_print("aic3204: software reset enviado\n");

    return 1;
}

void aic3204_scan_bus(void)
{
    debug_print("aic3204: escaneando bus I2C (0x08-0x77)...\n");
    i2c_scan(0x08, 0x77, print_addr);
    debug_print("aic3204: escaneo completado\n");
}

/*
 * FASE 2 (28/07/2026) - CONFIRMADO (decimotercera vuelta): Jorge midio
 * con el osciloscopio MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz de
 * verdad (la relacion fija 256x/32x nativa de MCKOUT, activada con
 * Fs=48kHz - ver aviso en gd32_i2s.c). Ya NO es una prediccion: estos
 * valores estan calculados sobre el MCLK real medido, no sobre una
 * formula sin verificar.
 *   CODEC_CLKIN = NADC x MADC x AOSR x ADC_FS
 *   12.288.000 = NADC x MADC x 128 x 48.000  =>  NADC x MADC = 2
 * Filter A (recomendado por TI para 48kHz alto rendimiento, AOSR=128),
 * processing block PRB_R1 (Stereo, RC=6): MADC*AOSR/32=2*128/32=8>=6 OK.
 */
#define AIC3204_NADC   1U
#define AIC3204_MADC   2U
#define AIC3204_AOSR   128U  /* Filter A, para 48kHz (recomendado por TI) */
/* DAC: mismo CODEC_CLKIN compartido, Filter A tambien para 48kHz
 * (DOSR multiplo de 8; 128*48000=6.144MHz, dentro del rango valido
 * 2.8-6.2MHz que exige el manual). NDAC*MDAC*128*48000=12.288.000
 * => NDAC*MDAC=2, igual que el lado ADC. */
#define AIC3204_NDAC   1U
#define AIC3204_MDAC   2U
#define AIC3204_DOSR   128U

static void wr(uint8_t page, uint8_t reg, uint8_t value, const char *what)
{
    if (!aic3204_write_reg(page, reg, value)) {
        debug_print("aic3204: *** escritura SIN ACK: ");
        debug_print(what);
        debug_print(" ***\n");
    }
}

void aic3204_phase2_init(void)
{
    debug_print("\naic3204: fase 2 - reloj + ADC diferencial I/Q + power-up\n");

    /* --- Reloj: MCLK directo a CODEC_CLKIN, SIN PLL --- */
    wr(0, 0x04, 0x00, "R4 CODEC_CLKIN=MCLK, sin PLL");
    wr(0, 0x05, 0x00, "R5 PLL power down");

    /* --- Interfaz de audio: I2S, 16 bits, BCLK/WCLK como ENTRADA
     * (AIC3204 esclavo - el GD32 es el maestro I2S) --- */
    wr(0, 0x1B, 0x00, "R27 I2S 16bit, BCLK/WCLK entrada (esclavo)");
    wr(0, 0x1C, 0x00, "R28 data offset = 0");

    /* --- Dividers ADC (NADC/MADC) y AOSR --- */
    wr(0, 0x12, 0x80 | AIC3204_NADC, "R18 NADC power+valor");
    wr(0, 0x13, 0x80 | AIC3204_MADC, "R19 MADC power+valor");
    wr(0, 0x14, AIC3204_AOSR,        "R20 AOSR");

    /* --- Dividers DAC (NDAC/MDAC) y DOSR - reloj compartido con ADC,
     * se dejan encendidos aunque no usemos la salida de audio --- */
    wr(0, 0x0B, 0x80 | AIC3204_NDAC, "R11 NDAC power+valor");
    wr(0, 0x0C, 0x80 | AIC3204_MDAC, "R12 MDAC power+valor");
    wr(0, 0x0D, 0x00,                "R13 DOSR hi_byte");
    wr(0, 0x0E, AIC3204_DOSR,        "R14 DOSR lo_byte");

    /* --- Processing block: PRB_R1 (Stereo, Filter A, sin biquads -
     * RC=6, valido con MADC=2/AOSR=128 ya que MADC*AOSR/32=8>=6) --- */
    wr(0, 0x3D, 1, "R61 ADC processing block PRB_R1");

    /* --- Page 1: alimentacion analogica --- */
    wr(1, 0x01, 0x08, "P1R1 deshabilitar AVDD-desde-DVDD");
    wr(1, 0x02, 0x01, "P1R2 bloques analogicos + LDO ON");

    /* --- ADC: ruteo diferencial ---
     * I (izquierdo) = IN2_L(+) / IN2_R(-)  -- CONFIRMADO en hardware
     *   real por Jorge (probado con exito electrico anteriormente).
     * Q (derecho)   = IN3_R(+) / IN3_L(-)  -- arquitectura confirmada
     *   contra SLAA557 (Figura 2-1: "Right Channel differential:
     *   IN3_R and IN3_L"), pero el VALOR EXACTO de registro
     *   (0b00000100 = 10kOhm en el 3er campo de 2 bits) es una
     *   extrapolacion del patron de tu propio driver, no algo leido
     *   directamente de una tabla de bits - verificar con el AIC3204
     *   Control Software o releyendo el registro si hay dudas.
     */
    wr(1, 0x33, 0x00, "P1R51 MICBIAS off (no hace falta, entrada diferencial)");
    wr(1, 0x34, 0x10, "P1R52 IN2L -> PGA_L positivo, 10k (I, confirmado)");
    wr(1, 0x36, 0x10, "P1R54 IN2R -> PGA_L negativo, 10k (I, confirmado)");
    wr(1, 0x37, 0x04, "P1R55 IN3R -> PGA_R positivo, 10k (Q, EXTRAPOLADO)");
    wr(1, 0x39, 0x04, "P1R57 IN3L -> PGA_R negativo, 10k (Q, EXTRAPOLADO)");

    /* Ganancia PGA: 0dB para empezar (bit7=0 => sin mute, bits[6:0]=0dB).
     * Subir mas adelante segun nivel de señal real. */
    wr(1, 0x3B, 0x00, "P1R59 MIC_PGA_L 0dB, sin mute");
    wr(1, 0x3C, 0x00, "P1R60 MIC_PGA_R 0dB, sin mute");

    /* --- Power-up del ADC (pagina 0) --- */
    wr(0, 0x51, 0xC0, "R81 power-up ADC izq+dcha");
    wr(0, 0x52, 0x00, "R82 ADC sin mute, ganancia fina 0dB");

    debug_print("aic3204: fase 2 completada - I(IN2)=izq confirmado, "
                "Q(IN3)=dcha EXTRAPOLADO (verificar), NADC=1/MADC=2/AOSR=128 sobre "
                "MCLK=12.288MHz y Fs=48kHz, ambos CONFIRMADOS con osciloscopio\n");
}
