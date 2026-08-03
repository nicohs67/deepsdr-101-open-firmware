#include "aic3204.h"
#include "i2c_bitbang.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

#define AIC3204_NRST_PORT GPIOB
#define AIC3204_NRST_PIN  GPIO_PIN_11

static uint8_t s_addr = AIC3204_ADDR_DEFAULT;

/* ms tick counter from main.c, same mechanism already used by
 * rm68120_exmc.c and touch.c, for reset timing. */
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
    debug_print_hex32("aic3204: I2C device found at", addr);
}

/*
 * Releases the AIC3204 from hardware reset before any I2C
 * transaction. Without this the chip can stay in reset indefinitely
 * (depending on how nRESET is pulled) and never respond reliably on
 * the bus.
 *
 * The timing used here (10ms held in reset, 10ms after release) is
 * conservative and has not been trimmed against the exact datasheet
 * minimums for this specific board - if more margin is ever needed,
 * this is the place to adjust.
 *
 * Extended from an initial 10ms to 50ms per side, matching the more
 * generous timing (200ms) used by an earlier known-working driver for
 * this same board - cheap extra margin while chasing an RX capture
 * bug that could plausibly be a power-up/reset sequencing issue.
 */
static void aic3204_hw_reset(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(AIC3204_NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AIC3204_NRST_PIN);
    gpio_output_options_set(AIC3204_NRST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, AIC3204_NRST_PIN);

    gpio_bit_reset(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* hold in reset */
    delay_ms(50);
    gpio_bit_set(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* release */
    delay_ms(50);

    debug_print("aic3204: nRESET (PB11) released\n");
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

    buf[0] = 0x00; /* Page Control Register, always 0x00 on every page */
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

    /* "Register to read" write with no real repeated-start in
     * between: i2c_write/i2c_read each do a full START..STOP. The
     * AIC3204 accepts this (a real repeated-start is not mandatory,
     * unlike some sensors). If this ever causes issues on real
     * hardware, switching to a true repeated-start would be the first
     * thing to try. */
    buf[0] = reg;
    if (!i2c_write(s_addr, buf, 1)) {
        return 0;
    }

    return i2c_read(s_addr, value, 1);
}

/* Raw register read WITHOUT selecting a page first (unlike
 * aic3204_read_reg, meant for normal use). Only useful to verify that
 * a previously written value (e.g. the active page) actually
 * persisted, without the read itself masking anything. */
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

    debug_print("aic3204: Page Control Register closed-loop test...\n");

    /* Select page 1 (direct write, not via aic3204_write_reg, to avoid
     * confusing "page" with "target register within that page" -
     * here both happen to be 0x00). */
    buf[0] = 0x00;
    buf[1] = 1;
    if (!i2c_write(s_addr, buf, 2)) {
        debug_print("aic3204: NACK writing page 1\n");
        ok = 0;
    } else if (!read_reg_raw(0x00, &readback) || readback != 1) {
        debug_print_dec("aic3204: page read back after selecting 1", readback);
        ok = 0;
    } else {
        debug_print("aic3204: page 1 confirmed\n");
    }

    buf[0] = 0x00;
    buf[1] = 0;
    if (!i2c_write(s_addr, buf, 2)) {
        debug_print("aic3204: NACK writing page 0\n");
        ok = 0;
    } else if (!read_reg_raw(0x00, &readback) || readback != 0) {
        debug_print_dec("aic3204: page read back after selecting 0", readback);
        ok = 0;
    } else {
        debug_print("aic3204: page 0 confirmed\n");
    }

    if (!ok) {
        debug_print("aic3204: closed-loop test FAILED - check the I2C address with "
                     "aic3204_scan_bus(), codec power, or the SCL/SDA wiring\n");
        return 0;
    }

    debug_print("aic3204: closed-loop OK, I2C link confirmed\n");

    /* Software reset: page 0, register 0x01, bit 0 = 1. Self-clearing,
     * no register to read back for confirmation - relying on the
     * closed-loop test above having already validated the link. */
    if (!aic3204_write_reg(0, 0x01, 0x01)) {
        debug_print("aic3204: software reset write failed (no ACK)\n");
        return 0;
    }
    debug_print("aic3204: software reset sent\n");

    return 1;
}

void aic3204_scan_bus(void)
{
    debug_print("aic3204: scanning I2C bus (0x08-0x77)...\n");
    i2c_scan(0x08, 0x77, print_addr);
    debug_print("aic3204: scan complete\n");
}

/*
 * Phase 2, rewritten (28/07/2026) from a REAL I2C bus capture of the
 * original, known-working firmware's AIC3204 configuration - not
 * derived, not extrapolated, not guessed. Every write below is a
 * literal (page, register, value) triple exactly as captured on the
 * real bus, in the exact order it was sent, address 0x18 only (the
 * capture also showed unrelated traffic to 0x60 - the MS5351 LO chip
 * on the same bus - filtered out here).
 *
 * The clock chain this reveals is genuinely different from what this
 * driver assumed for a long time: the AIC3204's own PLL is enabled,
 * clocked from BCLK (not from a separate MCLK signal). Confirmed by
 * exact arithmetic against oscilloscope measurements: BCLK=6.144MHz,
 * PLL J=14 (D=0) -> PLL_CLK = 6.144MHz x 14 = 86.016MHz = CODEC_CLKIN.
 * Both divider chains reduce this to exactly Fs=192kHz:
 *   ADC: NADC(1) x MADC(7) x AOSR(64)  = 448 -> 86.016MHz/448 = 192kHz
 *   DAC: NDAC(2) x MDAC(7) x DOSR(32)  = 448 -> 86.016MHz/448 = 192kHz
 * This is why gd32_i2s.c no longer configures an MCLK pin at all - the
 * codec doesn't need one for this design.
 *
 * The ADC input routing (P1_R52/54/55/57) and ADC power-up (R81/R82)
 * already matched this capture exactly even before this rewrite - so
 * those were never the bug. Registers 8/9/44/46 (pages, not
 * addresses) further down are DRC/biquad filter coefficients (audio
 * processing, not core bring-up) - included for completeness since
 * they're genuinely part of the real sequence, not because their
 * exact function has been analyzed here.
 */
static void wr(uint8_t page, uint8_t reg, uint8_t value, const char *what)
{
    if (!aic3204_write_reg(page, reg, value)) {
        debug_print("aic3204: *** write with NO ACK: ");
        debug_print(what);
        debug_print(" ***\n");
    }
}

void aic3204_phase2_init(void)
{
    debug_print("\naic3204: phase 2 - full sequence ported from a real I2C capture\n");

    /* --- Software reset (page 0) --- */
    wr(0, 0x00, 0x00, "select page 0");
    wr(0, 0x01, 0x01, "software reset");

    /* --- Clock/PLL: CODEC_CLKIN sourced via PLL from BCLK, PLL ON,
     * J=14/D=0, giving CODEC_CLKIN=86.016MHz from BCLK=6.144MHz --- */
#if AIC3204_TEST_LOOPBACK
    wr(0, 0x04, 0x43, "R4 CODEC_CLKIN mux (captured)");
    wr(0, 0x05, 0x94, "R5 PLL on + P/R (captured)");
    wr(0, 0x06, 0x0E, "R6 PLL J=14 (captured)");
    wr(0, 0x07, 0x00, "R7 PLL D hi (captured)");
    wr(0, 0x08, 0x00, "R8 PLL D lo (captured)");
    wr(0, 0x0B, 0x82, "R11 NDAC=2, power on (captured)");
    wr(0, 0x0C, 0x87, "R12 MDAC=7, power on (captured)");
    wr(0, 0x0D, 0x00, "R13 DOSR hi (captured)");
    wr(0, 0x0E, 0x20, "R14 DOSR=32 lo (captured)");
    /* bit D5 (0x20) = Audio Bus Loopback, on top of the captured 0x11 */
    wr(0, 0x3C, 0x11 | 0x20, "R60(pg0) captured value + AUDIO BUS LOOPBACK (TEST)");
    debug_print("aic3204: *** LOOPBACK TEST MODE ENABLED - RX should mirror TX, ADC/DAC "
                "bypassed. Remember to feed a real TX pattern, not silence, and to set "
                "AIC3204_TEST_LOOPBACK back to 0 afterwards ***\n");
#else
    wr(0, 0x04, 0x43, "R4 CODEC_CLKIN mux (captured)");
    wr(0, 0x05, 0x94, "R5 PLL on + P/R (captured)");
    wr(0, 0x06, 0x0E, "R6 PLL J=14 (captured)");
    wr(0, 0x07, 0x00, "R7 PLL D hi (captured)");
    wr(0, 0x08, 0x00, "R8 PLL D lo (captured)");
    wr(0, 0x0B, 0x82, "R11 NDAC=2, power on (captured)");
    wr(0, 0x0C, 0x87, "R12 MDAC=7, power on (captured)");
    wr(0, 0x0D, 0x00, "R13 DOSR hi (captured)");
    wr(0, 0x0E, 0x20, "R14 DOSR=32 lo (captured)");
    wr(0, 0x3C, 0x11, "R60(pg0) (captured)");
#endif
    wr(0, 0x1B, 0x0C, "R27 I2S format (captured)");
    wr(0, 0x1E, 0x87, "R30 (captured)");
    wr(0, 0x25, 0xEE, "R37 (captured)");
    wr(0, 0x12, 0x81, "R18 NADC=1, power on (captured)");
    wr(0, 0x13, 0x87, "R19 MADC=7, power on (captured)");
    wr(0, 0x14, 0x40, "R20 AOSR=64 (captured)");
    wr(0, 0x3D, 0x01, "R61 ADC processing block PRB_R1 (captured)");

    /* --- Page 1: analog power + HP/LO bias config --- */
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x01, 0x08, "P1R1 disable AVDD-from-DVDD (captured)");
    wr(1, 0x02, 0x01, "P1R2 analog blocks + LDO ON (captured)");
    wr(1, 0x7B, 0x01, "P1R123 (captured)");
    wr(1, 0x14, 0x25, "P1R20 (captured)");
    wr(1, 0x0A, 0x3B, "P1R10 (captured)");
    wr(1, 0x0C, 0x08, "P1R12 (captured)");
    wr(1, 0x0D, 0x08, "P1R13 (captured)");
    wr(1, 0x0E, 0x08, "P1R14 (captured)");
    wr(1, 0x03, 0x00, "P1R3 (captured)");
    wr(1, 0x04, 0x00, "P1R4 (captured)");
    wr(1, 0x10, 0x00, "P1R16 HPL gain (captured)");
    wr(1, 0x11, 0x00, "P1R17 HPR gain (captured)");
    wr(1, 0x12, 0x00, "P1R18 (captured)");
    wr(1, 0x09, 0x38, "P1R9 (captured)");
    wr(1, 0x3D, 0x00, "P1R61 (captured)");
    wr(1, 0x47, 0x32, "P1R71 (captured)");
    wr(1, 0x7B, 0x01, "P1R123 again (captured)");

    /*
    Lo que significa registro por registro
    Canal Izquierdo (Left ADC):
    0x34 = 0x10 → IN2L entra al terminal positivo del MICPGA izquierdo con resistencia de 10 kΩ 
    0x36 = 0x10 → IN2R entra al terminal negativo del MICPGA izquierdo con resistencia de 10 kΩ 
    Esto configura el canal izquierdo como diferencial: la señal se toma entre IN2L (+) e IN2R (−), no contra common-mode.
    Canal Derecho (Right ADC):
    0x37 = 0x04 → IN3R entra al terminal positivo del MICPGA derecho con 10 kΩ 
    0x39 = 0x04 → IN3R entra al terminal negativo del MICPGA derecho con 10 kΩ 
    Esto conecta IN3R a ambos terminales (+ y −) del PGA derecho, lo cual anularía la señal (ganancia diferencial ≈ 0). 

    Ganancia y Mute:
    0x3B = 0x00 y 0x3C = 0x00 → Ambos MICPGA están desmuteados con ganancia de 0 dB 
    MICBIAS:
    0x33 = 0x60 → El bias de micrófono está activo a 2.0 V, alimentado desde AVdd (no desde LDOIN) 
    */

    wr(1, 0x34, 0x10, "P1R52 IN2L -> LADC_P, 40k (captured)");
    wr(1, 0x36, 0x10, "P1R54 (captured)");
    wr(1, 0x37, 0x04, "P1R55 (captured)");
    wr(1, 0x39, 0x04, "P1R57 (captured)");
    wr(1, 0x3B, 0x00, "P1R59 MIC_PGA_L (captured)");
    wr(1, 0x3C, 0x00, "P1R60 MIC_PGA_R (captured)");
    wr(1, 0x33, 0x60, "P1R51 MICBIAS level (captured - NOT off, unlike the earlier "
                      "single-ended baseline which used 0x00 here)");

    /* --- Pages 8/44: DRC/biquad processing coefficients (page 8 =
     * ADC filter bank, page 44 = another coefficient bank) --- */
    wr(8,  0x00, 0x08, "select page 8");
    wr(8,  0x01, 0x04, "P8R1 (captured)");
    wr(44, 0x00, 0x2C, "select page 44");
    wr(44, 0x01, 0x04, "P44R1 (captured)");

    /* --- Page 0: DAC power/routing + ADC power-up --- */
    wr(0, 0x00, 0x00, "select page 0");
    wr(0, 0x3F, 0xD6, "R63 DAC power/routing (captured)");
    wr(0, 0x40, 0x00, "R64 (captured)");
    wr(0, 0x41, 0x0C, "R65 DAC L volume (captured)");
    wr(0, 0x42, 0x0C, "R66 DAC R volume (captured)");
    wr(0, 0x51, 0xC0, "R81 power-up left+right ADC (captured, matches proven baseline)");
    wr(0, 0x52, 0x00, "R82 ADC unmuted, 0dB fine gain (captured, matches proven baseline)");
    wr(0, 0x43, 0x93, "R67 (captured)");

    /* --- Short page1/page0 dance adjusting HP driver gain a few
     * times (captured exactly as-is - looks like the original
     * firmware's own gain ramp-up sequence for the headphone path) --- */
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x12, 0x3A, "P1R18 (captured)");
    wr(0, 0x00, 0x00, "select page 0");
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x10, 0x3A, "P1R16 HPL gain step (captured)");
    wr(1, 0x11, 0x3A, "P1R17 HPR gain step (captured)");
    wr(0, 0x00, 0x00, "select page 0");
    wr(0, 0x00, 0x00, "select page 0 (captured - repeated in the original)");
    wr(0, 0x41, 0x00, "R65 DAC L volume step (captured)");
    wr(0, 0x42, 0x00, "R66 DAC R volume step (captured)");
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x3B, 0x28, "P1R59 MIC_PGA_L step (captured)");
    wr(1, 0x3C, 0x28, "P1R60 MIC_PGA_R step (captured)");
    wr(0, 0x00, 0x00, "select page 0");
    wr(0, 0x00, 0x00, "select page 0 (captured - repeated in the original)");
    wr(0, 0x53, 0x00, "R83 (captured)");
    wr(0, 0x54, 0x00, "R84 (captured)");
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x3B, 0x28, "P1R59 (captured, repeated)");
    wr(1, 0x3C, 0x28, "P1R60 (captured, repeated)");
    wr(0, 0x00, 0x00, "select page 0");

    /* --- Page 8/9: DRC/biquad coefficient banks (ADC filter) --- */
    wr(8, 0x00, 0x08, "select page 8");
    wr(8, 0x18, 0x7F, "P8R24 biquad coeff (captured)");
    wr(8, 0x19, 0xFA, "P8R25 biquad coeff (captured)");
    wr(8, 0x1A, 0xDA, "P8R26 biquad coeff (captured)");
    wr(8, 0x1B, 0x00, "P8R27 biquad coeff (captured)");
    wr(8, 0x1C, 0x80, "P8R28 biquad coeff (captured)");
    wr(8, 0x1D, 0x05, "P8R29 biquad coeff (captured)");
    wr(8, 0x1E, 0x26, "P8R30 biquad coeff (captured)");
    wr(8, 0x1F, 0x00, "P8R31 biquad coeff (captured)");
    wr(8, 0x20, 0x7F, "P8R32 biquad coeff (captured)");
    wr(8, 0x21, 0xF5, "P8R33 biquad coeff (captured)");
    wr(8, 0x22, 0xB5, "P8R34 biquad coeff (captured)");
    wr(8, 0x23, 0x00, "P8R35 biquad coeff (captured)");
    wr(9, 0x00, 0x09, "select page 9");
    wr(9, 0x20, 0x7F, "P9R32 biquad coeff (captured)");
    wr(9, 0x21, 0xFA, "P9R33 biquad coeff (captured)");
    wr(9, 0x22, 0xDA, "P9R34 biquad coeff (captured)");
    wr(9, 0x23, 0x00, "P9R35 biquad coeff (captured)");
    wr(9, 0x24, 0x80, "P9R36 biquad coeff (captured)");
    wr(9, 0x25, 0x05, "P9R37 biquad coeff (captured)");
    wr(9, 0x26, 0x26, "P9R38 biquad coeff (captured)");
    wr(9, 0x27, 0x00, "P9R39 biquad coeff (captured)");
    wr(9, 0x28, 0x7F, "P9R40 biquad coeff (captured)");
    wr(9, 0x29, 0xF5, "P9R41 biquad coeff (captured)");
    wr(9, 0x2A, 0xB5, "P9R42 biquad coeff (captured)");
    wr(9, 0x2B, 0x00, "P9R43 biquad coeff (captured)");
    wr(8, 0x00, 0x08, "select page 8");
    wr(8, 0x01, 0x05, "P8R1 (captured, updated)");
    wr(0, 0x00, 0x00, "select page 0");

    /* --- Page 46: another DRC/biquad coefficient bank --- */
    wr(46, 0x00, 0x2E, "select page 46");
    wr(46, 0x1C, 0x04, "P46R28 biquad coeff (captured)");
    wr(46, 0x1D, 0x0E, "P46R29 biquad coeff (captured)");
    wr(46, 0x1E, 0xB5, "P46R30 biquad coeff (captured)");
    wr(46, 0x1F, 0x00, "P46R31 biquad coeff (captured)");
    wr(46, 0x20, 0x04, "P46R32 biquad coeff (captured)");
    wr(46, 0x21, 0x0E, "P46R33 biquad coeff (captured)");
    wr(46, 0x22, 0xB5, "P46R34 biquad coeff (captured)");
    wr(46, 0x23, 0x00, "P46R35 biquad coeff (captured)");
    wr(46, 0x24, 0x77, "P46R36 biquad coeff (captured)");
    wr(46, 0x25, 0xE2, "P46R37 biquad coeff (captured)");
    wr(46, 0x26, 0x93, "P46R38 biquad coeff (captured)");
    wr(46, 0x27, 0x00, "P46R39 biquad coeff (captured)");
    wr(46, 0x28, 0x04, "P46R40 biquad coeff (captured)");
    wr(46, 0x29, 0x0E, "P46R41 biquad coeff (captured)");
    wr(46, 0x2A, 0xB5, "P46R42 biquad coeff (captured)");
    wr(46, 0x2B, 0x00, "P46R43 biquad coeff (captured)");
    wr(46, 0x2C, 0x04, "P46R44 biquad coeff (captured)");
    wr(46, 0x2D, 0x0E, "P46R45 biquad coeff (captured)");
    wr(46, 0x2E, 0xB5, "P46R46 biquad coeff (captured)");
    wr(46, 0x2F, 0x00, "P46R47 biquad coeff (captured)");
    wr(46, 0x30, 0x77, "P46R48 biquad coeff (captured)");
    wr(46, 0x31, 0xE2, "P46R49 biquad coeff (captured)");
    wr(46, 0x32, 0x93, "P46R50 biquad coeff (captured)");
    wr(46, 0x33, 0x00, "P46R51 biquad coeff (captured)");
    wr(44, 0x00, 0x2C, "select page 44");
    wr(44, 0x01, 0x05, "P44R1 (captured, updated)");
    wr(0, 0x00, 0x00, "select page 0");

    /* --- Final HP gain step (captured) --- */
    wr(1, 0x00, 0x01, "select page 1");
    wr(1, 0x3B, 0x28, "P1R59 (captured, final)");
    wr(1, 0x3C, 0x28, "P1R60 (captured, final)");
    wr(0, 0x00, 0x00, "select page 0 (leave the codec on page 0)");

    /*
     * Confirm the ADC actually finished powering up, instead of just
     * trusting that the R81 write got ACKed - Page 0 / Register 36
     * (0x24, "ADC Flag Register") reports real power state: bit 6 =
     * Left ADC powered up, bit 2 = Right ADC powered up. An ACKed I2C
     * write only means the register accepted the command; it says
     * nothing about whether the analog block actually finished
     * starting up. Poll for up to ~200ms since power-up isn't
     * instantaneous.
     */
    {
        uint8_t flag = 0x00;
        uint8_t left_up = 0, right_up = 0;
        uint8_t tries;

        for (tries = 0; tries < 20U; tries++) {
            if (aic3204_read_reg(0, 0x24, &flag)) {
                left_up  = (uint8_t)((flag & 0x40U) != 0U);
                right_up = (uint8_t)((flag & 0x04U) != 0U);
                if (left_up && right_up) {
                    break;
                }
            }
            delay_ms(10);
        }

        debug_print_hex32("aic3204: R36 ADC Flag Register raw", flag);
        if (!left_up) {
            debug_print("aic3204: *** Left ADC reports POWERED DOWN (bit6=0) despite "
                        "the R81 power-up write - the ADC never actually started ***\n");
        }
        if (!right_up) {
            debug_print("aic3204: *** Right ADC reports POWERED DOWN (bit2=0) despite "
                        "the R81 power-up write - the ADC never actually started ***\n");
        }
        if (left_up && right_up) {
            debug_print("aic3204: both ADC channels confirm POWERED UP (R36 bits 6 "
                        "and 2 both set) - power-up genuinely completed\n");
        }
    }

    debug_print("aic3204: phase 2 complete - full sequence ported from a real I2C "
                "capture of the original firmware. PLL sourced from BCLK (not MCLK), "
                "J=14/D=0 -> CODEC_CLKIN=86.016MHz -> Fs=192kHz exact on both ADC and "
                "DAC divider chains.\n");
}

/*
 * DAC digital output volume, both channels identically: Page 0,
 * Register 65 (0x41) = Left DAC Digital Volume, Register 66 (0x42) =
 * Right DAC Digital Volume. Per the TLV320AIC3204 datasheet this is a
 * plain signed 8-bit field in 0.5dB steps, range -63.5dB (0x81) to
 * +24dB (0x30) - register_value = round(db * 2). aic3204_phase2_init()
 * leaves both at 0x00 (0dB, unity) as part of its byte-exact captured
 * sequence; this is the first thing in this driver that actually
 * moves them away from that baseline on purpose.
 *
 * This is a DIGITAL gain stage after the DAC's own volume DSP, not
 * the analog HPL/HPR headphone driver gain (Page 1, R16/R17, also
 * captured fixed at 0x3A) - deliberately not exposing that one here:
 * it's an analog output stage bias/gain with a less-documented bit
 * layout in this codebase (see the "(captured)" comments on it in
 * aic3204_phase2_init() - nobody has decoded its exact dB mapping
 * yet), and getting it wrong risks driving the headphone amp harder
 * than intended. The digital DAC volume is the well-documented,
 * predictable, safe control to expose first; if +24dB of digital
 * headroom on top of the analog stage still isn't enough, that's the
 * next place to look.
 */
uint8_t aic3204_set_volume_db(float db)
{
    int32_t reg_val;
    uint8_t ok;

    if (db > 24.0f)  { db = 24.0f; }
    if (db < -63.5f) { db = -63.5f; }

    /* round-to-nearest 0.5dB step, then encode as the signed register
     * value (already in 0.5dB units: reg = db * 2). */
    reg_val = (int32_t)((db * 2.0f) + ((db >= 0.0f) ? 0.5f : -0.5f));
    if (reg_val > 48)   { reg_val = 48; }   /* +24.0dB */
    if (reg_val < -127) { reg_val = -127; } /* -63.5dB */

    ok  = aic3204_write_reg(0, 0x41, (uint8_t)(int8_t)reg_val); /* R65 DAC L volume */
    ok &= aic3204_write_reg(0, 0x42, (uint8_t)(int8_t)reg_val); /* R66 DAC R volume */

    if (reg_val < 0) {
        debug_print_dec("aic3204: DAC volume set, register value NEGATIVE, abs (0.5dB units)",
                         (uint32_t)(-reg_val));
    } else {
        debug_print_dec("aic3204: DAC volume set, register value (0.5dB units)",
                         (uint32_t)reg_val);
    }
    if (!ok) {
        debug_print("aic3204: *** DAC volume write with NO ACK ***\n");
    }
    return ok;
}

/*
 * MIC_PGA_L/R (Page 1, R59/R60) - the analog input gain stage BEFORE
 * the ADC, captured fixed at 0x28 (20dB, see aic3204_phase2_init())
 * until now. Exposed 31/07/2026 per the project owner, for the menu's
 * PGA tile - see main.c's ENCODER_TARGET_PGA.
 *
 * 7-bit field, 0.5dB/count: 0x00-0x5F (0-95) = 0-47.5dB, 0x60-0x7F
 * reserved per the TLV320AIC3204 datasheet - do not use. Bit 7 is
 * unused/reserved here too (always write 0 there), unlike the DAC
 * volume register above, which is a signed field - MIC_PGA gain is
 * unsigned, there's no "cut" direction, only 0dB (no boost) up to
 * 47.5dB.
 */
uint8_t aic3204_set_pga_gain_db(float db)
{
    int32_t reg_val;
    uint8_t ok;

    if (db > 47.5f) { db = 47.5f; }
    if (db < 0.0f)  { db = 0.0f; }

    /* round-to-nearest 0.5dB step (reg = db * 2, unsigned). */
    reg_val = (int32_t)((db * 2.0f) + 0.5f);
    if (reg_val > 95) { reg_val = 95; } /* 47.5dB - see the field-range note above */
    if (reg_val < 0)  { reg_val = 0; }

    ok  = aic3204_write_reg(1, 0x3BU, (uint8_t)reg_val); /* P1R59 MIC_PGA_L */
    ok &= aic3204_write_reg(1, 0x3CU, (uint8_t)reg_val); /* P1R60 MIC_PGA_R */

    debug_print_dec("aic3204: PGA gain set, register value (0.5dB units)", (uint32_t)reg_val);
    if (!ok) {
        debug_print("aic3204: *** PGA gain write with NO ACK ***\n");
    }
    return ok;
}
