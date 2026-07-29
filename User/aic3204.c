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
 */
static void aic3204_hw_reset(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(AIC3204_NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AIC3204_NRST_PIN);
    gpio_output_options_set(AIC3204_NRST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, AIC3204_NRST_PIN);

    gpio_bit_reset(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* hold in reset */
    delay_ms(10);
    gpio_bit_set(AIC3204_NRST_PORT, AIC3204_NRST_PIN); /* release */
    delay_ms(10);

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
 * Phase 2 clock/dividers: confirmed with an oscilloscope that
 * MCLK=12.288MHz, BCLK=1.536MHz, WCLK=48kHz - the fixed 256x/32x ratio
 * native to this MCU's MCKOUT block, with Fs=48kHz (see gd32_i2s.c).
 * These are not predictions - they're computed from the real, measured
 * MCLK.
 *   CODEC_CLKIN = NADC x MADC x AOSR x ADC_FS
 *   12,288,000 = NADC x MADC x 128 x 48,000  =>  NADC x MADC = 2
 * Filter A (TI's recommendation for 48kHz high-performance operation,
 * AOSR=128), processing block PRB_R1 (Stereo, RC=6):
 * MADC*AOSR/32=2*128/32=8>=6, OK.
 */
#define AIC3204_NADC   1U
#define AIC3204_MADC   2U
#define AIC3204_AOSR   128U  /* Filter A, for 48kHz (per TI's recommendation) */
/* DAC: same shared CODEC_CLKIN, Filter A also used for 48kHz (DOSR
 * multiple of 8; 128*48000=6.144MHz, within the 2.8-6.2MHz range
 * required by the datasheet). NDAC*MDAC*128*48000=12,288,000 =>
 * NDAC*MDAC=2, same as the ADC side. */
#define AIC3204_NDAC   1U
#define AIC3204_MDAC   2U
#define AIC3204_DOSR   128U

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
    debug_print("\naic3204: phase 2 - clock + differential I/Q ADC + power-up\n");

    /* --- Clock: MCLK fed directly to CODEC_CLKIN, no PLL --- */
    wr(0, 0x04, 0x00, "R4 CODEC_CLKIN=MCLK, no PLL");
    wr(0, 0x05, 0x00, "R5 PLL power down");

    /* --- Audio interface: I2S, 16-bit, BCLK/WCLK as INPUT (AIC3204 is
     * the slave - the GD32 is the I2S master) --- */
    wr(0, 0x1B, 0x00, "R27 I2S 16-bit, BCLK/WCLK input (slave)");
    wr(0, 0x1C, 0x00, "R28 data offset = 0");

    /* --- ADC dividers (NADC/MADC) and AOSR --- */
    wr(0, 0x12, 0x80 | AIC3204_NADC, "R18 NADC power+value");
    wr(0, 0x13, 0x80 | AIC3204_MADC, "R19 MADC power+value");
    wr(0, 0x14, AIC3204_AOSR,        "R20 AOSR");

    /* --- DAC dividers (NDAC/MDAC) and DOSR - clock is shared with the
     * ADC side, left enabled even though the audio output isn't used
     * here --- */
    wr(0, 0x0B, 0x80 | AIC3204_NDAC, "R11 NDAC power+value");
    wr(0, 0x0C, 0x80 | AIC3204_MDAC, "R12 MDAC power+value");
    wr(0, 0x0D, 0x00,                "R13 DOSR hi_byte");
    wr(0, 0x0E, AIC3204_DOSR,        "R14 DOSR lo_byte");

    /* --- Processing block: PRB_R1 (Stereo, Filter A, no biquads -
     * RC=6, valid with MADC=2/AOSR=128 since MADC*AOSR/32=8>=6) --- */
    wr(0, 0x3D, 1, "R61 ADC processing block PRB_R1");

    /* --- Page 1: analog power --- */
    wr(1, 0x01, 0x08, "P1R1 disable AVDD-from-DVDD");
    wr(1, 0x02, 0x01, "P1R2 analog blocks + LDO ON");

    /* --- ADC: differential input routing ---
     * I (left)  = IN2_L(+) / IN2_R(-)  -- confirmed on real hardware
     *   (previously validated electrically).
     * Q (right) = IN3_R(+) / IN3_L(-)  -- architecture confirmed
     *   against TI's SLAA557 (Figure 2-1, right-channel differential
     *   option: IN3_R and IN3_L), but the EXACT register value
     *   (0b00000100 = 10kOhm in the 3rd 2-bit field) is extrapolated
     *   from the bit pattern of an earlier driver for this board, not
     *   read directly from a bit-level table - verify with the
     *   AIC3204 Control Software or a register readback if in doubt.
     */
    wr(1, 0x33, 0x00, "P1R51 MICBIAS off (not needed, differential input)");
    wr(1, 0x34, 0x10, "P1R52 IN2L -> PGA_L positive, 10k (I, confirmed)");
    wr(1, 0x36, 0x10, "P1R54 IN2R -> PGA_L negative, 10k (I, confirmed)");
    wr(1, 0x37, 0x04, "P1R55 IN3R -> PGA_R positive, 10k (Q, extrapolated)");
    wr(1, 0x39, 0x04, "P1R57 IN3L -> PGA_R negative, 10k (Q, extrapolated)");

    /* PGA gain: 0dB to start (bit7=0 => unmuted, bits[6:0]=0dB).
     * Raise later depending on the real signal level. */
    wr(1, 0x3B, 0x00, "P1R59 MIC_PGA_L 0dB, unmuted");
    wr(1, 0x3C, 0x00, "P1R60 MIC_PGA_R 0dB, unmuted");

    /* --- ADC power-up (page 0) --- */
    wr(0, 0x51, 0xC0, "R81 power-up left+right ADC");
    wr(0, 0x52, 0x00, "R82 ADC unmuted, 0dB fine gain");

    debug_print("aic3204: phase 2 complete - I(IN2)=left confirmed, "
                "Q(IN3)=right extrapolated (pending verification), NADC=1/MADC=2/AOSR=128 "
                "computed from MCLK=12.288MHz and Fs=48kHz, both confirmed on hardware\n");
}
