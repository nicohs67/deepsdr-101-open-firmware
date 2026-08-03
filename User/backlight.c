#include "backlight.h"
#include "gd32f4xx.h"
#include "debug_uart.h"

/*
 * TIMER1CLK on this project's clock tree: APB1_PSC=4 (!=1), so - same
 * x2 doubling rule gd32_i2s_mclk_timer_start()'s comment already
 * spells out for TIMER2 on this same APB1 bus - TIMER1CLK = 2*PCLK1 =
 * 2*49.92MHz = 99.84MHz.
 *
 * PSC=0, period 4991 (4992 counts): 99.84MHz / 4992 = 20.0kHz exact -
 * see backlight.h's PWM FREQUENCY note. A clean, exact division, not
 * a rounded approximation (same standard this project already holds
 * itself to for TIMER2's MCLK).
 */
#define BACKLIGHT_PWM_PERIOD 4991U /* ARR; total counts = PERIOD+1 = 4992 */

/* Never let the panel go fully black, even if something asks for
 * percent=0 (an encoder run to the floor, a bug elsewhere, etc.) -
 * per the project owner's explicit request, 31/07/2026. Picked as "a
 * dim but still legible glow in a dark room", not calibrated against
 * an actual lux measurement - adjust if it's still too dim/too bright
 * for real use. Enforced centrally in backlight_set_percent() below,
 * so every caller (the encoder, any future UI control, and
 * backlight_init()'s own boot default) gets the floor for free with
 * no risk of forgetting it somewhere. */
#define BACKLIGHT_MIN_PERCENT 10U

/* Boot-time default - LOWERED to 50% on 31/07/2026 after the project
 * owner found the panel went BLACK at the original 100% default and
 * came back at 50%. Likely explanation: CCR=ARR+1 at 100% duty (see
 * backlight_set_percent()'s comment) makes the PWM pin sit
 * permanently HIGH with no toggling at all - many backlight driver
 * ICs (boost converters, charge pumps) use the PWM edges themselves
 * to run their internal switching/dimming control, and can
 * misbehave or shut off entirely on a static DC input rather than
 * simply treating it as "always on". NOT confirmed against this
 * board's actual driver IC - just the most common failure mode for
 * this symptom. If true, the fix is to cap backlight_set_percent()
 * at something like 99% (CCR=ARR, still visually full brightness,
 * but the pin keeps toggling at 20kHz) instead of true 100% - worth
 * trying if 100% is ever needed again. */
static uint8_t s_backlight_percent = 50U;

void backlight_init(void)
{
    timer_parameter_struct timer_init_struct;
    timer_oc_parameter_struct oc_init_struct;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_TIMER1);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

    timer_deinit(TIMER1);

    timer_struct_para_init(&timer_init_struct);
    timer_init_struct.prescaler         = 0U;
    timer_init_struct.alignedmode       = TIMER_COUNTER_EDGE;
    timer_init_struct.counterdirection  = TIMER_COUNTER_UP;
    timer_init_struct.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init_struct.period            = BACKLIGHT_PWM_PERIOD;
    timer_init_struct.repetitioncounter = 0U;
    gd32_timer_init(TIMER1, &timer_init_struct);

    timer_channel_output_struct_para_init(&oc_init_struct);
    oc_init_struct.outputstate  = TIMER_CCX_ENABLE;
    /* CONFIRMED ON HARDWARE 31/07/2026 (was a documented guess before):
     * the backlight driver is ACTIVE-LOW (pin LOW = LEDs on) - with
     * TIMER_OC_POLARITY_HIGH, percent=0 (CCR=0, pin sits LOW the whole
     * period) came out at MAXIMUM brightness and percent=100 (CCR=
     * ARR+1, pin sits HIGH the whole period) came out as TOTAL
     * DARKNESS - backwards from backlight_set_percent()'s documented
     * "0=off, 100=full" contract. TIMER_OC_POLARITY_LOW inverts the
     * pin's electrical state relative to the internal OC compare
     * result, which restores the intended mapping (0=darkest,
     * 100=brightest) without touching any of the CCR math below - see
     * backlight.h's POLARITY note. */
    oc_init_struct.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_3, &oc_init_struct);

    timer_channel_output_mode_config(TIMER1, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    /* Start at some safe, valid CCR before the timer is even enabled -
     * the real boot-default value gets applied right after
     * timer_enable() below, via backlight_set_percent() itself, so
     * the CCR formula and the BACKLIGHT_MIN_PERCENT floor only live in
     * ONE place. */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, 0U);

    timer_auto_reload_shadow_enable(TIMER1);
    timer_enable(TIMER1);

    backlight_set_percent(s_backlight_percent); /* apply the real boot default */

    debug_print("backlight: PWM started on PA3 (TIMER1_CH3, AF1), 20kHz - "
                "verify the panel actually responds before relying on this\n");
}

void backlight_set_percent(uint8_t percent)
{
    uint32_t ccr;

    if (percent > 100U) {
        percent = 100U;
    }
    if (percent < BACKLIGHT_MIN_PERCENT) {
        percent = BACKLIGHT_MIN_PERCENT;
    }
    s_backlight_percent = percent;

    /* BACKLIGHT_PWM_PERIOD+1 = total counts (ARR is counts-1, same
     * convention gd32_i2s_mclk_timer_start() uses) - so percent=100
     * lands on CCR=4992, one count PAST the ARR of 4991. That's
     * intentional: in PWM0 mode a CCR strictly greater than ARR pins
     * the output permanently active for the whole period (true 100%
     * duty), rather than the near-100%-but-not-quite you'd get by
     * clamping CCR to ARR itself. */
    ccr = ((uint32_t)(BACKLIGHT_PWM_PERIOD + 1U) * percent) / 100U;
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_3, ccr);
}

uint8_t backlight_get_percent(void)
{
    return s_backlight_percent;
}
