#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"

#ifdef CPU_RP2040
#include <bsp/libbsp.h>
#include <stdint.h>

#define TRIG_PIN 16u
#define ECHO_PIN 17u
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define PWM_SLICE(pin) (PWM_BASE + (((pin) >> 1u) & 7u) * 0x14u)
#define PWM_DIV(pin) REG32(PWM_SLICE(pin) + 4u)
#define PWM_CTR(pin) REG32(PWM_SLICE(pin) + 8u)
#define ECHO_CSR REG32(PWM_SLICE(ECHO_PIN))
#define COUNT_WHILE_B_HIGH (1u << 4)
#define ECHO_TIMEOUT_US 45000u
#define CAPTURE_LIMIT_US 63000u
#define ECHO_PAD_HIGH (1u << 17)
#define MICROS REG32(TIMER_BASE + 0x28u)

static bool echo_high(void)
{
    return (REG32(GPIO_CTRL(ECHO_PIN) - 4u) & ECHO_PAD_HIGH) != 0;
}

_Static_assert((ECHO_PIN & 1u) != 0, "Echo capture requires PWM channel B");
_Static_assert(PWM_SLICE(ECHO_PIN) != PWM_SLICE(S5_SERVO_PIN),
               "Echo capture and servo require separate PWM slices");

void s5_hw_init(void)
{
    /* Release shared blocks without changing the other slices or resetting the clock. */
    REG32(RESETS_RESET + 0x3000u) = RESETS_RESET_PWM | RESETS_RESET_TIMER;
    while ((REG32(RESETS_RESET_DONE) & (RESETS_RESET_PWM | RESETS_RESET_TIMER)) !=
           (RESETS_RESET_PWM | RESETS_RESET_TIMER)) {}
    REG32(0x4005802cu) = 0x20cu; /* Same 1 us watchdog tick as motor hardware. */
    REG32(TIMER_PAUSE + 0x3000u) = 1u;

    gpio_set_val(TRIG_PIN, 0);
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
    /* Connected push-pull ECHO drives idle low. Bias an unplugged input high. */
    REG32(GPIO(ECHO_PIN)) = (REG32(GPIO(ECHO_PIN)) & ~GPIO_PDE) | GPIO_PUE;
    ECHO_CSR = COUNT_WHILE_B_HIGH;
    PWM_DIV(ECHO_PIN) = 125u * 16u; /* 1 us/count at 125 MHz. */
    pwm_set_wrap(ECHO_PIN, 65535u);
    pwm_set_cc(ECHO_PIN, 0);
    /* GPIO_MODE_IN keeps output disabled on the level-shifted echo input. */
    REG32(GPIO_CTRL(ECHO_PIN)) = GPIO_CTRL_FUNCSEL_PWM;

    pwm_set_enabled(S5_SERVO_PIN, FALSE);
    REG32(PWM_SLICE(S5_SERVO_PIN)) = 0;
    PWM_DIV(S5_SERVO_PIN) = 125u * 16u;
    PWM_CTR(S5_SERVO_PIN) = 0;
    pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 50 Hz. */
    s5_hw_angle(S5_CENTRE_ANGLE);
    pwm_set_pin(S5_SERVO_PIN);
    pwm_set_enabled(S5_SERVO_PIN, TRUE);
}

void s5_hw_angle(unsigned angle)
{
    /* Conservative 1–2 ms travel over 0–180 degrees. Calibrate on the vehicle. */
    pwm_set_cc(S5_SERVO_PIN, 1000u + (angle * 1000u + 90u) / 180u);
}

unsigned s5_hw_measure_mm(void)
{
    if (echo_high()) return S5_UNKNOWN_RANGE;
    PWM_CTR(ECHO_PIN) = 0;
    ECHO_CSR = COUNT_WHILE_B_HIGH | PWM_CH_CSR_EN;
    UW saved;
    DI(saved);
    gpio_set_val(TRIG_PIN, 1);
    uint32_t started = MICROS;
    while ((uint32_t)(MICROS - started) < 10u) {}
    gpio_set_val(TRIG_PIN, 0);
    EI(saved);

    /* Hardware counts while echo is high, including while the task sleeps. */
    unsigned range = S5_UNKNOWN_RANGE;
    for (;;) {
        uint32_t elapsed = MICROS - started;
        /* Reject a delayed sample before the 16-bit counter can wrap. */
        if (elapsed >= CAPTURE_LIMIT_US) break;
        unsigned count = PWM_CTR(ECHO_PIN);
        if (!echo_high() && (count || elapsed >= ECHO_TIMEOUT_US)) {
            range = s5_range_from_echo(PWM_CTR(ECHO_PIN));
            break;
        }
        if (elapsed >= ECHO_TIMEOUT_US) break; /* ECHO stayed high. */
        tk_dly_tsk(1);
    }
    ECHO_CSR = COUNT_WHILE_B_HIGH;
    return range;
}
#endif
