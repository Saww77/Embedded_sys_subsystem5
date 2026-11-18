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
#define STARTUP_POLLS 32768u
#define TRIGGER_POLLS 4096u
#define CAPTURE_POLLS 8u
#define SERVO_OFFSET_US(angle) (((angle) * 1000u + 90u) / 180u)

static bool pwm_ready, echo_ready, seen_echo;

static bool echo_high(void)
{
    return (REG32(GPIO_CTRL(ECHO_PIN) - 4u) & ECHO_PAD_HIGH) != 0;
}

_Static_assert((ECHO_PIN & 1u) != 0, "Echo capture requires PWM channel B");
_Static_assert(PWM_SLICE(ECHO_PIN) != PWM_SLICE(S5_SERVO_PIN),
               "Echo capture and servo require separate PWM slices");
_Static_assert(S5_SERVO_CENTRE_US + SERVO_OFFSET_US(S5_MIN_ANGLE) >= 1500u &&
               S5_SERVO_CENTRE_US + SERVO_OFFSET_US(S5_MAX_ANGLE) <= 2500u,
               "Keep the calibrated scan within the 1000..2000 us pulse range");

bool s5_hw_init(void)
{
    gpio_set_val(TRIG_PIN, 0);
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
    /* Connected push-pull ECHO drives idle low. Bias an unplugged input high. */
    REG32(GPIO(ECHO_PIN)) = (REG32(GPIO(ECHO_PIN)) & ~GPIO_PDE) | GPIO_PUE;

    /* Release shared blocks without changing the other slices or resetting the clock. */
    const unsigned blocks = RESETS_RESET_PWM | RESETS_RESET_TIMER;
    REG32(RESETS_RESET + 0x3000u) = blocks;
    unsigned ready = 0;
    for (unsigned i = 0; i < STARTUP_POLLS && ready != blocks; ++i)
        ready = REG32(RESETS_RESET_DONE) & blocks;
    pwm_ready = (ready & RESETS_RESET_PWM) != 0;
    echo_ready = seen_echo = false;
    if (!pwm_ready) return false;

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

    if (!(ready & RESETS_RESET_TIMER)) return false;
    REG32(0x4005802cu) = 0x20cu; /* Same 1 us watchdog tick as motor hardware. */
    REG32(TIMER_PAUSE + 0x3000u) = 1u;
    uint32_t before = MICROS;
    for (unsigned i = 0; i < STARTUP_POLLS && !echo_ready; ++i)
        echo_ready = MICROS != before;
    return echo_ready;
}

void s5_hw_angle(unsigned angle)
{
    /* Apply the calibrated straight-ahead pulse to every scan angle. */
    if (pwm_ready)
        pwm_set_cc(S5_SERVO_PIN, S5_SERVO_CENTRE_US + SERVO_OFFSET_US(angle) - 500u);
}

S5_PING s5_hw_ping(void)
{
    if (!echo_ready) return s5_classify_echo(S5_ECHO_TIMING, 0, &seen_echo);
    bool idle_high = echo_high();
    PWM_CTR(ECHO_PIN) = 0;
    ECHO_CSR = COUNT_WHILE_B_HIGH | PWM_CH_CSR_EN;
    UW saved;
    DI(saved);
    gpio_set_val(TRIG_PIN, 1);
    uint32_t started = MICROS;
    bool triggered = false;
    for (unsigned i = 0; i < TRIGGER_POLLS && !triggered; ++i)
        triggered = (uint32_t)(MICROS - started) >= 12u;
    gpio_set_val(TRIG_PIN, 0);
    EI(saved);

    /* Even with ECHO already high, emit TRIG so both pins can be probed. */
    S5_ECHO_STATE state = triggered && idle_high ? S5_ECHO_HIGH : S5_ECHO_TIMING;
    unsigned pulse = 0;
    /* Hardware counts while echo is high, including while the task sleeps. */
    for (unsigned i = 0; triggered && !idle_high && i < CAPTURE_POLLS; ++i) {
        uint32_t elapsed = MICROS - started;
        /* Reject a delayed sample before the 16-bit counter can wrap. */
        if (elapsed >= CAPTURE_LIMIT_US) break;
        unsigned count = PWM_CTR(ECHO_PIN);
        if (!echo_high() && (count || elapsed >= ECHO_TIMEOUT_US)) {
            pulse = PWM_CTR(ECHO_PIN);
            state = pulse ? S5_ECHO_PULSE : S5_ECHO_QUIET;
            break;
        }
        if (elapsed >= ECHO_TIMEOUT_US) {
            state = S5_ECHO_HIGH;
            break;
        }
        tk_dly_tsk(1);
    }
    ECHO_CSR = COUNT_WHILE_B_HIGH;
    return s5_classify_echo(state, pulse, &seen_echo);
}
#endif
