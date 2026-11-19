#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "s5_echo.h"

#ifdef CPU_RP2040
#include "../subsystem2/motion_hw.h"
#include <bsp/libbsp.h>
#include <stdint.h>

#define TRIG_PIN 16u
#define ECHO_PIN 17u
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define PWM_SERVO (PWM_BASE + ((S5_SERVO_PIN >> 1u) & 7u) * 0x14u)
#define ECHO_BANK_OFFSET ((ECHO_PIN / 8u) * 4u)
#define ECHO_FALL (4u << ((ECHO_PIN % 8u) * 4u))
#define ECHO_RISE (8u << ((ECHO_PIN % 8u) * 4u))
#define ECHO_EDGES (ECHO_FALL | ECHO_RISE)
#define ECHO_INTR REG32(IO_BANK0_BASE + 0xf0u + ECHO_BANK_OFFSET)
#define ECHO_ENABLE REG32(echo_irq_base)
#define ECHO_PENDING REG32(echo_irq_base + 0x20u)
#define TRIGGER_MIN_US 10u
#define MICROS REG32(TIMER_TIMERAWL)
#define STARTUP_POLLS 32768u
#define TRIGGER_POLLS 4096u
#define SERVO_OFFSET_US(angle) (((angle) * 1000u + 90u) / 180u)

static uintptr_t echo_irq_base;
static bool pwm_ready;
static volatile S5_ECHO_CAPTURE capture;

static bool echo_high(void)
{
    return (REG32(GPIO_CTRL(ECHO_PIN) - 4u) & (1u << 17)) != 0;
}

void s5_hw_echo_irq(void)
{
    if (!echo_irq_base) return;
    uint32_t edges = ECHO_PENDING & ECHO_EDGES;
    if (!edges) return;
    uint32_t now = MICROS;
    ECHO_INTR = edges;
    s5_capture_edge(&capture, now, (edges & ECHO_RISE) != 0, (edges & ECHO_FALL) != 0, S5_ECHO_OBSERVE_US);
}

bool s5_hw_init(void)
{
    gpio_set_val(TRIG_PIN, 0);
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
    /* A connected push-pull output drives idle low. An unplugged input reads high. */
    REG32(GPIO(ECHO_PIN)) = (REG32(GPIO(ECHO_PIN)) & ~GPIO_PDE) | GPIO_PUE;

    const unsigned blocks = RESETS_RESET_PWM | RESETS_RESET_TIMER;
    REG32(RESETS_RESET + 0x3000u) = blocks;
    unsigned ready = 0;
    for (unsigned i = 0; i < STARTUP_POLLS && ready != blocks; ++i)
        ready = REG32(RESETS_RESET_DONE) & blocks;
    pwm_ready = (ready & RESETS_RESET_PWM) != 0;
    if (!pwm_ready) return false;

    pwm_set_enabled(S5_SERVO_PIN, FALSE);
    REG32(PWM_SERVO) = 0;
    REG32(PWM_SERVO + 4u) = 125u * 16u; /* 1 us/count at 125 MHz. */
    REG32(PWM_SERVO + 8u) = 0;
    pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 50 Hz. */
    s5_hw_angle(S5_CENTRE_ANGLE);
    pwm_set_pin(S5_SERVO_PIN);
    pwm_set_enabled(S5_SERVO_PIN, TRUE);

    if (!(ready & RESETS_RESET_TIMER)) return false;
    REG32(0x4005802cu) = 0x20cu;
    REG32(TIMER_PAUSE + 0x3000u) = 1u;
    uint32_t before = MICROS;
    unsigned polls = 0;
    while (MICROS == before && ++polls < STARTUP_POLLS) {}
    if (polls == STARTUP_POLLS) return false;
#if APP_S5_BENCH
    if (app_gpio_irq_init() < E_OK) return false;
#endif
    /* In production this task and the shared encoder ISR run on processor 2. */
    unsigned cpu = 0;
#if TK_SUPPORT_SMP
    cpu = (unsigned)tk_get_prc() - 1u;
#endif
    UW saved;
    DI(saved);
    echo_irq_base = IO_BANK0_BASE + 0x100u + cpu * 0x30u + ECHO_BANK_OFFSET;
    ECHO_ENABLE &= ~ECHO_EDGES;
    ECHO_INTR = ECHO_EDGES;
    EI(saved);
    return true;
}

void s5_hw_angle(unsigned angle)
{
    if (pwm_ready)
        pwm_set_cc(S5_SERVO_PIN, S5_SERVO_CENTRE_US + SERVO_OFFSET_US(angle) - 500u);
}

S5_PING s5_hw_ping(void)
{
    if (!echo_irq_base) return s5_classify_echo(S5_ECHO_TIMING, 0);
    UW saved;
    DI(saved);
    bool idle_high = echo_high();
    gpio_set_val(TRIG_PIN, 1);
    uint32_t trigger_at = MICROS;
    unsigned polls = 0;
    /* Wait past the minimum to allow for sampling between timer ticks. */
    while (MICROS - trigger_at <= TRIGGER_MIN_US && ++polls < TRIGGER_POLLS) {}
    ECHO_INTR = ECHO_EDGES;
    capture.triggered_at = MICROS;
    bool triggered = polls < TRIGGER_POLLS;
    capture.state = S5_CAPTURE_IDLE;
    if (triggered && !idle_high)
        capture.state = echo_high() ? S5_CAPTURE_BAD : S5_CAPTURE_RISE;
    if (capture.state == S5_CAPTURE_RISE) ECHO_ENABLE |= ECHO_EDGES;
    gpio_set_val(TRIG_PIN, 0);
    EI(saved);

    /* Bound the wait even if the hardware timebase stops. */
    for (unsigned i = 0; triggered && !idle_high && i < 32; ++i) {
        unsigned state = capture.state;
        uint32_t elapsed = MICROS - capture.triggered_at;
        if (state >= S5_CAPTURE_DONE ||
            elapsed >= (state == S5_CAPTURE_FALL ? S5_ECHO_OBSERVE_US : S5_ECHO_TIMEOUT_US)) break;
        tk_dly_tsk(1);
    }

    DI(saved);
    ECHO_ENABLE &= ~ECHO_EDGES;
    ECHO_INTR = ECHO_EDGES;
    const S5_ECHO_CAPTURE sample = capture;
    uint32_t finished_at = MICROS;
    capture.state = S5_CAPTURE_IDLE;
    EI(saved);
    return s5_capture_result(&sample, finished_at, triggered, idle_high);
}
#endif
