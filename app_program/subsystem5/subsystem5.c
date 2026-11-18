#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"

#ifdef CPU_RP2040
#define COARSE_INDEX_STEP 6u
#define SERVO_SETTLE_MS 120
#define SERVO_SWEEP_SETTLE_MS 240
#define SERVO_TEST_MS 350
#define OPEN_RECHECK_DELAY_MS 40
#define PING_INTERVAL_MS 60
#define RANGE_LOG_INTERVAL_MS 2000u
#define CENTRE_INDEX ((S5_CENTRE_ANGLE - S5_MIN_ANGLE) / S5_FINE_STEP)

static unsigned servo_angle = S5_CENTRE_ANGLE;
static const char *const echo_status[] = {"no pulse", "pulse", "stuck high", "timing fault"};

static void aim(unsigned angle)
{
    unsigned change = angle > servo_angle ? angle - servo_angle : servo_angle - angle;
    s5_hw_angle(angle);
    servo_angle = angle;
    tk_dly_tsk(change > 15u ? SERVO_SWEEP_SETTLE_MS : SERVO_SETTLE_MS);
}

static void log_ping(const char *tag, const char *context, unsigned angle, S5_PING ping)
{
    if (s5_range_measured(ping.range_mm))
        tm_printf((UB*)"%s %s angle=%u echo=%u us range=%u mm\n",
                  tag, context, angle, (unsigned)ping.pulse_us, (unsigned)ping.range_mm);
    else
        tm_printf((UB*)"%s %s angle=%u echo=%s pulse=%u us range=%s\n",
                  tag, context, angle, echo_status[ping.state], (unsigned)ping.pulse_us,
                  ping.range_mm == S5_OPEN_RANGE ? "open" : "unavailable");
}

static unsigned read_at(unsigned index, BOOL verbose)
{
    aim(S5_MIN_ANGLE + index * S5_FINE_STEP);
    S5_PING ping = s5_hw_ping();
    if (verbose) log_ping("[S5 BENCH]", "scan", servo_angle, ping);
    if (ping.range_mm == S5_OPEN_RANGE) {
        /* A long out-of-range pulse plus this delay spaces triggers by 60 ms. */
        tk_dly_tsk(OPEN_RECHECK_DELAY_MS);
        ping = s5_hw_ping();
        if (verbose) log_ping("[S5 BENCH]", "recheck", servo_angle, ping);
    }
    return ping.range_mm;
}

static BOOL scan_obstacle(S5_PROFILE *profile, BOOL verbose)
{
    const char *tag = verbose ? "[S5 BENCH]" : "[S5]";
    unsigned short samples[S5_SCAN_POINTS];
    tm_printf((UB*)"%s GP12 sweep start 30..150 deg\n", tag);
    for (unsigned i = 0; i < S5_SCAN_POINTS; i += COARSE_INDEX_STEP)
        samples[i] = read_at(i, verbose);

    /* Profile the return in the car's path. Nearby objects elsewhere in
       the coarse sweep still contribute clearance evidence. */
    unsigned first = CENTRE_INDEX - COARSE_INDEX_STEP;
    unsigned last = CENTRE_INDEX + COARSE_INDEX_STEP;
    unsigned coarse = samples[CENTRE_INDEX];
    if (coarse > S5_CLEAR_MM) {
        aim(S5_CENTRE_ANGLE);
        tm_printf((UB*)"%s no target within %u mm at 90 deg (sample=%u; 4094=open, 4095=unavailable)\n",
                  tag, S5_CLEAR_MM, coarse);
        return FALSE;
    }
    if (samples[first] <= coarse + S5_PROFILE_TOLERANCE_MM) first = 0;
    if (samples[last] <= coarse + S5_PROFILE_TOLERANCE_MM) last = S5_SCAN_POINTS - 1;
    for (unsigned i = first; i <= last; ++i) samples[i] = read_at(i, verbose);

    aim(S5_CENTRE_ANGLE);
    unsigned fine = samples[CENTRE_INDEX];
    if (fine > S5_CLEAR_MM || fine > coarse + S5_PROFILE_TOLERANCE_MM ||
        coarse > fine + S5_PROFILE_TOLERANCE_MM) {
        tm_printf((UB*)"%s centre echo changed during sweep (coarse=%u fine=%u; 4094=open, 4095=unavailable)\n",
                  tag, coarse, fine);
        return FALSE;
    }
    *profile = s5_profile(samples, first, last, CENTRE_INDEX);
    if (profile->width_mm == S5_UNKNOWN_WIDTH)
        tm_printf((UB*)"%s width unavailable: nearest=%u mm edges L=%u R=%u (1=resolved)\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->left_edge_valid,
                  (unsigned)profile->right_edge_valid);
    else
        tm_printf((UB*)"%s profile nearest=%u mm width=%u mm angle=%u deg\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->width_mm,
                  (unsigned)profile->angle_deg);
    return TRUE;
}

static void log_forward(S5_PING ping, const char *context)
{
    static BOOL logged;
    static uint32_t last_log_at;
    SYSTIM time;
    tk_get_tim(&time);
    if (logged && time.lo - last_log_at < RANGE_LOG_INTERVAL_MS) return;
    logged = TRUE;
    last_log_at = time.lo;
    log_ping("[S5]", context, S5_CENTRE_ANGLE, ping);
}

static void setup(void)
{
    if (!s5_hw_init())
        tm_printf((UB*)"[S5] PWM or ultrasonic timer failed to start\n");
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    setup();
    tm_printf((UB*)"[S5] GP16 trigger, GP17 echo. Entry=%u mm, clear=%u mm\n",
              S5_STOP_MM, S5_CLEAR_MM);
    /* The controller stays interlocked until the first accepted sonar sample. */
    tm_printf((UB*)"[S5] GP%u servo check: 60, 120, 90 degrees\n", (unsigned)S5_SERVO_PIN);
    s5_hw_angle(60u);
    tk_dly_tsk(SERVO_TEST_MS);
    s5_hw_angle(120u);
    tk_dly_tsk(SERVO_TEST_MS);
    s5_hw_angle(S5_CENTRE_ANGLE);
    tk_dly_tsk(SERVO_TEST_MS);

    BOOL has_profile = FALSE;
    for (;;) {
        S5_PING ping = s5_hw_ping();
        unsigned range = ping.range_mm;
        vehicle_submit_obstacle_range(range);
        if (s5_range_measured(range) && range <= S5_CLEAR_MM && vehicle_obstacle_scan_allowed()) {
            if (!has_profile) {
                vehicle_submit_obstacle_profile((S5_PROFILE){
                    .nearest_mm = range, .width_mm = S5_UNKNOWN_WIDTH,
                    .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                    .angle_deg = S5_CENTRE_ANGLE
                }, FALSE);
                has_profile = TRUE;
            }
            tk_dly_tsk(PING_INTERVAL_MS);
            S2_STATUS motion;
            if (motion_get_status(&motion) && s2_get_bits(&motion, 0, 2) == 0 &&
                s2_get_bits(&motion, 9, 9) <= 2 && s2_get_bits(&motion, 18, 9) <= 2) {
                S5_PROFILE profile;
                has_profile = scan_obstacle(&profile, FALSE);
                if (has_profile) vehicle_submit_obstacle_profile(profile, TRUE);
            } else {
                log_forward(ping, "scan waiting for drive motors to stop");
            }
        } else {
            log_forward(ping, "forward (sweep inactive)");
            has_profile = FALSE;
            tk_dly_tsk(PING_INTERVAL_MS);
        }
    }
}

#if APP_S5_BENCH
EXPORT void subsystem5_bench(void)
{
    tm_printf((UB*)"[S5 BENCH] GP12 servo, GP16 trigger, GP17 echo. Drive outputs off.\n");
    tm_printf((UB*)"[S5 BENCH] Use a flat-front box 120..150 mm straight ahead for width.\n");
    setup();
    for (;;) {
        for (unsigned sample = 0; sample < 3; ++sample) {
            log_ping("[S5 BENCH]", "forward", S5_CENTRE_ANGLE, s5_hw_ping());
            tk_dly_tsk(350);
        }
        S5_PROFILE profile;
        (void)scan_obstacle(&profile, TRUE);
        tk_dly_tsk(500);
    }
}
#endif
#endif

EXPORT ER subsystem5_init(void)
{
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
#ifdef CPU_RP2040
    vehicle_enable_obstacle_monitor();
    ID task = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC1, "obstacle_scan");
    return task < E_OK ? task : E_OK;
#else
    return E_OK;
#endif
}
