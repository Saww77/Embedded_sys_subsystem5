#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"

#ifdef CPU_RP2040
#define SERVO_SETTLE_MS 120
#define SERVO_SWEEP_SETTLE_MS 240
#define SERVO_HOME_SETTLE_MS 350
#define PING_INTERVAL_MS 60
#define RANGE_LOG_INTERVAL_MS 2000u

static unsigned servo_angle = S5_CENTRE_ANGLE;
static const char *const echo_status[] = {
    [S5_ECHO_QUIET] = "rise timeout",
    [S5_ECHO_PULSE] = "pulse",
    [S5_ECHO_HIGH] = "high before trigger",
    [S5_ECHO_TIMING] = "timing fault",
    [S5_ECHO_FALL_TIMEOUT] = "fall timeout",
};

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
    bool previous_open = false;
    for (unsigned attempt = 0; attempt < 3u; ++attempt) {
        if (attempt) tk_dly_tsk(PING_INTERVAL_MS);
        S5_PING ping = s5_hw_ping();
        if (verbose) log_ping("[S5 BENCH]", attempt ? "recheck" : "scan", servo_angle, ping);
        if (s5_range_measured(ping.range_mm)) return ping.range_mm;
        /* Retry one missing return. A third ping is only needed to confirm open range. */
        if (ping.range_mm == S5_OPEN_RANGE) {
            if (previous_open) return S5_OPEN_RANGE;
            previous_open = true;
        } else if (attempt) {
            break;
        }
    }
    return S5_UNKNOWN_RANGE;
}

static BOOL scan_obstacle(S5_PROFILE *profile, BOOL verbose, const uint32_t *request)
{
    const char *tag = verbose ? "[S5 BENCH]" : "[S5]";
    S5_SCAN samples = {0};
    tm_printf((UB*)"%s GP12 coarse sweep 30..150 deg\n", tag);
    for (unsigned i = 0; i < S5_SCAN_POINTS; i += S5_COARSE_STEP) {
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
        s5_scan_set(&samples, i, read_at(i, verbose));
    }

    unsigned anchor = s5_scan_target(&samples);
    if (anchor == S5_NO_TARGET) {
        aim(S5_CENTRE_ANGLE);
        unsigned forward = s5_hw_ping().range_mm;
        if (request) vehicle_submit_obstacle_range(forward);
        /* A missing echo on any coarse bearing leaves that sector unresolved. */
        for (unsigned i = 0; i < S5_SCAN_POINTS; i += S5_COARSE_STEP)
            if (!s5_range_available(s5_scan_get(&samples, i))) return FALSE;
        if (!s5_range_available(forward) || forward <= S5_CLEAR_MM) return FALSE;
        *profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        tm_printf((UB*)"%s no obstacle in the coarse sweep\n", tag);
        return TRUE;
    }

    unsigned coarse = s5_scan_get(&samples, anchor);
    if (verbose)
        tm_printf((UB*)"%s fine scan starting at %u deg\n", tag, S5_MIN_ANGLE + anchor * S5_FINE_STEP);
    unsigned fine = read_at(anchor, verbose);
    if (!s5_same_obstacle(fine, coarse) || fine > S5_CLEAR_MM) {
        tm_printf((UB*)"%s fine scan could not confirm the %u mm target at %u deg\n",
                  tag, coarse, servo_angle);
        goto cancelled;
    }
    s5_scan_set(&samples, anchor, fine);

    /* Walk outward at 5-degree resolution until each edge is bracketed. */
    unsigned first = anchor, last = anchor;
    while (first > 0 && s5_same_obstacle(s5_scan_get(&samples, first), fine)) {
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
        unsigned range = read_at(--first, verbose);
        s5_scan_set(&samples, first, range);
    }
    while (last + 1u < S5_SCAN_POINTS && s5_same_obstacle(s5_scan_get(&samples, last), fine)) {
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
        unsigned range = read_at(++last, verbose);
        s5_scan_set(&samples, last, range);
    }

    aim(S5_CENTRE_ANGLE);
    if (request) vehicle_submit_obstacle_range(s5_hw_ping().range_mm);
    *profile = s5_profile(&samples, first, last, anchor);
    if (profile->width_mm == S5_UNKNOWN_WIDTH)
        tm_printf((UB*)"%s width unavailable: nearest=%u mm edges L=%u R=%u (1=resolved)\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->left_edge_valid,
                  (unsigned)profile->right_edge_valid);
    else
        tm_printf((UB*)"%s profile nearest=%u mm width=%u mm angle=%u deg\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->width_mm,
                  S5_MIN_ANGLE + profile->angle_offset_deg);
    return TRUE;

cancelled:
    aim(S5_CENTRE_ANGLE);
    return FALSE;
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
        tm_printf((UB*)"[S5] servo or ultrasonic capture failed to start\n");
    else
        tm_printf((UB*)"[S5] GP%u centred: %u us. Waiting for sensor to settle.\n",
                  S5_SERVO_PIN, S5_SERVO_CENTRE_US);
    /* The driver holds centre as soon as PWM is enabled. Let the sensor settle
       there before accepting the first range on either firmware profile. */
    tk_dly_tsk(SERVO_HOME_SETTLE_MS);
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    setup();
    tm_printf((UB*)"[S5] GP16 trigger, GP17 echo. Entry=%u mm, clear=%u mm\n",
              S5_STOP_MM, S5_CLEAR_MM);

    for (;;) {
        S5_PING ping = s5_hw_ping();
        vehicle_submit_obstacle_range(ping.range_mm);
        log_forward(ping, "forward");
        tk_dly_tsk(PING_INTERVAL_MS);

        uint32_t request;
        S2_STATUS motion;
        if (vehicle_obstacle_scan_request(&request) && motion_get_status(&motion) &&
            s2_get_bits(&motion, 0, 2) == 0 && s2_get_bits(&motion, 9, 9) <= 2 &&
            s2_get_bits(&motion, 18, 9) <= 2) {
            S5_PROFILE profile;
            if (scan_obstacle(&profile, FALSE, &request))
                vehicle_submit_obstacle_profile(profile, request);
            /* The sweep finishes with a forward ping. */
            tk_dly_tsk(PING_INTERVAL_MS);
        }
    }
}

#if APP_S5_BENCH
#define BENCH_PING_MS 350
#define BENCH_RELEASE_MM 250u
#define BENCH_CONFIRMATIONS 2u
#define BENCH_TIMEOUT_CONFIRMATIONS 3u

static unsigned bench_wait_for_echo(BOOL want_target)
{
    unsigned confirmed = 0;
    unsigned timeouts = 0;
    for (;;) {
        tk_dly_tsk(BENCH_PING_MS);
        S5_PING ping = s5_hw_ping();
        log_ping("[S5 BENCH]", want_target ? "waiting for target" : "waiting to rearm",
                 S5_CENTRE_ANGLE, ping);

        unsigned range = ping.range_mm;
        bool measured = s5_range_measured(range);
        bool matches = want_target ? measured && range <= S5_CLEAR_MM :
                       range == S5_OPEN_RANGE || (measured && range > BENCH_RELEASE_MM);
        confirmed = matches ? confirmed + 1u : 0u;
        if (confirmed == BENCH_CONFIRMATIONS) return range;
        /* In this motor-off test a missing target can restart the detector. */
        bool timed_out = ping.state == S5_ECHO_QUIET || ping.state == S5_ECHO_FALL_TIMEOUT;
        timeouts = !want_target && timed_out ? timeouts + 1u : 0u;
        if (timeouts == BENCH_TIMEOUT_CONFIRMATIONS) {
            tm_printf((UB*)"[S5 BENCH] Three echo timeouts. Restarting target detection.\n");
            return S5_UNKNOWN_RANGE;
        }
    }
}

EXPORT void subsystem5_bench(void)
{
    tm_printf((UB*)"[S5 BENCH] GP12 servo, GP16 trigger, GP17 echo. Drive outputs off.\n");
    tm_printf((UB*)"[S5 BENCH] Scan starts after two forward echoes within %u mm.\n", S5_CLEAR_MM);
    setup();
    for (;;) {
        unsigned detected = bench_wait_for_echo(TRUE);
        tm_printf((UB*)"[S5 BENCH] obstacle detected at %u mm. Starting width test.\n", detected);
        S5_PROFILE profile;
        (void)scan_obstacle(&profile, TRUE, NULL);
        tm_printf((UB*)"[S5 BENCH] Servo centred. Remove target or move it beyond %u mm to rearm.\n",
                  BENCH_RELEASE_MM);
        (void)bench_wait_for_echo(FALSE);
        tm_printf((UB*)"[S5 BENCH] Test rearmed.\n");
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
    ID task = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC2, "obstacle_scan");
    return task < E_OK ? task : E_OK;
#else
    return E_OK;
#endif
}
