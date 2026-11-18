#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"

#ifdef CPU_RP2040
#define COARSE_INDEX_STEP 6u
#define SERVO_SETTLE_MS 120
#define PING_INTERVAL_MS 60
#define CENTRE_INDEX ((S5_CENTRE_ANGLE - S5_MIN_ANGLE) / S5_FINE_STEP)

static unsigned read_at(unsigned index)
{
    s5_hw_angle(S5_MIN_ANGLE + index * S5_FINE_STEP);
    tk_dly_tsk(SERVO_SETTLE_MS);
    return s5_hw_measure_mm();
}

static BOOL scan_obstacle(void)
{
    unsigned short samples[S5_SCAN_POINTS];
    for (unsigned i = 0; i < S5_SCAN_POINTS; i += COARSE_INDEX_STEP)
        samples[i] = read_at(i);

    /* Profile the return in the car's path. Nearby objects elsewhere in
       the coarse sweep still contribute clearance evidence. */
    unsigned first = CENTRE_INDEX - COARSE_INDEX_STEP;
    unsigned last = CENTRE_INDEX + COARSE_INDEX_STEP;
    unsigned coarse = samples[CENTRE_INDEX];
    if (samples[first] <= coarse + S5_PROFILE_TOLERANCE_MM) first = 0;
    if (samples[last] <= coarse + S5_PROFILE_TOLERANCE_MM) last = S5_SCAN_POINTS - 1;
    if (coarse <= S5_CLEAR_MM) {
        for (unsigned i = first; i <= last; ++i) samples[i] = read_at(i);
    }

    s5_hw_angle(S5_CENTRE_ANGLE);
    tk_dly_tsk(SERVO_SETTLE_MS);
    unsigned fine = samples[CENTRE_INDEX];
    BOOL found = coarse <= S5_CLEAR_MM && fine <= S5_CLEAR_MM &&
                 fine <= coarse + S5_PROFILE_TOLERANCE_MM &&
                 coarse <= fine + S5_PROFILE_TOLERANCE_MM;
    if (found) vehicle_submit_obstacle_profile(s5_profile(samples, first, last, CENTRE_INDEX), TRUE);
    return found;
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    s5_hw_init();
    tk_dly_tsk(SERVO_SETTLE_MS);

    BOOL has_profile = FALSE;
    for (;;) {
        unsigned range = s5_hw_measure_mm();
        vehicle_submit_obstacle_range(range);
        if (range != S5_UNKNOWN_RANGE && vehicle_obstacle_scan_allowed()) {
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
                has_profile = scan_obstacle();
            }
        } else {
            has_profile = FALSE;
            tk_dly_tsk(PING_INTERVAL_MS);
        }
    }
}
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
