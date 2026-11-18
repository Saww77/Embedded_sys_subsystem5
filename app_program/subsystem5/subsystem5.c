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

static void publish_profile(S5_PROFILE profile)
{
    vehicle_submit_obstacle_profile(profile.nearest_mm, profile.width_mm,
                                    profile.left_mm, profile.right_mm,
                                    profile.angle_deg);
}

static BOOL scan_obstacle(unsigned centre)
{
    unsigned short samples[S5_SCAN_POINTS];
    unsigned anchor = CENTRE_INDEX, nearest = centre;
    for (unsigned i = 0; i < S5_SCAN_POINTS; i += COARSE_INDEX_STEP) {
        unsigned range = read_at(i);
        if (range < nearest) {
            nearest = range;
            anchor = i;
        }
    }
    unsigned first = anchor >= COARSE_INDEX_STEP ? anchor - COARSE_INDEX_STEP : 0;
    unsigned last = anchor + COARSE_INDEX_STEP;
    if (last >= S5_SCAN_POINTS) last = S5_SCAN_POINTS - 1;
    for (unsigned i = first; i <= last; ++i) samples[i] = read_at(i);
    BOOL found = samples[anchor] <= S5_CLEAR_MM;
    if (found) publish_profile(s5_profile(samples, first, last, anchor));
    s5_hw_angle(S5_CENTRE_ANGLE);
    tk_dly_tsk(SERVO_SETTLE_MS);
    return found;
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    s5_hw_init();
    tk_dly_tsk(SERVO_SETTLE_MS);

    BOOL blocked = TRUE;
    BOOL has_profile = FALSE;
    for (;;) {
        unsigned range = s5_hw_measure_mm();
        BOOL no_echo = range == S5_UNKNOWN_RANGE;
        blocked = no_echo || range <= (blocked ? S5_CLEAR_MM : S5_STOP_MM);
        vehicle_update_obstacle_guard(blocked, no_echo);
        if (!no_echo && blocked) {
            if (!has_profile) {
                publish_profile((S5_PROFILE){
                    .nearest_mm = range, .width_mm = S5_UNKNOWN_WIDTH,
                    .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                    .angle_deg = S5_CENTRE_ANGLE
                });
                has_profile = TRUE;
            }
            tk_dly_tsk(PING_INTERVAL_MS);
            S2_STATUS motion;
            if (motion_get_status(&motion) && s2_get_bits(&motion, 0, 2) == 0 &&
                s2_get_bits(&motion, 9, 9) == 0 && s2_get_bits(&motion, 18, 9) == 0) {
                has_profile = scan_obstacle(range);
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
    vehicle_update_obstacle_guard(TRUE, FALSE);
    ID task = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC1, "obstacle_scan");
    return task < E_OK ? task : E_OK;
#else
    return E_OK;
#endif
}
