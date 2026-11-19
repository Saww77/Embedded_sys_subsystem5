#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_profile.h"

#define S5_RANGE_MAX_AGE_MS 300u
#define S5_SLOW_RANGE_MM 450u
#define S5_MANEUVER_STOP_MM 150u
#define S5_MAX_PASS_MM 500u
#define S5_APPROACH_RPM 20u
#define S5_TURN_CLEARANCE_MM 100u
#define S5_REVERSE_MM 30u
#define S5_SEARCH_MARGIN_MM 40u

/* Phase and side are also sent in the low six bits of /avoidance. */
typedef enum {
    S5_OFF, S5_PROFILE_PATH, S5_TURN_OUT, S5_SHIFT_OUT, S5_ALIGN_OUTSIDE,
    S5_PASS, S5_TURN_IN, S5_FIND_LINE, S5_CENTER_LINE, S5_ALIGN_LINE,
    S5_ALIGN_RETRY, S5_COMPLETE, S5_FAILED, S5_CHECK_LINE, S5_BACK_UP
} S5_PHASE;
typedef enum { S5_NO_SIDE, S5_LEFT, S5_RIGHT } S5_SIDE;
typedef enum { S5_WAIT, S5_STOP, S5_ADVANCE, S5_PIVOT_LEFT, S5_PIVOT_RIGHT, S5_REVERSE } S5_ACTION;
typedef enum { S5_MOTION_NONE, S5_MOTION_QUEUED, S5_MOTION_PENDING, S5_MOTION_DONE, S5_MOTION_FAILED } S5_MOTION;

typedef struct {
    uint32_t now, range_at, profile_at, line_at;
    S5_PROFILE profile;
    unsigned range_mm : 12;
    unsigned sensor_to_axle_mm : 9;
    unsigned motion : 3;
    bool stopped : 1, range_valid : 1, profile_valid : 1;
    bool line_hit : 1, line_junction : 1, line_centered : 1, can_retrace : 1;
    bool near_original_line : 1;
} S5_AVOID_INPUT;
typedef struct {
    uint32_t phase_at, last_line_at;
    unsigned phase:4, side:2, line_samples:2, reverse_available:1;
    unsigned offset_mm:9, next_pass_mm:9, passed_mm:9;
} S5_AVOID;
typedef struct {
    uint16_t action : 3;
    uint16_t distance_mm : 9;
} S5_AVOID_OUTPUT;

S5_SIDE s5_plan_bypass(const S5_PROFILE *profile, unsigned *offset_mm, unsigned *pass_mm);
void s5_avoid_begin(S5_AVOID *state, uint32_t now);
S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in);
#endif
