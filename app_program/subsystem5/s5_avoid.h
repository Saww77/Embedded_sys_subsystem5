#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_profile.h"

#define S5_RANGE_MAX_AGE_MS 300u
#define S5_SLOW_RANGE_MM 300u
#define S5_APPROACH_RPM 20u
#define S5_TURN_CLEARANCE_MM 100u

/* Phase and side are also sent in the low six bits of /avoidance. */
typedef enum {
    S5_OFF, S5_PROFILE_PATH, S5_TURN_OUT, S5_SHIFT_OUT, S5_ALIGN_OUTSIDE,
    S5_PASS, S5_TURN_IN, S5_FIND_LINE, S5_CENTER_LINE, S5_ALIGN_LINE,
    S5_ALIGN_RETRY, S5_COMPLETE, S5_FAILED, S5_CHECK_LINE
} S5_PHASE;
typedef enum { S5_NO_SIDE, S5_LEFT, S5_RIGHT } S5_SIDE;
typedef enum { S5_WAIT, S5_STOP, S5_ADVANCE, S5_PIVOT_LEFT, S5_PIVOT_RIGHT } S5_ACTION;
typedef enum { S5_MOTION_NONE, S5_MOTION_QUEUED, S5_MOTION_PENDING, S5_MOTION_DONE, S5_MOTION_FAILED } S5_MOTION;

typedef struct {
    uint32_t now, range_at, profile_at, line_at;
    S5_PROFILE profile;
    unsigned range_mm, sensor_to_axle_mm;
    S5_MOTION motion;
    bool stopped, range_valid, profile_valid, line_hit, line_junction;
} S5_AVOID_INPUT;
typedef struct {
    uint32_t phase_at, last_line_at;
    unsigned phase:4, side:2, line_samples:2;
    unsigned offset_mm:9, next_pass_mm:9, passed_mm:9;
} S5_AVOID;
typedef struct { S5_ACTION action; unsigned distance_mm; } S5_AVOID_OUTPUT;

void s5_avoid_begin(S5_AVOID *state, uint32_t now);
S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in);
#endif
