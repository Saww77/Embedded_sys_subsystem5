#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#include "s5_range.h"
#include <stdint.h>

#define S5_MIN_ANGLE 30u
#define S5_CENTRE_ANGLE 90u
#define S5_MAX_ANGLE 150u
#define S5_FINE_STEP 5u
#define S5_SCAN_POINTS 25u
#define S5_COARSE_STEP 6u
#define S5_CENTRE_INDEX ((S5_CENTRE_ANGLE - S5_MIN_ANGLE) / S5_FINE_STEP)
#define S5_NO_TARGET S5_SCAN_POINTS
#define S5_UNKNOWN_WIDTH 2047u
#define S5_PROFILE_TOLERANCE_MM 100u

/* Scan offsets increase from the robot's left to its right. */
static inline unsigned s5_servo_angle(unsigned scan_offset_deg)
{
    return S5_MAX_ANGLE - scan_offset_deg;
}

typedef struct {
    unsigned nearest_mm : 12;
    unsigned width_mm : 11;
    unsigned left_mm : 12;
    unsigned right_mm : 12;
    unsigned angle_offset_deg : 7;
    signed left_edge_mm : 13;
    signed right_edge_mm : 13;
    unsigned left_edge_valid : 1;
    unsigned right_edge_valid : 1;
} S5_PROFILE;

/* Index zero commands 150 degrees (left), decreasing in 5-degree increments toward the right. */
typedef struct {
    uint8_t bytes[(S5_SCAN_POINTS * 12u + 7u) / 8u];
} S5_SCAN;

void s5_scan_set(S5_SCAN *scan, unsigned index, unsigned range);
unsigned s5_scan_get(const S5_SCAN *scan, unsigned index);
unsigned s5_scan_target(const S5_SCAN *scan);
bool s5_same_obstacle(unsigned range, unsigned reference);
/* Optionally reports the lateral span between the outermost target echoes. */
S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor,
                      unsigned *observed_span_mm);
#endif
