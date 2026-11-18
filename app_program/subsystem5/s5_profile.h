#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#define S5_MIN_ANGLE 30u
#define S5_CENTRE_ANGLE 90u
#define S5_MAX_ANGLE 150u
#define S5_FINE_STEP 5u
#define S5_SCAN_POINTS 25u
#define S5_UNKNOWN_RANGE 4095u
#define S5_UNKNOWN_WIDTH 2047u
#define S5_STOP_MM 150u
#define S5_CLEAR_MM 200u

typedef struct {
    unsigned nearest_mm : 12;
    unsigned width_mm : 11;
    unsigned left_mm : 12;
    unsigned right_mm : 12;
    unsigned angle_deg : 7;
} S5_PROFILE;

/* Index zero is 30 degrees, increasing to the right in 5-degree increments. */
S5_PROFILE s5_profile(const unsigned short samples[S5_SCAN_POINTS],
                      unsigned first, unsigned last, unsigned anchor);
#endif
