#include "s5_profile.h"

#define MAX_WIDTH_MM 2000

static int lateral_mm(unsigned index, unsigned range_mm)
{
    /* sin(0..60 degrees) in steps of 5 degrees, scaled by 1024. */
    static const short sine[] = {0, 89, 178, 265, 350, 433, 512,
                                587, 658, 724, 784, 839, 887};
    int offset = (int)index - (S5_SCAN_POINTS - 1) / 2;
    int projection = (int)range_mm * sine[offset < 0 ? -offset : offset] / 1024;
    return offset < 0 ? -projection : projection;
}

static int same_obstacle(unsigned range, unsigned reference)
{
    return range != S5_UNKNOWN_RANGE &&
           range <= reference + S5_PROFILE_TOLERANCE_MM &&
           reference <= range + S5_PROFILE_TOLERANCE_MM;
}

S5_PROFILE s5_profile(const unsigned short samples[S5_SCAN_POINTS],
                      unsigned first, unsigned last, unsigned anchor)
{
    unsigned near = anchor, left = anchor, right = anchor;
    unsigned reference = samples[anchor];
    while (left > first && same_obstacle(samples[left - 1], reference)) --left;
    while (right < last && same_obstacle(samples[right + 1], reference)) ++right;
    for (unsigned i = left; i <= right; ++i) {
        if (samples[i] < samples[near]) near = i;
    }
    S5_PROFILE result = {
        .nearest_mm = samples[near], .angle_deg = S5_MIN_ANGLE + near * S5_FINE_STEP,
        .width_mm = S5_UNKNOWN_WIDTH,
        .left_mm = left > first ? samples[left - 1] : S5_UNKNOWN_RANGE,
        .right_mm = right < last ? samples[right + 1] : S5_UNKNOWN_RANGE
    };
    if (left > first && right < last && left < right) {
        int left_edge = lateral_mm(left, samples[left]);
        int right_edge = lateral_mm(right, samples[right]);
        int width = right_edge - left_edge;
        if (width > 0 && width <= MAX_WIDTH_MM) {
            result.width_mm = (unsigned)width;
            result.left_edge_mm = left_edge;
            result.right_edge_mm = right_edge;
        }
    }
    return result;
}
