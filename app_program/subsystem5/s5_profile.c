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
    return s5_range_measured(range) &&
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
    /* For routing, use the first clear bearing beyond each clustered edge.
       This bounds the offset even if the opposite edge wasn't captured. */
    if (left > first && s5_range_available(result.left_mm)) {
        result.left_edge_mm = lateral_mm(left - 1, samples[left]);
        result.left_edge_valid = true;
    }
    if (right < last && s5_range_available(result.right_mm)) {
        result.right_edge_mm = lateral_mm(right + 1, samples[right]);
        result.right_edge_valid = true;
    }
    if (result.left_edge_valid && result.right_edge_valid && left < right) {
        int width = lateral_mm(right, samples[right]) - lateral_mm(left, samples[left]);
        if (width > 0 && width <= MAX_WIDTH_MM) result.width_mm = (unsigned)width;
    }
    return result;
}
