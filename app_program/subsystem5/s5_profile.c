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

void s5_scan_set(S5_SCAN *scan, unsigned index, unsigned range)
{
    unsigned offset = index * 3u / 2u;
    if (index & 1u) {
        scan->bytes[offset] = (scan->bytes[offset] & 15u) | ((range & 15u) << 4u);
        scan->bytes[offset + 1u] = range >> 4u;
    } else {
        scan->bytes[offset] = range & 255u;
        scan->bytes[offset + 1u] = (scan->bytes[offset + 1u] & 240u) | (range >> 8u);
    }
}

unsigned s5_scan_get(const S5_SCAN *scan, unsigned index)
{
    unsigned offset = index * 3u / 2u;
    unsigned word = scan->bytes[offset] | ((unsigned)scan->bytes[offset + 1u] << 8u);
    return (word >> ((index & 1u) * 4u)) & 4095u;
}

unsigned s5_scan_target(const S5_SCAN *scan)
{
    /* Give priority to a return directly in the vehicle's path. */
    if (s5_scan_get(scan, S5_CENTRE_INDEX) <= S5_CLEAR_MM) return S5_CENTRE_INDEX;
    unsigned target = S5_NO_TARGET;
    unsigned closest = S5_CLEAR_MM + 1;
    for (unsigned i = 0; i < S5_SCAN_POINTS; i += S5_COARSE_STEP) {
        unsigned range = s5_scan_get(scan, i);
        if (range < closest) {
            target = i;
            closest = range;
        }
    }
    return target;
}

bool s5_same_obstacle(unsigned range, unsigned reference)
{
    return s5_range_measured(range) && s5_range_measured(reference) &&
           range <= reference + S5_PROFILE_TOLERANCE_MM &&
           reference <= range + S5_PROFILE_TOLERANCE_MM;
}

typedef struct {
    unsigned last_hit, boundary, clearance;
} S5_EDGE;

static S5_EDGE profile_edge(const S5_SCAN *scan, unsigned anchor, unsigned limit, int step)
{
    S5_EDGE edge = {.last_hit = anchor, .clearance = S5_UNKNOWN_RANGE};
    unsigned reference = s5_scan_get(scan, anchor);
    for (int i = (int)anchor; i != (int)limit;) {
        i += step;
        unsigned range = s5_scan_get(scan, (unsigned)i);
        if (!s5_range_available(range)) continue;
        if (!s5_same_obstacle(range, reference)) {
            edge.boundary = (unsigned)i;
            edge.clearance = range;
            break;
        }
        edge.last_hit = (unsigned)i;
    }
    return edge;
}

S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor)
{
    S5_EDGE left = profile_edge(scan, anchor, first, -1);
    S5_EDGE right = profile_edge(scan, anchor, last, 1);
    unsigned near = anchor;
    for (unsigned i = left.last_hit; i <= right.last_hit; ++i) {
        if (s5_scan_get(scan, i) < s5_scan_get(scan, near)) near = i;
    }
    S5_PROFILE result = {
        .nearest_mm = s5_scan_get(scan, near), .angle_offset_deg = near * S5_FINE_STEP,
        .width_mm = S5_UNKNOWN_WIDTH,
        .left_mm = left.clearance,
        .right_mm = right.clearance
    };
    /* The first usable bearing beyond each edge supplies a conservative lateral bound. */
    if (s5_range_available(left.clearance)) {
        result.left_edge_mm = lateral_mm(left.boundary, s5_scan_get(scan, left.last_hit));
        result.left_edge_valid = true;
    }
    if (s5_range_available(right.clearance)) {
        result.right_edge_mm = lateral_mm(right.boundary, s5_scan_get(scan, right.last_hit));
        result.right_edge_valid = true;
    }
    if (result.left_edge_valid && result.right_edge_valid && left.last_hit < right.last_hit) {
        int width = lateral_mm(right.last_hit, s5_scan_get(scan, right.last_hit)) -
                    lateral_mm(left.last_hit, s5_scan_get(scan, left.last_hit));
        if (width > 0 && width <= MAX_WIDTH_MM) result.width_mm = (unsigned)width;
    }
    return result;
}
