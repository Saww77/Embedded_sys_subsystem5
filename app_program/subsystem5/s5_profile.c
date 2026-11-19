#include "s5_profile.h"

#define MAX_WIDTH_MM 2000
#define MAX_EDGE_BRACKET_STEPS 2u
#define WIDTH_RAY_TOLERANCE_MM 8u
#define WIDTH_MIN_EDGE_RISE_MM 5u

_Static_assert(S5_CLEAR_MM <= S5_MAX_PROJECTED_FRONT_MM, "Front projection must cover the obstacle window");

/* sin and cos for 0..60 degrees in steps of 5 degrees, scaled by 1024. */
static const unsigned short sine[] = {0, 89, 178, 265, 350, 433, 512, 587, 658, 724, 784, 839, 887};
static const unsigned short cosine[] = {1024, 1020, 1008, 989, 962, 928, 886, 838, 784, 724, 658, 587, 512};

static unsigned angle_steps(unsigned index)
{
    return index < S5_CENTRE_INDEX ? S5_CENTRE_INDEX - index : index - S5_CENTRE_INDEX;
}

static int lateral_mm(unsigned index, unsigned range_mm)
{
    int projection = range_mm * sine[angle_steps(index)] / 1024u;
    return index < S5_CENTRE_INDEX ? -projection : projection;
}

static unsigned project_forward_mm(unsigned index, unsigned range_mm)
{
    return range_mm * cosine[angle_steps(index)] / 1024u;
}

static int edge_bound_mm(unsigned bearing, unsigned obstacle_depth)
{
    unsigned angle = angle_steps(bearing);
    int projection = (obstacle_depth * sine[angle] + cosine[angle] - 1u) / cosine[angle];
    return bearing < S5_CENTRE_INDEX ? -projection : projection;
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

bool s5_background_beyond(unsigned range, unsigned reference)
{
    return range == S5_OPEN_RANGE ||
           (s5_range_measured(range) && range > reference + S5_PROFILE_TOLERANCE_MM);
}

typedef struct {
    unsigned last_hit, boundary, clearance;
} S5_EDGE;

static unsigned radial_extent_mm(unsigned range, unsigned face_depth)
{
    if (range <= face_depth) return 0;
    unsigned square = (range - face_depth) * (range + face_depth);
    unsigned low = 0, high = range;
    while (low < high) {
        unsigned midpoint = (low + high + 1u) / 2u;
        if (midpoint * midpoint <= square) low = midpoint;
        else high = midpoint - 1u;
    }
    return low;
}

static unsigned face_ray_range(unsigned bearing, unsigned face_depth)
{
    unsigned projection = cosine[angle_steps(bearing)];
    return (face_depth * 1024u + projection / 2u) / projection;
}

/* Estimate one side of a flat face whose confirmed centre echo gives its
 * perpendicular depth. The echo can come from anywhere inside the beam. */
static bool width_extent_mm(const S5_SCAN *scan, S5_EDGE edge, unsigned face_depth,
                            int step, unsigned *extent)
{
    if (edge.last_hit == S5_CENTRE_INDEX) return false;
    unsigned inner = (unsigned)((int)edge.last_hit - step);
    unsigned range = s5_scan_get(scan, edge.last_hit);
    unsigned inner_range = s5_scan_get(scan, inner);
    unsigned expected = face_ray_range(edge.last_hit, face_depth);
    if (!s5_same_obstacle(inner_range, face_depth) || range < face_depth + WIDTH_MIN_EDGE_RISE_MM ||
        range + WIDTH_RAY_TOLERANCE_MM < inner_range || range > expected + WIDTH_RAY_TOLERANCE_MM)
        return false;

    if (range + WIDTH_RAY_TOLERANCE_MM >= expected) {
        unsigned inner_expected = face_ray_range(inner, face_depth);
        if (inner_range + WIDTH_RAY_TOLERANCE_MM < inner_expected ||
            inner_range > inner_expected + WIDTH_RAY_TOLERANCE_MM)
            return false;
        /* Adjacent readings agree with the aimed rays. Centre the edge in
         * the hit-to-background bracket on the confirmed face plane. */
        int last = edge_bound_mm(edge.last_hit, face_depth);
        int beyond = edge_bound_mm(edge.boundary, face_depth);
        *extent = (unsigned)((last + beyond) * step + 1) / 2u;
        return true;
    }

    /* A short off-axis echo cannot be on the aimed ray at this depth.
     * Use slant range to the face, interpolating half a bracket step from
     * the measured inward neighbour. Reject abrupt jumps between echoes. */
    if (range > inner_range + S5_FINE_REPEAT_TOLERANCE_MM) return false;
    unsigned lateral = radial_extent_mm(range, face_depth);
    unsigned previous = radial_extent_mm(inner_range, face_depth);
    unsigned gap = (unsigned)(((int)edge.boundary - (int)edge.last_hit) * step);
    *extent = lateral + (lateral > previous ? (lateral - previous) * gap / 2u : 0);
    return true;
}

static S5_EDGE profile_edge(const S5_SCAN *scan, unsigned anchor, unsigned limit, int step)
{
    S5_EDGE edge = {.last_hit = anchor, .clearance = S5_UNKNOWN_RANGE};
    unsigned reference = s5_scan_get(scan, anchor);
    for (int i = (int)anchor; i != (int)limit;) {
        i += step;
        unsigned range = s5_scan_get(scan, (unsigned)i);
        if (!s5_range_available(range)) continue;
        if (s5_background_beyond(range, reference)) {
            edge.boundary = (unsigned)i;
            edge.clearance = range;
            break;
        }
        if (s5_same_obstacle(range, reference)) edge.last_hit = (unsigned)i;
    }
    return edge;
}

S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor,
                      unsigned *observed_span_mm)
{
    S5_EDGE left = profile_edge(scan, anchor, first, -1);
    S5_EDGE right = profile_edge(scan, anchor, last, 1);
    unsigned near = anchor;
    unsigned reference = s5_scan_get(scan, anchor);
    unsigned front = reference;
    for (unsigned i = left.last_hit; i <= right.last_hit; ++i) {
        unsigned range = s5_scan_get(scan, i);
        if (!s5_same_obstacle(range, reference)) continue;
        if (range < s5_scan_get(scan, near)) near = i;
        unsigned forward = project_forward_mm(i, range);
        if (forward < front) front = forward;
    }
    S5_PROFILE result = {
        .nearest_mm = s5_scan_get(scan, near), .angle_offset_deg = near * S5_FINE_STEP,
        .front_mm = front < S5_MAX_PROJECTED_FRONT_MM ? front : S5_MAX_PROJECTED_FRONT_MM,
        .width_mm = S5_UNKNOWN_WIDTH,
        .left_mm = left.clearance,
        .right_mm = right.clearance,
        .left_path_mm = S5_UNKNOWN_RANGE,
        .right_path_mm = S5_UNKNOWN_RANGE,
    };
    /* Project the background bearing onto the depth of the adjacent obstacle face. */
    if (s5_range_available(left.clearance) && left.last_hit - left.boundary <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(left.last_hit, s5_scan_get(scan, left.last_hit));
        result.left_edge_mm = edge_bound_mm(left.boundary, depth);
        result.left_edge_valid = true;
    }
    if (s5_range_available(right.clearance) && right.boundary - right.last_hit <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(right.last_hit, s5_scan_get(scan, right.last_hit));
        result.right_edge_mm = edge_bound_mm(right.boundary, depth);
        result.right_edge_valid = true;
    }
    int left_hit = lateral_mm(left.last_hit, s5_scan_get(scan, left.last_hit));
    int right_hit = lateral_mm(right.last_hit, s5_scan_get(scan, right.last_hit));
    int span = right_hit - left_hit;
    if (observed_span_mm) *observed_span_mm = span > 0 ? (unsigned)span : 0u;
    /* The measured width is independent of the conservative motion bounds.
     * A nearer, conflicting echo or an off-centre anchor lacks a good face reference. */
    unsigned left_extent, right_extent;
    if (result.left_edge_valid && result.right_edge_valid && anchor == S5_CENTRE_INDEX &&
        result.nearest_mm + WIDTH_RAY_TOLERANCE_MM >= reference &&
        width_extent_mm(scan, left, reference, -1, &left_extent) &&
        width_extent_mm(scan, right, reference, 1, &right_extent)) {
        unsigned width = left_extent + right_extent;
        if (width > 0 && width <= MAX_WIDTH_MM) result.width_mm = width;
    }
    return result;
}
