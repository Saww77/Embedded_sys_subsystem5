#include "s5_profile.h"

#define MAX_WIDTH_MM 2000
#define MAX_EDGE_BRACKET_STEPS 2u
#define MIN_DISTANT_ECHOES 3u
#define WIDTH_RAY_TOLERANCE_MM 8u
#define WIDTH_MIN_EDGE_RISE_MM 3u
#define WIDTH_FACE_TOLERANCE_MM 12u
#define WIDTH_STABLE_CORNER_MM 6u
#define WIDTH_EDGE_CANDIDATES 3u
#define WIDTH_MAX_SUPPORT_GAP 3u

_Static_assert(S5_CLEAR_MM <= S5_MAX_PROJECTED_FRONT_MM, "Front projection must cover the obstacle window");

/* sin and cos for 0..75 degrees in steps of 5 degrees, scaled by 1024. */
static const unsigned short sine[] = {
    0, 89, 178, 265, 350, 433, 512, 587, 658, 724, 784, 839, 887, 928, 962, 989,
};
static const unsigned short cosine[] = {
    1024, 1020, 1008, 989, 962, 928, 886, 838, 784, 724, 658, 587, 512, 433, 350, 265,
};

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
    for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP) {
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

unsigned s5_edge_background(const unsigned *ranges, unsigned count, unsigned reference)
{
    unsigned shortest = S5_UNKNOWN_RANGE;
    unsigned previous = S5_UNKNOWN_RANGE;
    unsigned distant_echoes = 0;
    bool repeated = false;
    for (unsigned i = 0; i < count; ++i) {
        unsigned range = ranges[i];
        if (s5_range_measured(range)) {
            /* A single near return leaves the bearing in the edge bracket. */
            if (!s5_background_beyond(range, reference)) return S5_UNKNOWN_RANGE;
            ++distant_echoes;
        }
        if (s5_range_available(s5_confirm_scan_range(previous, range))) repeated = true;
        if (s5_range_available(range) && range < shortest) shortest = range;
        if (s5_range_available(range) || !s5_range_measured(previous)) previous = range;
    }
    /* Beyond the edge, separate reflectors can produce different distances. */
    return repeated || distant_echoes >= MIN_DISTANT_ECHOES ? shortest : S5_UNKNOWN_RANGE;
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

static bool width_ray_matches(unsigned range, unsigned bearing, unsigned face_depth)
{
    unsigned expected = face_ray_range(bearing, face_depth);
    return range + WIDTH_RAY_TOLERANCE_MM >= expected &&
           range <= expected + WIDTH_RAY_TOLERANCE_MM;
}

/* A repeated corner echo can be visible at multiple aimed angles. With
 * limited angular evidence, use two ranges that locate the same corner. */
static bool width_from_pair(const S5_SCAN *scan, S5_EDGE edge, unsigned depth,
                            unsigned outer, unsigned inner, int step, unsigned *extent)
{
    unsigned range = s5_scan_get(scan, outer), supporting = s5_scan_get(scan, inner);
    if (!s5_same_obstacle(supporting, depth)) return false;

    bool outermost = outer == edge.last_hit;
    bool on_ray = width_ray_matches(range, outer, depth);
    if (outermost && on_ray && width_ray_matches(supporting, inner, depth)) {
        int last = edge_bound_mm(edge.last_hit, depth);
        int beyond = edge_bound_mm(edge.boundary, depth);
        *extent = (unsigned)((last + beyond) * step + 1) / 2u;
        return true;
    }

    /* Off-axis returns can include different surfaces as the beam sweeps.
     * Require two bearings that see the same distance before using slant range. */
    if (supporting < depth + WIDTH_MIN_EDGE_RISE_MM || supporting > range + WIDTH_STABLE_CORNER_MM ||
        range > supporting + WIDTH_STABLE_CORNER_MM)
        return false;
    *extent = radial_extent_mm(range, depth);
    return true;
}

static bool width_extent_mm(const S5_SCAN *scan, S5_EDGE edge, unsigned depth,
                            int step, unsigned *extent)
{
    unsigned edge_steps = angle_steps(edge.last_hit);
    unsigned last_range = s5_scan_get(scan, edge.last_hit);
    for (unsigned i = 0; i < WIDTH_EDGE_CANDIDATES && i < edge_steps; ++i) {
        unsigned outer = (unsigned)((int)edge.last_hit - (int)i * step);
        unsigned range = s5_scan_get(scan, outer);
        if (!s5_same_obstacle(range, depth) || range < depth + WIDTH_MIN_EDGE_RISE_MM ||
            range > face_ray_range(outer, depth) + WIDTH_RAY_TOLERANCE_MM)
            continue;
        /* An inner pair can replace a closer face echo at the last bearing.
         * It cannot explain a substantially farther last hit. */
        if (last_range > range + WIDTH_STABLE_CORNER_MM) continue;
        for (unsigned gap = 1; gap <= WIDTH_MAX_SUPPORT_GAP && i + gap <= edge_steps; ++gap) {
            unsigned inner = (unsigned)((int)outer - (int)gap * step);
            if (width_from_pair(scan, edge, depth, outer, inner, step, extent)) return true;
        }
    }
    return false;
}

static unsigned width_face_depth(const S5_SCAN *scan, S5_EDGE left, S5_EDGE right, unsigned reference)
{
    if (left.last_hit > S5_CENTRE_INDEX || right.last_hit < S5_CENTRE_INDEX)
        return S5_UNKNOWN_RANGE;
    unsigned centre = s5_scan_get(scan, S5_CENTRE_INDEX);
    if (s5_same_obstacle(centre, reference)) return centre;
    if (s5_range_available(centre) || left.last_hit == S5_CENTRE_INDEX ||
        right.last_hit == S5_CENTRE_INDEX)
        return S5_UNKNOWN_RANGE;

    unsigned left_range = s5_scan_get(scan, S5_CENTRE_INDEX - 1);
    unsigned right_range = s5_scan_get(scan, S5_CENTRE_INDEX + 1);
    if (!s5_same_obstacle(left_range, reference) || !s5_same_obstacle(right_range, reference) ||
        left_range > right_range + WIDTH_STABLE_CORNER_MM ||
        right_range > left_range + WIDTH_STABLE_CORNER_MM)
        return S5_UNKNOWN_RANGE;
    /* The symmetric adjacent returns give the face depth if centre missed. */
    return project_forward_mm(S5_CENTRE_INDEX - 1, (left_range + right_range + 1u) / 2u);
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
        /* The legacy telemetry angle field represents the standard bearings. */
        unsigned bearing = s5_scan_bearing(i);
        if (bearing >= S5_MIN_ANGLE && bearing <= S5_MAX_ANGLE && range < s5_scan_get(scan, near))
            near = i;
        unsigned forward = project_forward_mm(i, range);
        if (forward < front) front = forward;
    }
    S5_PROFILE result = {
        .nearest_mm = s5_scan_get(scan, near),
        .angle_offset_deg = S5_MAX_ANGLE - s5_scan_bearing(near),
        .front_mm = front < S5_MAX_PROJECTED_FRONT_MM ? front : S5_MAX_PROJECTED_FRONT_MM,
        .width_mm = S5_UNKNOWN_WIDTH,
        .left_mm = left.clearance,
        .right_mm = right.clearance,
        .left_path_mm = S5_UNKNOWN_RANGE,
        .right_path_mm = S5_UNKNOWN_RANGE,
        .left_clipped = left.last_hit == 0,
        .right_clipped = right.last_hit == S5_SCAN_POINTS - 1,
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
    /* The width estimate is independent of the conservative motion bounds. */
    if (!result.left_edge_valid || !result.right_edge_valid) {
        result.width_status = result.left_edge_valid ? S5_WIDTH_RIGHT_EDGE :
                              result.right_edge_valid ? S5_WIDTH_LEFT_EDGE : S5_WIDTH_BOTH_EDGES;
        return result;
    }
    unsigned depth = width_face_depth(scan, left, right, reference);
    if (!s5_range_measured(depth) || result.nearest_mm + WIDTH_FACE_TOLERANCE_MM < depth) {
        result.width_status = S5_WIDTH_FACE;
        return result;
    }
    unsigned left_extent, right_extent;
    bool left_known = width_extent_mm(scan, left, depth, -1, &left_extent);
    bool right_known = width_extent_mm(scan, right, depth, 1, &right_extent);
    if (!left_known || !right_known) {
        result.width_status = left_known ? S5_WIDTH_RIGHT_ECHO :
                              right_known ? S5_WIDTH_LEFT_ECHO : S5_WIDTH_BOTH_ECHOES;
        return result;
    }
    unsigned width = left_extent + right_extent;
    if (width > 0 && width <= MAX_WIDTH_MM) result.width_mm = width;
    else result.width_status = S5_WIDTH_BOTH_ECHOES;
    return result;
}
