#include "s5_avoid.h"

#define BODY_MARGIN_MM 135u
#define ESTIMATED_EDGE_MARGIN_MM 25u
#define FRONT_MARGIN_MM 15u
#define MAX_LATERAL_OFFSET_MM 400u
#define FOLLOW_AHEAD_MARGIN_MM 130u
#define RETRY_STEP_MM 60u
#define MOTION_SETTLE_MS 260u
#define SCAN_TIMEOUT_MS 14000u
#define RESCAN_INTERVAL_MS 1000u
#define RESCAN_RANGE_CHANGE_MM 30u
#define PHASE_TIMEOUT_MS 6500u

_Static_assert(S5_CLEAR_MM + FOLLOW_AHEAD_MARGIN_MM <= S5_MAX_PASS_MM,
               "The maximum profiled distance must fit the bypass budget");
_Static_assert(S5_MAX_PASS_MM < 512u, "Bypass distances must fit the 9-bit fields");
_Static_assert(MAX_LATERAL_OFFSET_MM <= 511u, "Lateral offset must fit in 9 bits");
_Static_assert(MAX_LATERAL_OFFSET_MM + S5_SEARCH_MARGIN_MM <= 511u,
               "Perpendicular line search must fit in 9 bits");

static bool at_or_after(uint32_t time, uint32_t reference)
{
    return (uint32_t)(time - reference) < UINT32_C(0x80000000);
}

static S5_AVOID_OUTPUT go(S5_AVOID *state, S5_PHASE phase, uint32_t now)
{
    state->phase = phase;
    state->phase_at = now;
    state->line_samples = 0;
    state->last_line_at = now;
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    if (state->phase == S5_FAILED) return;
    state->failure = reason;
    state->failed_phase = state->phase;
    go(state, S5_FAILED, now);
}

static S5_AVOID_OUTPUT fail(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    s5_avoid_abort(state, reason, now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

void s5_avoid_begin(S5_AVOID *state, uint32_t now)
{
    *state = (S5_AVOID){.phase = S5_PROFILE_PATH, .phase_at = now};
}

unsigned s5_reentry_detection_mm(unsigned offset_mm, unsigned sensor_to_axle_mm)
{
    unsigned margin = S5_SEARCH_MARGIN_MM + sensor_to_axle_mm;
    return offset_mm > margin ? offset_mm - margin : 0u;
}

static unsigned bypass_offset(int lateral_extent, bool edge_valid, unsigned width)
{
    unsigned extent = lateral_extent > 0 ? (unsigned)lateral_extent : 0u;
    if (!edge_valid) {
        if (width != S5_UNKNOWN_WIDTH && extent < width) extent = width;
        extent += ESTIMATED_EDGE_MARGIN_MM;
    }
    return BODY_MARGIN_MM + extent;
}

static unsigned side_range(unsigned heading, unsigned background, unsigned offset)
{
    /* Prefer the 45-degree sample. A farther background beyond the edge can
       guide the turn when that bearing still intersects the obstacle. */
    unsigned required = offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    if (s5_range_available(heading) && heading >= required) return heading;
    if (s5_range_available(background)) return background;
    return heading;
}

S5_BYPASS s5_plan_bypass(const S5_PROFILE *profile)
{
    S5_BYPASS plan = {.failure = S5_FAIL_TARGET};
    if (!s5_range_measured(profile->nearest_mm) || profile->nearest_mm > S5_CLEAR_MM)
        return plan;

    unsigned left_offset = bypass_offset(-profile->left_edge_mm, profile->left_edge_valid, profile->width_mm);
    unsigned right_offset = bypass_offset(profile->right_edge_mm, profile->right_edge_valid, profile->width_mm);
    unsigned left_path = side_range(profile->left_path_mm, profile->left_mm, left_offset);
    unsigned right_path = side_range(profile->right_path_mm, profile->right_mm, right_offset);
    bool left_room = !profile->left_clipped && left_offset <= MAX_LATERAL_OFFSET_MM;
    bool right_room = !profile->right_clipped && right_offset <= MAX_LATERAL_OFFSET_MM;
    bool left = left_room && s5_range_available(left_path) &&
                left_path >= left_offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    bool right = right_room && s5_range_available(right_path) &&
                 right_path >= right_offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    if (!left && !right) {
        if ((left_room && !s5_range_available(left_path)) ||
            (right_room && !s5_range_available(right_path))) plan.failure = S5_FAIL_ECHO;
        else if (profile->left_clipped && profile->right_clipped) plan.failure = S5_FAIL_BOTH_LIMITS;
        else if (profile->left_clipped) plan.failure = S5_FAIL_LEFT_LIMIT;
        else if (profile->right_clipped) plan.failure = S5_FAIL_RIGHT_LIMIT;
        else plan.failure = left_room || right_room ? S5_FAIL_SIDE : S5_FAIL_ROOM;
        return plan;
    }
    /* Verify turning room here and live clearance at 90 degrees before driving sideways. */
    if (profile->front_mm < S5_TURN_CLEARANCE_MM + FRONT_MARGIN_MM) {
        plan.failure = S5_FAIL_ROOM;
        return plan;
    }

    bool use_left = left && (!right || left_path >= right_path);
    plan.side = use_left ? S5_LEFT : S5_RIGHT;
    plan.failure = S5_FAIL_NONE;
    plan.offset_mm = use_left ? left_offset : right_offset;
    plan.pass_mm = profile->nearest_mm + FOLLOW_AHEAD_MARGIN_MM;
    return plan;
}

static S5_AVOID_OUTPUT wait_for_path(S5_AVOID *state, S5_FAILURE reason, const S5_AVOID_INPUT *in)
{
    state->failure = reason;
    state->failed_phase = state->phase;
    state->blocked_range_mm = S5_UNKNOWN_RANGE;
    if (in->range_valid &&
        (reason == S5_FAIL_ROOM || reason == S5_FAIL_SIDE || reason >= S5_FAIL_LEFT_LIMIT))
        state->blocked_range_mm = in->range_mm;
    return go(state, S5_WAIT_PATH, in->now);
}

static void clear_path_failure(S5_AVOID *state)
{
    state->failure = S5_FAIL_NONE;
    state->failed_phase = S5_OFF;
}

S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (phase == S5_OFF || phase == S5_COMPLETE || phase == S5_FAILED) return wait;

    uint32_t elapsed = in->now - state->phase_at;
    bool range_fresh = in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS;
    if (in->motion == S5_MOTION_FAILED) return fail(state, S5_FAIL_MOTION, in->now);
    if (phase == S5_WAIT_PATH) {
        /* The scan task keeps ranging at 90 degrees while stopped. A new
           sweep can begin once a current forward range justifies it. */
        if (!in->stopped || !range_fresh || !at_or_after(in->range_at, state->phase_at)) return wait;
        if (in->range_mm > S5_CLEAR_MM) {
            clear_path_failure(state);
            return go(state, S5_COMPLETE, in->now);
        }
        /* An unchanged obstacle gives the same geometry on the next sweep. */
        unsigned blocked = state->blocked_range_mm;
        if (s5_range_measured(blocked) && in->range_mm < blocked + RESCAN_RANGE_CHANGE_MM &&
            blocked < in->range_mm + RESCAN_RANGE_CHANGE_MM) return wait;
        if (elapsed >= RESCAN_INTERVAL_MS) {
            clear_path_failure(state);
            state->scan_retried = true;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        return wait;
    }
    if (elapsed >= (phase == S5_PROFILE_PATH ? SCAN_TIMEOUT_MS : PHASE_TIMEOUT_MS)) {
        S5_FAILURE reason = phase == S5_PROFILE_PATH && !in->profile_valid ? S5_FAIL_SCAN_TIMEOUT :
                            !range_fresh ? S5_FAIL_ECHO : S5_FAIL_PHASE_TIMEOUT;
        if (phase == S5_PROFILE_PATH ||
            ((phase == S5_TURN_OUT || phase == S5_BACK_UP) && in->motion == S5_MOTION_NONE))
            return wait_for_path(state, reason, in);
        return fail(state, reason, in->now);
    }
    if (phase == S5_PROFILE_PATH) {
        /* The scan belongs to this stop and has completed with the servo centred. */
        if (!in->profile_valid || in->profile_at == state->phase_at ||
            !at_or_after(in->profile_at, state->phase_at) || !in->stopped) return wait;
        /* Keep a completed obstacle profile while centered pings recover.
           Side clearance and the sweep's nearest hit do not measure the
           space in front of the chassis at departure time. */
        bool has_result = s5_range_available(in->profile.nearest_mm);
        if (has_result && !range_fresh) return wait;
        if (range_fresh && in->range_mm > S5_CLEAR_MM) {
            clear_path_failure(state);
            return go(state, S5_COMPLETE, in->now);
        }
        S5_PROFILE profile = in->profile;
        if (range_fresh && in->range_mm < profile.front_mm) profile.front_mm = in->range_mm;
        S5_BYPASS plan = s5_plan_bypass(&profile);
        if (plan.side != S5_NO_SIDE) {
            clear_path_failure(state);
            state->side = plan.side;
            state->offset_mm = plan.offset_mm;
            state->next_pass_mm = plan.pass_mm;
            return go(state, S5_TURN_OUT, in->now);
        }
        S5_FAILURE reason = plan.failure;
        if (!state->scan_retried && (reason == S5_FAIL_TARGET || reason == S5_FAIL_ECHO)) {
            state->scan_retried = true;
            go(state, S5_PROFILE_PATH, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_RESCAN};
        }
        bool clipped = reason >= S5_FAIL_LEFT_LIMIT;
        /* Change vantage point only if the tracked segment permits retracing and
           a clipped target should still be in profiling range after the move. */
        bool retrace_in_range = range_fresh && in->range_mm <= S5_CLEAR_MM - S5_REVERSE_MM;
        if (state->reverse_available && range_fresh && in->line_centered && !in->line_junction &&
            (!clipped || retrace_in_range)) {
            state->reverse_available = false;
            state->reverse_long = state->reverse_long && reason == S5_FAIL_ROOM &&
                                  profile.front_mm <= S5_CLEAR_MM - S5_LONG_REVERSE_MM &&
                                  range_fresh && in->range_mm <= S5_CLEAR_MM - S5_LONG_REVERSE_MM;
            return go(state, S5_BACK_UP, in->now);
        }
        return wait_for_path(state, range_fresh ? reason : S5_FAIL_ECHO, in);
    }

    if (phase == S5_BACK_UP) {
        if (in->line_junction || !in->line_centered) return fail(state, S5_FAIL_LINE, in->now);
        if (!range_fresh)
            return in->motion == S5_MOTION_NONE ? wait_for_path(state, S5_FAIL_ECHO, in) :
                                                  fail(state, S5_FAIL_ECHO, in->now);
        if (in->motion == S5_MOTION_DONE) return go(state, S5_PROFILE_PATH, in->now);
        if (in->motion != S5_MOTION_NONE || !in->stopped || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        return (S5_AVOID_OUTPUT){.action = S5_REVERSE,
                                 .distance_mm = state->reverse_long ? S5_LONG_REVERSE_MM : S5_REVERSE_MM};
    }

    bool forward = phase == S5_SHIFT_OUT || phase == S5_PASS ||
                   phase == S5_FIND_LINE || phase == S5_CENTER_LINE;
    if (phase == S5_CHECK_LINE) {
        if (!in->stopped || !range_fresh || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        if (in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        if (!in->line_centered || !at_or_after(in->line_at, state->phase_at + MOTION_SETTLE_MS))
            return wait;
        if (in->range_mm <= S5_CLEAR_MM) {
            s5_avoid_begin(state, in->now);
            return wait;
        }
        return go(state, S5_COMPLETE, in->now);
    }
    if ((in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING) &&
        (!range_fresh || in->range_mm < S5_TURN_CLEARANCE_MM ||
         (forward && in->range_mm <= S5_MANEUVER_STOP_MM)))
        return fail(state, range_fresh ? S5_FAIL_CLOSE : S5_FAIL_ECHO, in->now);

    if (phase == S5_FIND_LINE && (in->motion == S5_MOTION_PENDING || in->motion == S5_MOTION_DONE)) {
        if (in->near_original_line && at_or_after(in->line_at, state->phase_at) && in->line_at != state->last_line_at) {
            state->last_line_at = in->line_at;
            state->line_samples = in->line_hit ? state->line_samples + 1u : 0u;
        }
        if (state->line_samples >= 2) {
            go(state, S5_CENTER_LINE, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_STOP};
        }
    }

    if (in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING) return wait;
    if (in->motion == S5_MOTION_DONE) {
        switch (phase) {
        case S5_TURN_OUT: return go(state, S5_SHIFT_OUT, in->now);
        case S5_SHIFT_OUT: return go(state, S5_ALIGN_OUTSIDE, in->now);
        case S5_ALIGN_OUTSIDE: case S5_ALIGN_RETRY: return go(state, S5_PASS, in->now);
        case S5_PASS:
            state->passed_mm += state->next_pass_mm;
            return go(state, S5_TURN_IN, in->now);
        case S5_TURN_IN: return go(state, S5_FIND_LINE, in->now);
        case S5_FIND_LINE: return fail(state, S5_FAIL_LINE, in->now);
        case S5_CENTER_LINE: return go(state, S5_ALIGN_LINE, in->now);
        case S5_ALIGN_LINE: return go(state, S5_CHECK_LINE, in->now);
        default: return fail(state, S5_FAIL_MOTION, in->now);
        }
    }

    if (!in->stopped || elapsed < MOTION_SETTLE_MS ||
        !range_fresh || !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
    if (!forward) {
        if (phase == S5_TURN_OUT && in->range_mm < S5_TURN_CLEARANCE_MM + FRONT_MARGIN_MM)
            return wait_for_path(state, S5_FAIL_ROOM, in);
        if (in->range_mm < S5_TURN_CLEARANCE_MM) return fail(state, S5_FAIL_CLOSE, in->now);
        bool outward = phase == S5_TURN_OUT || phase == S5_ALIGN_RETRY || phase == S5_ALIGN_LINE;
        bool left = outward ? state->side == S5_LEFT : state->side == S5_RIGHT;
        return (S5_AVOID_OUTPUT){.action = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT};
    }
    unsigned distance = phase == S5_SHIFT_OUT ? state->offset_mm :
                        phase == S5_PASS ? state->next_pass_mm :
                        phase == S5_CENTER_LINE ? in->sensor_to_axle_mm :
                        state->offset_mm + S5_SEARCH_MARGIN_MM;
    if (in->range_mm < distance + S5_MANEUVER_STOP_MM) {
        if (phase != S5_FIND_LINE || state->passed_mm == S5_MAX_PASS_MM)
            return fail(state, S5_FAIL_CLOSE, in->now);
        unsigned remaining = S5_MAX_PASS_MM - state->passed_mm;
        state->next_pass_mm = remaining < RETRY_STEP_MM ? remaining : RETRY_STEP_MM;
        return go(state, S5_ALIGN_RETRY, in->now);
    }
    if (phase == S5_FIND_LINE) {
        if (in->line_hit || in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        state->last_line_at = in->line_at;
    }
    return (S5_AVOID_OUTPUT){.action = S5_ADVANCE, .distance_mm = distance};
}
