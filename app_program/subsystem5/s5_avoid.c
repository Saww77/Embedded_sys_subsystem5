#include "s5_avoid.h"

#define BODY_MARGIN_MM 135u
#define MAX_OFFSET_MM 400u
#define FOLLOW_AHEAD_MARGIN_MM 90u
#define MAX_PASS_MM 300u
#define RETRY_STEP_MM 60u
#define MOTION_SETTLE_MS 260u
#define SCAN_TIMEOUT_MS 11000u
#define PHASE_TIMEOUT_MS 6500u

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

static S5_AVOID_OUTPUT fail(S5_AVOID *state, uint32_t now)
{
    go(state, S5_FAILED, now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

void s5_avoid_begin(S5_AVOID *state, uint32_t now)
{
    *state = (S5_AVOID){.phase = S5_PROFILE_PATH, .phase_at = now};
}

static bool safe_side(unsigned clearance, unsigned offset)
{
    return s5_range_available(clearance) && offset <= MAX_OFFSET_MM &&
           clearance >= offset + S5_STOP_MM;
}

static bool choose_side(S5_AVOID *state, const S5_PROFILE *profile)
{
    if (profile->nearest_mm < S5_TURN_CLEARANCE_MM ||
        profile->nearest_mm + FOLLOW_AHEAD_MARGIN_MM > MAX_PASS_MM)
        return false;

    unsigned left_offset = BODY_MARGIN_MM + (profile->left_edge_mm < 0 ? -profile->left_edge_mm : 0);
    unsigned right_offset = BODY_MARGIN_MM + (profile->right_edge_mm > 0 ? profile->right_edge_mm : 0);
    bool left = profile->left_edge_valid && safe_side(profile->left_mm, left_offset);
    bool right = profile->right_edge_valid && safe_side(profile->right_mm, right_offset);
    if (!left && !right) return false;

    bool use_left = left && (!right || profile->left_mm > profile->right_mm ||
                            (profile->left_mm == profile->right_mm && left_offset <= right_offset));
    state->side = use_left ? S5_LEFT : S5_RIGHT;
    state->offset_mm = use_left ? left_offset : right_offset;
    state->next_pass_mm = profile->nearest_mm + FOLLOW_AHEAD_MARGIN_MM;
    return true;
}

S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (phase == S5_OFF || phase == S5_COMPLETE || phase == S5_FAILED) return wait;

    uint32_t elapsed = in->now - state->phase_at;
    if (in->motion == S5_MOTION_FAILED ||
        elapsed >= (phase == S5_PROFILE_PATH ? SCAN_TIMEOUT_MS : PHASE_TIMEOUT_MS))
        return fail(state, in->now);

    bool range_fresh = in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS;
    if (phase == S5_PROFILE_PATH) {
        /* The scan belongs to this stop and has completed with the servo centred. */
        if (!in->profile_valid || in->profile_at == state->phase_at ||
            !at_or_after(in->profile_at, state->phase_at) || !in->stopped ||
            !range_fresh) return wait;
        if (in->profile.nearest_mm == S5_OPEN_RANGE)
            return go(state, S5_CHECK_LINE, in->now);
        if (choose_side(state, &in->profile)) return go(state, S5_TURN_OUT, in->now);
        if (state->reverse_available && in->line_centered && !in->line_junction) {
            state->reverse_available = false;
            return go(state, S5_BACK_UP, in->now);
        }
        return fail(state, in->now);
    }

    if (phase == S5_BACK_UP) {
        if (in->line_junction || !in->line_centered || !range_fresh) return fail(state, in->now);
        if (in->motion == S5_MOTION_DONE) return go(state, S5_PROFILE_PATH, in->now);
        if (in->motion != S5_MOTION_NONE || !in->stopped || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        return (S5_AVOID_OUTPUT){.action = S5_REVERSE, .distance_mm = S5_REVERSE_MM};
    }

    bool forward = phase == S5_SHIFT_OUT || phase == S5_PASS ||
                   phase == S5_FIND_LINE || phase == S5_CENTER_LINE;
    if (phase == S5_CHECK_LINE) {
        if (!in->stopped || !range_fresh || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        if (in->range_mm <= S5_CLEAR_MM || in->line_junction) return fail(state, in->now);
        if (!in->line_centered || !at_or_after(in->line_at, state->phase_at + MOTION_SETTLE_MS))
            return wait;
        return go(state, S5_COMPLETE, in->now);
    }
    if (forward && (in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING) &&
        (!range_fresh || in->range_mm <= S5_STOP_MM))
        return fail(state, in->now);

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
        case S5_FIND_LINE: return fail(state, in->now); /* reached search limit */
        case S5_CENTER_LINE: return go(state, S5_ALIGN_LINE, in->now);
        case S5_ALIGN_LINE: return go(state, S5_CHECK_LINE, in->now);
        default: return fail(state, in->now);
        }
    }

    if (!in->stopped || elapsed < MOTION_SETTLE_MS ||
        !range_fresh || !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
    if (!forward) {
        if (in->range_mm < S5_TURN_CLEARANCE_MM) return fail(state, in->now);
        bool outward = phase == S5_TURN_OUT || phase == S5_ALIGN_RETRY || phase == S5_ALIGN_LINE;
        bool left = outward ? state->side == S5_LEFT : state->side == S5_RIGHT;
        return (S5_AVOID_OUTPUT){.action = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT};
    }
    unsigned distance = phase == S5_SHIFT_OUT ? state->offset_mm :
                        phase == S5_PASS ? state->next_pass_mm :
                        phase == S5_CENTER_LINE ? in->sensor_to_axle_mm :
                        state->offset_mm + S5_SEARCH_MARGIN_MM;
    if (in->range_mm < distance + S5_STOP_MM) {
        if (phase != S5_FIND_LINE || state->passed_mm == MAX_PASS_MM)
            return fail(state, in->now);
        unsigned remaining = MAX_PASS_MM - state->passed_mm;
        state->next_pass_mm = remaining < RETRY_STEP_MM ? remaining : RETRY_STEP_MM;
        return go(state, S5_ALIGN_RETRY, in->now);
    }
    if (phase == S5_FIND_LINE) {
        if (in->line_hit || in->line_junction) return fail(state, in->now);
        state->last_line_at = in->line_at;
    }
    return (S5_AVOID_OUTPUT){.action = S5_ADVANCE, .distance_mm = distance};
}
