/* Host tests for the production controller and planner. */
#include "../../app_program/vehicle_controller/vehicle_core.h"
#include <assert.h>
#include <stdio.h>

static void step(VC_MODEL *model, VC_INPUT *input, VC_OUTPUT *output, unsigned ms)
{
    input->avoidance.now += ms;
    if (input->avoidance.range_valid) input->avoidance.range_at = input->avoidance.now;
    vc_core_step(model, input, output);
}

static VC_INPUT track_input(void)
{
    return (VC_INPUT){
        .ready = true, .line_valid = true, .link_required = true, .link_ok = true,
        .left_rpm = 30, .right_rpm = 30,
        .avoidance = {.stopped = true, .range_valid = true, .range_mm = 1000,
                      .sensor_to_axle_mm = 60},
    };
}

static void profile(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, S5_PROFILE scan)
{
    in->start = true;
    step(model, in, out, 20);
    assert(model->state == VC_FOLLOW_LINE && !out->stop);
    in->start = false;
    in->avoidance.range_mm = 250;
    step(model, in, out, 20);
    assert(out->left_rpm == 20 && out->right_rpm == 20);

    in->obstacle_blocked = in->obstacle_detected = true;
    in->avoidance.range_mm = 140;
    step(model, in, out, 20);
    assert(out->stop && model->state == VC_AVOIDING && model->avoidance.phase == S5_PROFILE_PATH);

    in->avoidance.profile = scan;
    in->avoidance.profile_valid = true;
    in->avoidance.profile_at = in->avoidance.now - 20;
    step(model, in, out, 20);
    assert(model->avoidance.phase == S5_PROFILE_PATH); /* ignore pre-stop scan */
    in->avoidance.profile_at = in->avoidance.now + 2000;
    step(model, in, out, 2000);
}

static S5_PROFILE obstacle(bool left)
{
    return (S5_PROFILE){.nearest_mm = 140, .width_mm = 90,
                        .left_mm = left ? 850 : 600, .right_mm = left ? 600 : 850,
                        .left_edge_mm = -50, .right_edge_mm = 40,
                        .left_edge_valid = true, .right_edge_valid = true};
}

static void start_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                       S5_ACTION action, unsigned travel)
{
    in->avoidance.motion = S5_MOTION_NONE;
    step(model, in, out, 260);
    assert(out->maneuver.action == action && out->maneuver.distance_mm == travel);
    in->avoidance.motion = S5_MOTION_PENDING;
}

static void end_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    in->avoidance.motion = S5_MOTION_DONE;
    step(model, in, out, 500);
    in->avoidance.motion = S5_MOTION_NONE;
}

static void reach_inward_turn(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, bool left, bool open_side)
{
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    S5_PROFILE scan = obstacle(left);
    if (open_side) {
        scan.width_mm = S5_UNKNOWN_WIDTH;
        scan.left_mm = left ? S5_OPEN_RANGE : S5_UNKNOWN_RANGE;
        scan.right_mm = left ? S5_UNKNOWN_RANGE : S5_OPEN_RANGE;
        scan.left_edge_valid = left;
        scan.right_edge_valid = !left;
    }
    profile(model, in, out, scan);
    assert(model->avoidance.side == (left ? S5_LEFT : S5_RIGHT));

    start_move(model, in, out, outward, 0);
    end_move(model, in, out);
    in->avoidance.range_mm = open_side ? S5_OPEN_RANGE : 1000;
    in->obstacle_blocked = in->obstacle_detected = false;
    start_move(model, in, out, S5_ADVANCE, left ? 185 : 175);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, 230);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    assert(model->state == VC_RECOVER_LINE && model->avoidance.phase == S5_FIND_LINE);
}

static void test_rejoin(bool left, bool retry, bool open_side)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    reach_inward_turn(&model, &in, &out, left, open_side);
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    if (retry) {
        in.avoidance.range_mm = 240;
        step(&model, &in, &out, 260);
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_ALIGN_RETRY);
        start_move(&model, &in, &out, outward, 0);
        end_move(&model, &in, &out);
        in.avoidance.range_mm = 1000;
        start_move(&model, &in, &out, S5_ADVANCE, 60);
        end_move(&model, &in, &out);
        start_move(&model, &in, &out, inward, 0);
        end_move(&model, &in, &out);
    }

    start_move(&model, &in, &out, S5_ADVANCE, left ? 225 : 215);
    in.avoidance.line_hit = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(!out.stop);
    step(&model, &in, &out, 20);
    assert(!out.stop); /* the same reading can't confirm the line twice */
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(out.stop && out.maneuver.action == S5_STOP && model.avoidance.phase == S5_CENTER_LINE);

    start_move(&model, &in, &out, S5_ADVANCE, 60);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    in.avoidance.range_valid = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
    step(&model, &in, &out, 20);
    assert(!out.stop && out.left_rpm == 30 && out.right_rpm == 30);
}

static void test_abort_paths(void)
{
    enum { OPERATOR_STOP, LINK_LOST, STALE_LINE, NO_ECHO, MOTOR_REJECTED, NO_LINE };
    for (unsigned cause = OPERATOR_STOP; cause <= NO_LINE; ++cause) {
        VC_MODEL model = {0};
        VC_INPUT in = track_input();
        VC_OUTPUT out;
        reach_inward_turn(&model, &in, &out, false, false);
        start_move(&model, &in, &out, S5_ADVANCE, 215);
        switch (cause) {
        case OPERATOR_STOP: in.stop = true; break;
        case LINK_LOST: in.link_ok = false; break;
        case STALE_LINE: in.line_age_ms = VC_LINE_MAX_AGE_MS + 1; break;
        case NO_ECHO: in.avoidance.range_valid = false; break;
        case MOTOR_REJECTED: in.avoidance.motion = S5_MOTION_FAILED; break;
        case NO_LINE: in.avoidance.motion = S5_MOTION_DONE; break;
        }
        step(&model, &in, &out, 20);
        assert(out.stop && !model.running && model.avoidance.phase == S5_FAILED);
        in = track_input();
        in.start = true;
        step(&model, &in, &out, 20);
        assert(out.stop && !model.running); /* repositioning needs actual IR evidence */
    }
}

static void test_unsafe_profile_and_scan_timeout(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_mm = S5_UNKNOWN_RANGE;
    scan.right_mm = 190;
    profile(&model, &in, &out, scan);
    assert(out.stop && model.avoidance.phase == S5_FAILED);

    model = (VC_MODEL){0};
    in = track_input();
    in.start = true;
    step(&model, &in, &out, 20);
    in.start = false;
    in.obstacle_blocked = in.obstacle_detected = true;
    step(&model, &in, &out, 20);
    step(&model, &in, &out, 11000);
    assert(out.stop && model.avoidance.phase == S5_FAILED);
}

static void test_reentry_limit(void)
{
    S5_AVOID model = {.phase = S5_FIND_LINE, .side = S5_LEFT,
                     .offset_mm = 185, .passed_mm = 300};
    S5_AVOID_INPUT in = {.now = 300, .range_at = 300, .stopped = true,
                         .range_valid = true, .range_mm = 240};
    S5_AVOID_OUTPUT out = s5_avoid_step(&model, &in);
    assert(out.action == S5_STOP && model.phase == S5_FAILED);
}

int main(void)
{
    test_rejoin(true, false, false);
    test_rejoin(false, true, false);
    test_rejoin(true, false, true);
    test_rejoin(false, false, true);
    test_abort_paths();
    test_unsafe_profile_and_scan_timeout();
    test_reentry_limit();
    puts("PASS obstacle avoidance, both sides, blocked return retry, recovery, and cancellation");
}
