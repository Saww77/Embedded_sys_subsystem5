#include "vehicle_core.h"

static bool detouring(const VC_MODEL *model)
{
    return model->state == VC_AVOIDING || model->state == VC_RECOVER_LINE;
}

static void halt(VC_MODEL *model, VC_STATE state, VC_REASON reason)
{
    if (detouring(model)) model->avoidance.phase = S5_FAILED;
    model->running = false;
    model->state = state;
    model->reason = reason;
}

void vc_core_step(VC_MODEL *m, const VC_INPUT *in, VC_OUTPUT *out)
{
    *out = (VC_OUTPUT){.stop = true};
    if (in->stop) { halt(m, VC_IDLE, VC_REASON_STOP); return; }
    if (in->motor_faults) { halt(m, VC_FAULT, VC_REASON_MOTOR); return; }

    bool fresh = in->line_valid && in->line_age_ms <= VC_LINE_MAX_AGE_MS;
    bool linked = !in->link_required || in->link_ok;
    if (!m->running && in->start && in->ready && fresh && !in->line_stop &&
        linked && !in->obstacle_blocked &&
        (m->avoidance.phase != S5_FAILED || (in->avoidance.line_hit && !in->avoidance.line_junction))) {
        m->running = true;
        m->avoidance = (S5_AVOID){0};
    }
    if (!m->running) return;

    if (!linked) { halt(m, VC_FAULT, VC_REASON_LINK); return; }
    if (!in->ready) { halt(m, VC_FAULT, VC_REASON_MOTOR); return; }
    if (!fresh) { halt(m, VC_FAULT, VC_REASON_STALE_LINE); return; }
    if (in->line_stop) { halt(m, VC_IDLE, VC_REASON_LINE_STOP); return; }

    if (!detouring(m) && in->obstacle_blocked) {
        if (!in->obstacle_detected) { halt(m, VC_IDLE, VC_REASON_OBSTACLE); return; }
        s5_avoid_begin(&m->avoidance, in->avoidance.now);
        m->avoidance.reverse_available = in->avoidance.can_retrace;
        m->state = VC_AVOIDING;
        m->reason = VC_REASON_OBSTACLE;
        return; /* Stop the line-follow velocity before starting the scan. */
    }
    if (detouring(m)) {
        out->maneuver = s5_avoid_step(&m->avoidance, &in->avoidance);
        S5_PHASE phase = m->avoidance.phase;
        if (phase == S5_FAILED) { halt(m, VC_FAULT, VC_REASON_OBSTACLE); return; }
        if (phase == S5_COMPLETE) {
            m->state = VC_FOLLOW_LINE;
            m->reason = VC_REASON_NONE;
            return; /* Hand off using the next fresh line-follow command. */
        }
        m->state = phase == S5_FIND_LINE || phase == S5_CENTER_LINE ||
                   phase == S5_ALIGN_LINE || phase == S5_CHECK_LINE ?
                   VC_RECOVER_LINE : VC_AVOIDING;
        out->stop = out->maneuver.action == S5_STOP;
        return;
    }

    m->state = VC_FOLLOW_LINE;
    m->reason = VC_REASON_NONE;
    bool approach = in->avoidance.range_valid && in->avoidance.range_mm <= S5_SLOW_RANGE_MM;
    out->stop = false;
    out->left_rpm = approach && in->left_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->left_rpm;
    out->right_rpm = approach && in->right_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->right_rpm;
}
