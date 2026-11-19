#include "../../app_program/subsystem5/s5_echo.h"
#include "../../app_program/subsystem5/s5_profile.h"
#include <assert.h>
#include <stdio.h>

static void test_echo_pairing(void)
{
    _Static_assert(sizeof(S5_PING) == 4, "The echo result fits in four bytes");
    S5_ECHO_CAPTURE capture = {.triggered_at = UINT32_MAX - 50};
    s5_capture_edge(&capture, UINT32_MAX - 20, true, false, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_IDLE);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 20, true, false, S5_ECHO_TIMEOUT_US);
    s5_capture_edge(&capture, 895, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_DONE && capture.pulse_us == 875);
    S5_PING measured = s5_capture_result(&capture, 895, true, false);
    assert(measured.state == S5_ECHO_PULSE && measured.range_mm == 150);
    s5_capture_edge(&capture, 901, true, false, S5_ECHO_TIMEOUT_US);
    assert(capture.pulse_us == 875);

    capture = (S5_ECHO_CAPTURE){.triggered_at = 100, .state = S5_CAPTURE_RISE};
    s5_capture_edge(&capture, 110, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 110, true, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 120, true, false, S5_ECHO_TIMEOUT_US);
    s5_capture_edge(&capture, 30100, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_FALL); /* trailing edge arrived too late */
    S5_PING fall = s5_capture_result(&capture, 30100, true, false);
    assert(fall.state == S5_ECHO_FALL_TIMEOUT && fall.range_mm == S5_UNKNOWN_RANGE);
    s5_capture_edge(&capture, 38120, false, true, 60000);
    S5_PING late = s5_capture_result(&capture, 38120, true, false);
    assert(late.state == S5_ECHO_LATE && late.pulse_us == 38000);
    assert(late.range_mm == S5_UNKNOWN_RANGE);
    capture.state = S5_CAPTURE_RISE;
    S5_PING rise = s5_capture_result(&capture, 30100, true, false);
    assert(rise.state == S5_ECHO_QUIET && rise.range_mm == S5_UNKNOWN_RANGE);
    assert(s5_capture_result(&capture, 30099, true, false).state == S5_ECHO_TIMING);
    capture.state = S5_CAPTURE_IDLE;
    S5_PING high = s5_capture_result(&capture, 100, true, true);
    assert(high.state == S5_ECHO_HIGH && high.range_mm == S5_UNKNOWN_RANGE);
    assert(s5_capture_result(&capture, 100, false, true).state == S5_ECHO_TIMING);
    S5_RANGE_GATE gate = {.range_mm = measured.range_mm};
    assert(s5_accept_range(&gate, fall.range_mm) && gate.blocked);
    assert(s5_classify_echo(S5_ECHO_PULSE, 25000).range_mm == S5_OPEN_RANGE);
}

static void test_rcwl_no_target(void)
{
    S5_ECHO_CAPTURE capture = {.triggered_at = 100, .state = S5_CAPTURE_RISE};
    s5_capture_edge(&capture, 400, true, false, S5_ECHO_OBSERVE_US);
    s5_capture_edge(&capture, 47495, false, true, S5_ECHO_OBSERVE_US);
    S5_PING ping = s5_capture_result(&capture, 47495, true, false);
    assert(ping.state == S5_ECHO_NO_TARGET && ping.pulse_us == 47095);
    assert(ping.range_mm == S5_OPEN_RANGE);
    assert(s5_range_from_echo(ping.pulse_us) == S5_UNKNOWN_RANGE);
    assert(s5_classify_echo(S5_ECHO_LATE, 46899).range_mm == S5_UNKNOWN_RANGE);
    assert(s5_classify_echo(S5_ECHO_LATE, 47301).range_mm == S5_UNKNOWN_RANGE);

    S5_RANGE_GATE gate = {.range_mm = 160, .blocked = true};
    assert(!s5_accept_range(&gate, ping.range_mm) && gate.blocked);
    assert(s5_accept_range(&gate, S5_UNKNOWN_RANGE) && gate.blocked);
    assert(!s5_accept_range(&gate, ping.range_mm) && gate.blocked);
    assert(s5_accept_range(&gate, ping.range_mm) && !gate.blocked);
}

static void test_scan_and_profile(void)
{
    _Static_assert(sizeof(S5_SCAN) == 38, "Pack the 25 12-bit samples into 38 bytes");
    assert(s5_servo_angle(0) == 150);
    assert(s5_servo_angle(S5_CENTRE_INDEX * S5_FINE_STEP) == 90);
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) {
        s5_scan_set(&scan, i, 4000 + i);
    }
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) assert(s5_scan_get(&scan, i) == 4000 + i);
    assert(s5_scan_target(&scan) == S5_NO_TARGET);
    s5_scan_set(&scan, 24, 140);
    assert(s5_scan_target(&scan) == 24);
    S5_PROFILE clipped = s5_profile(&scan, 23, 24, 24, 0);
    assert(s5_servo_angle(clipped.angle_offset_deg) == 30);
    assert(clipped.width_mm == S5_UNKNOWN_WIDTH && !clipped.right_edge_valid);
    s5_scan_set(&scan, 12, 150);
    assert(s5_scan_target(&scan) == 12);
    s5_scan_set(&scan, 10, 410);
    s5_scan_set(&scan, 11, 140);
    s5_scan_set(&scan, 13, 150);
    s5_scan_set(&scan, 14, S5_OPEN_RANGE);
    S5_PROFILE profile = s5_profile(&scan, 10, 14, 12, 0);
    assert(profile.nearest_mm == 140 && s5_servo_angle(profile.angle_offset_deg) == 95);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH && profile.right_mm == S5_OPEN_RANGE);
    assert(profile.left_edge_valid && profile.right_edge_valid);
    s5_scan_set(&scan, 14, S5_UNKNOWN_RANGE);
    profile = s5_profile(&scan, 10, 14, 12, 0);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH && !profile.right_edge_valid);
    s5_scan_set(&scan, 15, 550);
    profile = s5_profile(&scan, 10, 15, 12, 0);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH && profile.right_mm == 550);
    assert(profile.right_edge_valid && profile.right_edge_mm == 40);
    s5_scan_set(&scan, 15, 30); /* A much closer return cannot bracket the far edge. */
    s5_scan_set(&scan, 16, 550); /* This is too far from the last hit to estimate width. */
    profile = s5_profile(&scan, 10, 16, 12, 0);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH && !profile.right_edge_valid);
}

static void test_observed_span_without_clearance(void)
{
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) s5_scan_set(&scan, i, S5_UNKNOWN_RANGE);
    for (unsigned i = 8; i <= 15; ++i) s5_scan_set(&scan, i, 176);
    s5_scan_set(&scan, 8, 189); /* 70 degrees, the last hit on the left */
    unsigned span = 0;
    S5_PROFILE profile = s5_profile(&scan, 5, 18, S5_CENTRE_INDEX, &span);
    assert(span == 109 && profile.nearest_mm == 176);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH);
    assert(!profile.left_edge_valid && !profile.right_edge_valid);
    profile = s5_profile(&scan, S5_CENTRE_INDEX, S5_CENTRE_INDEX, S5_CENTRE_INDEX, &span);
    assert(span == 0 && profile.width_mm == S5_UNKNOWN_WIDTH);
}

static void test_repeat_and_narrow_target(void)
{
    assert(s5_confirm_scan_range(160, 170) == 160);
    assert(s5_confirm_scan_range(160, 210) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(160, S5_OPEN_RANGE) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(S5_UNKNOWN_RANGE, S5_OPEN_RANGE) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(S5_OPEN_RANGE, S5_OPEN_RANGE) == S5_OPEN_RANGE);

    S5_SCAN scan = {0};
    s5_scan_set(&scan, 11, S5_OPEN_RANGE);
    s5_scan_set(&scan, 12, 300);
    s5_scan_set(&scan, 13, S5_OPEN_RANGE);
    unsigned span;
    S5_PROFILE profile = s5_profile(&scan, 11, 13, 12, &span);
    assert(span == 0 && profile.width_mm == S5_UNKNOWN_WIDTH && profile.front_mm == 300);
    assert(profile.left_edge_mm == -27 && profile.right_edge_mm == 27);
}

static void test_box_width_from_ray_brackets(void)
{
    S5_SCAN scan = {0};
    /* A 220 mm face at 209 mm has edges between the 25 and 30 degree rays. */
    const unsigned face_ranges[] = {209, 210, 212, 216, 222, 231, S5_OPEN_RANGE};
    for (unsigned i = 0; i < sizeof(face_ranges) / sizeof(*face_ranges); ++i) {
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, face_ranges[i]);
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, face_ranges[i]);
    }
    S5_PROFILE profile = s5_profile(&scan, 6, 18, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 220);
}

static void test_box_width_with_wide_beam(void)
{
    S5_SCAN scan = {0};
    /* Synthetic 220 mm face with stable corner echoes across wide bearings.
     * The old angular projection reports 415 mm for these same samples. */
    const unsigned left[] = {209, 209, 209, 209, 210, 213, 219, 224, 227, 229, 229, S5_OPEN_RANGE};
    const unsigned right[] = {209, 209, 209, 209, 210, 214, 221, 231, 240, 245, 245, 245, S5_OPEN_RANGE};
    for (unsigned i = 0; i < sizeof(left) / sizeof(*left); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, left[i]);
    for (unsigned i = 0; i < sizeof(right) / sizeof(*right); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, right[i]);
    S5_PROFILE profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 220 && profile.nearest_mm == 209);
    assert(profile.left_edge_mm == -211 && profile.right_edge_mm == 243);

    /* The edges also need distance evidence. The same wide bracket with a
       flat stream of echoes cannot identify the width. */
    for (unsigned i = 2; i < 24; ++i) s5_scan_set(&scan, i, 209);
    profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH);
}

int main(void)
{
    test_echo_pairing();
    test_rcwl_no_target();
    test_scan_and_profile();
    test_observed_span_without_clearance();
    test_repeat_and_narrow_target();
    test_box_width_from_ray_brackets();
    test_box_width_with_wide_beam();
    puts("PASS echo capture, scan packing, and 220 mm width with aimed rays and broad echoes");
}
