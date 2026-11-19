#include "../../app_program/subsystem5/s5_echo.h"
#include "../../app_program/subsystem5/s5_profile.h"
#include <assert.h>
#include <stdio.h>

static void test_echo_pairing(void)
{
    S5_ECHO_CAPTURE capture = {.triggered_at = UINT32_MAX - 50};
    s5_capture_edge(&capture, UINT32_MAX - 20, true, false);
    assert(capture.state == S5_CAPTURE_IDLE);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 20, true, false);
    s5_capture_edge(&capture, 895, false, true);
    assert(capture.state == S5_CAPTURE_DONE && capture.pulse_us == 875);
    assert(s5_classify_echo(S5_ECHO_PULSE, capture.pulse_us).range_mm == 150);
    s5_capture_edge(&capture, 901, true, false);
    assert(capture.pulse_us == 875);

    capture = (S5_ECHO_CAPTURE){.triggered_at = 100, .state = S5_CAPTURE_RISE};
    s5_capture_edge(&capture, 110, false, true);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 110, true, true);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 120, true, false);
    s5_capture_edge(&capture, 30100, false, true);
    assert(capture.state == S5_CAPTURE_FALL); /* trailing edge arrived too late */
    assert(s5_classify_echo(S5_ECHO_QUIET, 0).range_mm == S5_UNKNOWN_RANGE);
    assert(s5_classify_echo(S5_ECHO_PULSE, 25000).range_mm == S5_OPEN_RANGE);
}

static void test_scan_and_profile(void)
{
    _Static_assert(sizeof(S5_SCAN) == 38, "Pack the 25 12-bit samples into 38 bytes");
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) {
        s5_scan_set(&scan, i, 4000 + i);
    }
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) assert(s5_scan_get(&scan, i) == 4000 + i);
    assert(s5_scan_target(&scan) == S5_NO_TARGET);
    s5_scan_set(&scan, 24, 140);
    assert(s5_scan_target(&scan) == 24);
    S5_PROFILE clipped = s5_profile(&scan, 23, 24, 24);
    assert(clipped.angle_offset_deg + S5_MIN_ANGLE == 150);
    assert(clipped.width_mm == S5_UNKNOWN_WIDTH && !clipped.right_edge_valid);
    s5_scan_set(&scan, 12, 150);
    assert(s5_scan_target(&scan) == 12);
    s5_scan_set(&scan, 10, 410);
    s5_scan_set(&scan, 11, 140);
    s5_scan_set(&scan, 13, 150);
    s5_scan_set(&scan, 14, S5_OPEN_RANGE);
    S5_PROFILE profile = s5_profile(&scan, 10, 14, 12);
    assert(profile.nearest_mm == 140 && profile.angle_offset_deg + S5_MIN_ANGLE == 85);
    assert(profile.width_mm == 25 && profile.right_mm == S5_OPEN_RANGE);
    s5_scan_set(&scan, 14, S5_UNKNOWN_RANGE);
    profile = s5_profile(&scan, 10, 14, 12);
    assert(profile.width_mm == S5_UNKNOWN_WIDTH && !profile.right_edge_valid);
}

int main(void)
{
    test_echo_pairing();
    test_scan_and_profile();
    puts("PASS echo pairing, expiry, scan packing, off-centre targeting and profile bounds");
}
