#include "subsystem5.h"
#include "s5_profile.h"
#include "../subsystem1/s1_wire.h"
#include "../test_report.h"

#if APP_TEST
EXPORT void subsystem5_test(void)
{
    TEST_INIT();
    unsigned short samples[S5_SCAN_POINTS];
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) samples[i] = S5_UNKNOWN_RANGE;
    samples[10] = 410;
    samples[11] = 150;
    samples[12] = 140;
    samples[13] = 150;
    samples[14] = S5_OPEN_RANGE;
    S5_PROFILE profile = s5_profile(samples, 7, 17, 12);
    TEST_CHECK("S5-U01", profile.nearest_mm == 140 && profile.angle_deg == 90 &&
               profile.width_mm == 26 && profile.left_mm == 410 &&
               profile.right_mm == S5_OPEN_RANGE && profile.right_edge_valid,
               "profile distinguishes an open side from an electrical fault");
    profile = s5_profile(samples, 12, 17, 12);
    TEST_CHECK("S5-U02", profile.width_mm == S5_UNKNOWN_WIDTH,
               "a clipped scan does not claim an obstacle width");
    unsigned char packet[7];
    TEST_CHECK("S5-U03", s1_wire_obstacle(packet, 140, 2047, 4095, 655, 43) &&
               s1_wire_get(packet, 47, 7) == 13 && s1_wire_get(packet, 54, 2) == 0 &&
               !s1_wire_obstacle(packet, 140, 0, 0, 0, 151),
               "packed scan carries angle and rejects out-of-range values");
    S5_RANGE_GATE gate = {.range_mm = S5_UNKNOWN_RANGE, .blocked = true};
    unsigned open = s5_range_from_echo(0);
    TEST_CHECK("S5-U04", open == S5_OPEN_RANGE && s5_range_from_echo(38000) == open &&
               s5_range_from_echo(875) == 150 && !s5_accept_range(&gate, open) &&
               gate.blocked && s5_accept_range(&gate, open) && !gate.blocked,
               "two ordinary no-target pings clear the startup interlock");
    TEST_CHECK("S5-U05", s5_accept_range(&gate, 100) && gate.blocked &&
               !s5_accept_range(&gate, open) && gate.range_mm == 100 &&
               s5_accept_range(&gate, S5_UNKNOWN_RANGE) && gate.blocked &&
               !s5_accept_range(&gate, open) && gate.blocked,
               "one open return cannot clear a close obstacle or a fault");
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) samples[i] = 140;
    profile = s5_profile(samples, 0, S5_SCAN_POINTS - 1, 12);
    TEST_CHECK("S5-U06", !profile.left_edge_valid && !profile.right_edge_valid,
               "identical near readings at every angle give no confirmed exit side");
    TEST_SUMMARY("subsystem5");
}
#endif
