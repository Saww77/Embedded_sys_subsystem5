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
    S5_PROFILE profile = s5_profile(samples, 7, 17, 12);
    TEST_CHECK("S5-U01", profile.nearest_mm == 140 && profile.angle_deg == 90 &&
               profile.width_mm == 26 && profile.left_mm == 410 &&
               profile.right_mm == S5_UNKNOWN_RANGE,
               "profile measures nearest point and preserves missing clearance");
    profile = s5_profile(samples, 12, 17, 12);
    TEST_CHECK("S5-U02", profile.width_mm == S5_UNKNOWN_WIDTH,
               "a clipped scan does not claim an obstacle width");
    unsigned char packet[7];
    TEST_CHECK("S5-U03", s1_wire_obstacle(packet, 140, 2047, 4095, 655, 43) &&
               s1_wire_get(packet, 47, 7) == 13 && s1_wire_get(packet, 54, 2) == 0 &&
               !s1_wire_obstacle(packet, 140, 0, 0, 0, 151),
               "packed scan carries angle and rejects out-of-range values");
    TEST_SUMMARY("subsystem5");
}
#endif
