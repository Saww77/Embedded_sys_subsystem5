# Subsystem 5 tests

## Integration

The Pico W scan task owns GP16 (trigger), GP17 (3.3 V echo input), and GP12 (50 Hz servo output on slice 6A). The boot check commands 60°, 120°, and 90° before enabling ranging. The hardware counts echo width on PWM slice 0B while the scan task sleeps.

During line following a valid forward return within 300 mm caps approach targets at 20 RPM. At 150 mm or below the controller stops, waits for the wheels to settle, and requests a coarse sweep followed by a fine scan around the forward obstacle. The scan estimates width from the lateral edges of the clustered returns and reports the adjacent side ranges.

The controller chooses the side with the larger measured clearance among routes with space for the vehicle and the estimated obstacle edge. It turns out, drives the calculated offset, realigns parallel to the original line, and passes the obstacle using 36 RPM encoder-bounded segments. It checks each heading before moving. If the return corridor is blocked, it can pass farther in short increments up to 300 mm total along the track. Two distinct IR detections during the bounded inward search trigger a stop, sensor-to-axle centering move and pivot back to the track heading. After a valid clear reading on the final heading, it resumes line following.

Two ordinary no-target pings count as open space for line following and detour clearance. The dashboard only displays measured distances. A stuck-high echo or overdue range sample still blocks motion. A signal held low can also look like an open track, so check range readings against a known target during bench validation.

STOP, stale inputs, motor faults, unconfirmed clearance, and a failed or timed-out line search end the detour stopped. An aborted detour requires fresh IR evidence and a new START. Calibrate the effective wheel track, servo direction and sensor-to-axle offset on the car before evaluating the track metrics.

Encoder pulse inputs follow the updated report. Left uses GP2 and right uses GP3.

## Data representation

| Field | Range | Bits in MQTT |
|---|---|---:|
| Closest range | 0–4000 mm | 12 |
| Estimated width | 0–2000 mm, 2047 unavailable | 11 |
| Left and right clearance | 0–4000 mm, 4095 unavailable | 12 each |
| Scan angle | 30–150°, transmitted as 0–120 | 7 |
| Ultrasonic echo fault | Flag in existing vehicle fault mask | 1 |
| Detour phase | 0–13 | 4 |
| Detour side | 0 none, 1 left, 2 right | 2 |

The profile packet grew from six bytes to seven bytes to carry the measured angle. The dashboard accepts both lengths. The producer and MQTT task share a mutex-protected vehicle snapshot. The on-wire encoding uses shifts and masks, independent of C bit-field layout.

For identical MQTT builds with Arm GCC 14.2.1 and SDK 2.2.0, ELF section sizes were:

| Build | Text | Data | BSS |
|---|---:|---:|---:|
| ZIP baseline | 366,664 B | 112 B | 82,780 B |
| Initial Subsystem 5 profiling build | 368,726 B | 112 B | 82,772 B |

The ELF counts cover static sections. The scan task also uses a kernel-allocated 2,048-byte stack at runtime.

## Results

| ID | Check | Result |
|---|---|---|
| S5-U01 | Compute nearest point, angle, width and missing clearance | Passed on host |
| S5-U02 | Leave width unavailable when the obstacle extends beyond the fine scan | Passed on host |
| S5-U03 | Pack angle and reject an out-of-bounds angle | Passed on host |
| S5-I01 | Decode the actual C encoder's output in the dashboard decoder | Passed on host |
| S5-I02 | Show valid values and missing clearance in Chromium via real Mosquitto with a simulated Pico | Passed |
| S5-I03 | Verify retained and malformed packets do not refresh obstacle data | Passed in dashboard unit suite |
| S5-H01 | Verify servo direction, echo timing, stop distance and scan accuracy using measured obstacles | Pending Pico W bench test |
| S5-H02 | Verify timing under concurrent Wi-Fi, motion and scan tasks | Pending Pico W bench test |
| S5-I04 | Simulate left and right detours, a blocked return, IR reacquisition and stop cases | Passed on host |
| S5-I05 | Display chosen side and line-reacquired stage from live MQTT packets | Passed in browser with simulated Pico |
| S5-H03 | Measure left/right bypass success, alignment, collision clearance and end-to-end duration | Pending track test |
| S5-U04 | Clear the interlock after two open pings, including a long out-of-range pulse | Passed on host |
| S5-U05 | Preserve the interlock after a single open ping following a close return or fault | Passed on host |
| S5-U06 | Reject a scan with close returns at every angle as evidence of side clearance | Passed on host |
| S5-I06 | Complete either detour with a confirmed open side and no measured width | Passed in controller simulation |
| S5-H04 | Observe the startup sweep on GP12 and check a target at known range | Pending Pico W bench test |

Run the controller and planner test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_avoidance
/tmp/test_avoidance
```
