# Subsystem 5 tests

## Integration

The Pico W scan task owns GP16 (trigger), GP17 (3.3 V echo input), and GP12 (50 Hz servo). The hardware counts echo width on PWM slice 0B while the scan task sleeps. PWM slice 6A controls the servo. This leaves the motor encoder interrupt and motor PWM slices available to Subsystem 2.

During line following a valid forward return within 300 mm caps approach targets at 20 RPM. At 150 mm or below the controller stops, waits for the wheels to settle, and requests a coarse sweep followed by a fine scan around the forward obstacle. The scan estimates width from the lateral edges of the clustered returns and reports the adjacent side ranges.

The controller chooses the side with the larger measured clearance among routes with space for the vehicle and the estimated obstacle edge. It turns out, drives the calculated offset, realigns parallel to the original line, and passes the obstacle using 36 RPM encoder-bounded segments. It checks each heading before moving. If the return corridor is blocked, it can pass farther in short increments up to 300 mm total along the track. Two distinct IR detections during the bounded inward search trigger a stop, sensor-to-axle centering move and pivot back to the track heading. After a valid clear reading on the final heading, it resumes line following.

STOP, stale inputs, motor faults, unconfirmed clearance, and a failed or timed-out line search end the detour stopped. An aborted detour requires fresh IR evidence and a new START. The ultrasonic sensor needs a measurable reflection on the relevant headings. Test initial turns, line alignment and clearance with the vehicle on a controlled track before evaluating the demonstration metrics. Calibrate the effective wheel track, servo direction and sensor-to-axle offset on the car.

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

Run the controller and planner test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_avoidance
/tmp/test_avoidance
```
