# Subsystem 5 tests

## Integration

The scan task owns GP16 (10 µs minimum trigger), GP17 (3.3 V echo input), and GP12 (50 Hz servo on PWM slice 6A). It shares GPIO IRQ13 with the encoders on core 1. The ISR applies a 30 ms deadline to distance readings and watches an in-progress pulse for up to 60 ms. A complete 46.9–47.3 ms pulse matches the RCWL-9610A no-target response observed on the car. Two consecutive open results confirm that bearing. Missing, malformed and other late echoes are unavailable. The sweep retries after 60 ms at the same bearing. Two complete out-of-range pulses can clear the forward interlock. Some modules give no pulse or hold ECHO high past 30 ms without a target. Verify the module's open-track behavior during bring-up.

1. The forward monitor caps approach targets at 20 RPM within 450 mm. At or below 300 mm the controller stops the drive, waits for the wheels to settle, then sweeps 30°, 60°, 90°, 120°, and 150°.
2. If an obstacle lies within 350 mm, the fine scan returns to its bearing. A forward hit takes priority. Otherwise the closest coarse hit supplies the bearing. The fine scan expands in 5° steps until it brackets each edge or reaches the scan limit. After a missed bearing it checks up to two more bearings for a usable background return.
3. The profile reports nearest distance and bearing, projected width when both edges resolve, and the ranges beyond the left and right edges. A missing echo leaves the corresponding clearance unknown.
4. The planner continues after a clear sweep and a fresh forward and line check. With an obstacle, it picks a side with clearance for the car and executes encoder-bounded turns, offset and bypass. If neither side qualifies, it can reverse 30 mm once along the approach and request a new sweep. Reverse requires at least 600 ms of recent straight line following with both measured wheel speeds at or above 20 RPM and fresh line and range data. The car has forward ultrasonic coverage, so reverse relies on the recently driven path. The vehicle stops when a permitted retry or route is unavailable.
5. If the inward corridor is blocked, the vehicle can pass farther in short increments up to 500 mm along the track. It accepts two distinct IR hits after encoder progress reaches the expected line vicinity. Both outer sensors can detect the line together during the perpendicular crossing. The vehicle stops, drives the sensor-to-axle offset, pivots to the original heading, and checks fresh line and range data before handing back control. In the stock two-sensor configuration, both outer sensors must read white at handoff after a confirmed crossing. With GP27 enabled, the centre sensor must see the line.

STOP, link or motor faults, stale inputs, and a failed or timed-out search stop the detour. An aborted detour requires fresh IR evidence and a new START. The wheel track, servo direction and sensor-to-axle offset require calibration on the car. Left and right encoders use GP2 and GP3 respectively.

## Stationary hardware check

With the Pico SDK configured, run `make -C build_make S5_BENCH=1` from the repository root and flash `build_make/mtk3pico_smp1_usb_cdc_s5bench.uf2`. This profile skips Wi-Fi, MQTT and the drive tasks. It holds GP12 at 90° until the ultrasonic sensor reports two consecutive valid ranges at or below 300 mm. It runs the production sweep, reports estimated width, and centres the servo. It prints the proposed turn and angle using the driving planner. The bypass uses a 90° turn followed by the printed lateral offset. If no bypass qualifies, the log prints `plan=STOP turn=0 deg`. This is a stationary test, so the plan is printed while the drive outputs stay off. Move the target beyond 350 mm for two clear readings, or remove it for three consecutive echo timeouts, until the log says `Test rearmed`. Rearming on timeouts applies to this stationary test. USB serial at 115200 prints the detection, commanded angles, raw echoes and estimated width.

For a range check, hold a flat target 30–50 cm ahead. A working sensor reports about 1,750–2,920 µs and 300–500 mm while the servo stays centred. For a width check, keep a narrow flat-front box stationary at 25–30 cm. Use a broad, flat background about 60–80 cm from the sensor to provide valid returns beside the box. A resolved profile prints `width=… mm`. The `nearest_distance` field reports how far away the closest return is. The scan reports `width=unavailable` when it cannot bracket the target with valid echoes. When the scan sees the object across several bearings, the bench also prints `observed_echo_span`, the lateral span between its outermost valid target echoes. Confirming a bypass requires a usable clearance return on the chosen side.

`rise timeout` means no rising edge was captured within 30 ms. `fall timeout` means the echo started and did not finish within that window. `late fall` reports the measured pulse duration when an echo that started on time ends between 30 and 60 ms. The range remains unavailable. The observed RCWL-9610A pulse is reported as `no target` with `range=open`. `high before trigger` means GP17 was already high at the start of the ping. If repeated with a flat target straight ahead, check GP16, level-shifted GP17, sensor supply and common ground. If the servo stays still when the commanded angle changes, check the GP12 header, plug orientation and servo supply.

## Data representation

| Field | Range | Representation |
|---|---|---|
| Captured pulse and state | 0–59,999 µs, 6 capture states and 7 echo outcomes | 16 bits for pulse duration and 3 bits for state in both ISR and task storage |
| Trigger, rise and freshness timestamps | 0–2³² − 1 µs or ms, rolling | 32 bits each, using wrap-safe elapsed time |
| Sweep range and closest distance | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits per entry and closest-distance field; 12 bits on wire for measured distance |
| Sweep bearing | 30°–150° in 5° steps | Implicit scan index 0–24; profile and wire store degrees minus 30 in 7 bits |
| Estimated width | 0–2000 mm, 2047 unavailable | 11 bits in the profile and on wire |
| Left and right clearance | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits each; MQTT carries measured values or 4095 |
| Left and right lateral bounds | Signed displacement up to ±3464 mm | Signed 13 bits and one validity bit per bound |
| Forward interlock status | Open confirmation and blocked flags | 1 bit each in the range gate |
| Ultrasonic echo fault | Set or clear | 1 bit in the vehicle fault mask |
| Detour phase and side | 0–14, none/left/right | 4 and 2 bits in RAM and the low six bits of the MQTT avoidance byte |
| Movement request and reply | 6 planner actions, 5 reply states | 3 bits each in the planner |
| Travel, pass distance and axle offset | 0–511 mm | 9 bits each in planner storage |
| Crossing count, reverse authorization and input flags | 0–2 crossing samples, boolean flags | 2-bit count and one bit per flag |
| Motion queue maneuver argument | Forward 1–2000, pivots 2048–2049, reverse 2050–4049 | Existing 12-bit argument within a 16-bit command |

The obstacle and avoidance MQTT packets remain seven bytes and one byte respectively. The dashboard also accepts legacy six-byte obstacle packets. A motion command remains two bytes with a separate queue timestamp. The ISR and scan task share capture state on one core. The task arms and drains it with local interrupts masked. Vehicle snapshots shared with MQTT are mutex-protected.

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
| S5-H04 | Verify GP12 centres the sensor straight ahead at startup and check a target at known range | Pending Pico W bench test |
| S5-U07 | Keep a silent input interlocked at boot and reject implausibly short pulses | Passed on host |
| S5-U08 | Accept a measured echo while preserving stuck-high and timer fault behavior | Passed on host |
| S5-U09 | Accept a complete out-of-range pulse as sensor response evidence | Passed on host |
| S5-I07 | Exercise GP12 pulse selection and GP16 trigger with high ECHO using modeled RP2040 registers | Passed on host |
| S5-I08 | Wait for two near echoes before sweeping, report width, and rearm only after two clear echoes | Passed with simulated bench input |
| S5-U10 | Distinguish pre-trigger high from missing rising or falling edges | Passed on host with the range interlock check |
| S5-I09 | Retry a missed fine-scan echo at 90° and rearm after three echo timeouts | Passed with simulated bench input. The retry produced a 28 mm profile from scripted returns. |
| S5-I10 | Scan a scripted target at 280 mm with one unavailable edge bearing | Passed with simulated bench input. The next usable return resolved the edge and the profile reported 48 mm. |
| S5-U11 | Stop at 300 mm and plan a 390 mm bypass segment with verified side clearance | Passed in controller simulation. |

Run the controller and planner test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_avoidance
/tmp/test_avoidance
```

The RCWL-9610A bench replay used obstacle echoes around 160–170 mm and consecutive open pulses around 47.1 ms. It reported a 116 mm width and a 90° left turn. Host tests also confirmed that a single open pulse or an unknown echo cannot release the interlock. The updated bench and MQTT firmware built successfully. Track testing is pending.

## Workflow revision verification

Results on 19 November 2026 for firmware commit 4ae863a. The table above retains earlier test history. The current echo deadline supersedes the historical silence and late-pulse behavior.

| Check | Result |
|---|---|
| Host scan and echo regression | Passed with undefined-behavior and bounds sanitizers. Covers packed samples, off-centre/high-angle targets, clipped profiles, and late, out-of-order and simultaneous edges. |
| Host controller and planner regression | Passed with the same sanitizers. Covers both detours, bounded reverse and rescanning, clear path continuation, rejecting old scans, and confirmation of line crossing and handoff. |
| Dashboard unit suite | All 26 tests passed, including the reverse phase in the existing avoidance byte. |
| Pico MQTT, stationary bench and UART self-test firmware | All three linked and generated UF2 files. |
| Optional Buddy 1 demo profile | Both baseline and revised trees failed to link because the demo source list omits the vehicle controller referenced by telemetry. |
| Pico bench and track | Pending physical GP12/GP16/GP17, echo accuracy and open-track behavior, concurrency timing, and left/right bypass and recovery measurements. |

[Host test output](logs/2026-11-19_host.txt)

The static section comparison uses the same Pico SDK 2.2.0 and Arm GCC 14.2.1 MQTT configuration for both commits.

| Build | Text | Data | BSS | Static RAM (data + BSS) |
|---|---:|---:|---:|---:|
| 5e667d1 baseline | 359,812 B | 112 B | 82,720 B | 82,832 B |
| 4ae863a workflow revision | 361,764 B | 112 B | 82,736 B | 82,848 B |

The sweep array shrank from 50 to 38 bytes. Arm layout checks show the avoidance input shrinking from 44 to 32 bytes and its output from 8 to 2 bytes. The allocated scan task stack stays at 2,048 bytes. Static RAM grew by 16 bytes with the edge-capture and retrace state. These measurements cover representation and build size. Measure scan timing, motor-loop latency and recovery accuracy on the robot before recording system performance.

Run the scan and echo regression from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_scan.c \
  app_program/subsystem5/s5_profile.c -o /tmp/test_s5_scan
/tmp/test_s5_scan
python -m unittest discover -s dashboard/tests
```
