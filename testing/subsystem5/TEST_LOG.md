# Subsystem 5 tests

## Integration

The scan task owns GP16 (10 µs minimum trigger), GP17 (3.3 V echo input), and GP12 (50 Hz servo on PWM slice 6A). It shares GPIO IRQ13 with the encoders on the motion core. Timer alarm 0 ends the trigger pulse and enforces the echo deadlines. GPIO IRQ13 timestamps edges and wakes the scan task on a complete or malformed echo. Distance readings have a 30 ms deadline. A pulse that has already started can be observed for up to 60 ms. A complete 46.9–47.3 ms pulse matches the RCWL-9610A no-target response observed on the car. Two consecutive open results confirm that bearing. Missing, malformed and other late echoes are unavailable. The sweep retries after 60 ms at the same bearing. Two complete out-of-range pulses can clear the forward interlock. Some modules give no pulse or hold ECHO high past 30 ms without a target. Verify the module's open-track behavior during bring-up.

1. The forward monitor caps approach targets at 20 RPM within 350 mm. At or below 220 mm the controller stops the drive, waits for the wheels to settle, then sweeps 30°, 60°, 90°, 120°, and 150°.
2. The stationary sweep confirms the centre reading first. A forward hit within the 270 mm scan window takes priority. Otherwise the closest coarse hit supplies the bearing. The fine scan expands in 5° steps and checks each bearing with up to five pings. A missing or closer return leaves an edge unresolved. Confirmed open space requires consecutive complete echoes.
3. The sweep estimates width from the two outermost obstacle echoes when both edges are bracketed, or from the lateral span of the observed hits when an edge is missing. The firmware halves both estimates and rounds to the nearest millimetre. It samples 45° for R (right) and 135° for T (left). The planner uses the outer edge, the corrected width when an edge is uncertain, and room for the chassis when setting lateral travel.
4. The car picks the clearer usable side and favours T when both side readings are equal. It turns 90°, verifies live range and shifts sideways. It then turns 90° in the opposite direction and passes the obstacle. If there is insufficient room, the controller can retrace a verified recent approach and rescan.
5. Behind the obstacle, the car turns 90° toward the original line. When encoder progress reaches the expected vicinity, two distinct IR hits confirm black tape. The car centres its axle and turns 90° to the original heading. Fresh line and forward range data allow line following to resume or start a new sweep immediately if there is another obstacle ahead.

STOP, link or motor faults, stale inputs, and a failed or timed-out search stop the detour. After a fault, reposition the car on the line. Reset clears the stopped controller and a separate START begins the next run. The wheel track, servo direction and sensor-to-axle offset require calibration on the car. Left and right encoders use GP2 and GP3 respectively.

## Full avoidance track test

1. Build with `make -C build_make NET=lwip MQTT=1` for dashboard START and STOP. For standalone testing, build with `make -C build_make WIFI=none` and flash `build_make/mtk3pico_smp1_usb_cdc_nos4.uf2`. Standalone starts automatically when the drive is ready and IR samples are valid. Place the robot on the line before restarting it. Open USB serial at 115200.
2. Use a straight line with space on both sides for the planned offset. Put a stable, flat-front obstacle far enough ahead for the car to pass through the slow-approach band before reaching the 220 mm stop threshold. Check that it stops before sweeping. The console reports width, chosen side, turn angle, departure distance, and pass distance.
3. Check that it leaves the line on the chosen side, passes the obstacle and finds the line behind it. Confirm that it realigns to the original heading and resumes in the original direction. Repeat with the other side clear and with the obstacle already in range when pressing START. With both sides blocked, verify that it waits and retries.
4. Record the phase log and where the robot actually stopped or rejoined. The track test is complete when both routes return to the line without contacting the obstacle and the blocked-route case stops.

## Ultrasonic sensor test

From the repository root, build with `make -C build_make S5_SENSOR_TEST=1` and flash `build_make/mtk3pico_smp1_usb_cdc_s5sensor.uf2`. The test pings GP16 and measures GP17 while the servo and drive motors remain off. Open USB serial at 115200 and aim the sensor straight at a flat target.

Hold the target 20 cm and then 40 cm from the sensor. Expect the `[S5 SENSOR]` lines to show ranges near 200 mm and 400 mm respectively. Remove the target and point the sensor into open space to check for `echo=no target` and `range=open`. The log also prints raw pulse duration and any timeout or capture fault.

## Stationary hardware check

For VS Code with the C/C++ extension, open the repository root and select the `Pico W Bench` IntelliSense configuration. If the editor cannot find the compiler, set its Compiler path to the installed `arm-none-eabi-gcc` executable.

With the Pico SDK configured, run `make -C build_make S5_BENCH=1` from the repository root and flash `build_make/mtk3pico_smp1_usb_cdc_s5bench.uf2`. This profile skips Wi-Fi, MQTT and the drive tasks. It holds GP12 at 90° until the sensor reports two valid ranges at or below 220 mm. One echo timeout may fall between the two readings. It runs the production sweep, reports estimated width, and centres the servo. The log prints the proposed turn and departure distance or `plan=STOP turn=0 deg` while the drive outputs stay off. Move the target beyond 270 mm for two clear readings, or remove it for three consecutive echo timeouts, until the log says `Test rearmed`. Rearming on timeouts applies to this stationary test. USB serial at 115200 prints the detection, commanded angles, raw echoes and estimated width.

At startup, keep the box out of the sweep. The bench holds the sensor at 30°, 45°, 135°, 150° and 90° in turn, plus any enabled outer bearings. It logs each pulse setting and an empty-track range. Watch the sensor move in both directions and return to centre. If it stays still, check that the servo signal is on GP12, the servo has its rated supply voltage, and the servo supply and Pico share ground. For a width check, wait for the background check to finish, measure the box across its flat face and place it squarely in front of the centred sensor, 200–220 mm away. Allow at least 100 mm more distance to the background on both sides of the box, or use open space if the sensor returns a valid no-target pulse there. Record the width estimate and the raw sweep log separately from the straight-ahead distance.

With the head centred at 90°, verify the 30° and 150° positions point 60° right and left of straight ahead. At 45° and 135°, verify the head points along the two departure diagonals. If needed, copy `config/s5_servo_calibration.example.h` to `config/s5_servo_calibration.h` and adjust the right, centre and left pulses to suit the mounted sensor. Keep the pulses within the servo's specified range and the mount's free travel. If a similar near echo appears with the box removed, move the wire bundles and USB cable out of the sensor's sweep and repeat the bench check. If the empty track is clear but the box still fills the sweep, an outer pulse can be enabled independently on each side after verifying free movement at 75° from forward. Use `S5_SERVO_LEFT_OUTER_US` for 165° or `S5_SERVO_RIGHT_OUTER_US` for 15°. Rebuild and use the same calibration file for the bench and integrated firmware.

For a range check, hold a flat target 30–50 cm ahead. A working sensor reports about 1,750–2,920 µs and 300–500 mm while the servo stays centred. The bench prints measured clearance, open, or unavailable separately for each side. A sweep with both edges bracketed prints `width=… mm`. A partial sweep with a usable span prints `estimated_width=… mm`. A single hit can report `width=unavailable`. The `nearest_distance` field reports how far away the closest return is. The planner can use either side with verified room for the diagonal when the width is unavailable.

Measure the width across the tape-facing panel with the box in its test orientation. Turning a rectangular box changes the width presented to the car. In the integrated firmware, a sweep with an incomplete width estimate logs the edge status and its sampled `[S5] width echo` bearings at 115200 baud. The readings use 4094 for confirmed open space and 4095 for unavailable.

To check a narrow target that gives an unconfirmed or oversized width, repeat the motor-off scan with a flat face square to the sensor at about 120 mm. A wide ultrasonic beam may still return an echo beyond the true edge bearing and overestimate the width.

`rise timeout` means no rising edge was captured within 30 ms. `fall timeout` means the echo started and did not finish within 60 ms. `late fall` reports the measured pulse duration when an echo that started on time ends between 30 and 60 ms. The range remains unavailable. The observed RCWL-9610A pulse is reported as `no target` with `range=open`. `high before trigger` means GP17 was already high at the start of the ping. If repeated with a flat target straight ahead, check GP16, level-shifted GP17, sensor supply and common ground. If the servo stays still when the commanded angle changes, check the GP12 header, plug orientation and servo supply.

## Data representation

| Field | Range | Representation |
|---|---|---|
| Captured pulse and state | 0–59,999 µs, 6 capture states and 7 echo outcomes | 16 bits for pulse duration and 3 bits for state in both ISR and task storage |
| Trigger, rise and freshness timestamps | 0–2³² − 1 µs or ms, rolling | 32 bits each, using wrap-safe elapsed time |
| Sweep range and closest distance | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits per entry and closest-distance field; 12 bits on wire for measured distance |
| Sweep bearing | 30°–150° by default, optionally 15°–165°, in 5° steps | Implicit scan index. Reported bearings stay in the standard range of the 7-bit wire field. |
| Corrected width estimate | 1–1000 mm, 2047 unavailable | 11 bits in the profile and on wire |
| Obstacle front projection | 0–511 mm, saturated above 511 | 9 bits in the profile |
| Background range near each edge | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits each in the local profile |
| Departure heading range | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits each at 45° left and right in the profile and MQTT |
| Left and right lateral bounds | Signed displacement | Signed 13 bits, one validity bit and one sweep-limit bit per side |
| Forward interlock status | Open confirmation and blocked flags | 1 bit each in the range gate |
| Ultrasonic echo fault | Set or clear | 1 bit in the vehicle fault mask |
| Detour phase and side | 0–15, none/left/right | 4 and 2 bits in RAM and the low six bits of the MQTT avoidance byte |
| Wait or stop cause, phase, and forward range | 16 cause codes, phases 0–15, range 0–4095 | 4, 4 and 12 bits in the extended MQTT avoidance packet |
| Movement request and reply | 7 planner actions, 5 reply states | 3 bits each in the planner |
| Travel command distance | 0–440 mm | 9 bits in planner output |
| Pass distance and axle offset | 0–511 mm | 9 bits each in planner storage |
| Lateral bypass offset | 0–400 mm | 9 bits in planner storage. Pivots use a fixed 90° turn. |
| Crossing count, reverse authorization and input flags | 0–2 crossing samples, boolean flags | 2-bit count and one bit per flag |
| Motion queue maneuver argument | Forward 1–2000, 90° 2048–2049, reverse 2050–4049, 45° 4094–4095 | 12-bit argument within a 16-bit command |

The obstacle packet uses seven bytes. Its saved-profile flag distinguishes sweep readings from forward samples. The extended avoidance packet uses four bytes. The dashboard also accepts legacy six-byte obstacle packets and one-byte avoidance packets. A motion command uses two bytes with a separate queue timestamp. The ISR and scan task share capture state on one core. The task arms and drains it with local interrupts masked. Vehicle snapshots shared with MQTT are mutex-protected.

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
| S5-U11 | Slow the approach and stop at 220 mm, then require measured width and space for the 45° diagonal | Passed in controller simulation. |
| S5-I11 | Detect and profile a target when one timeout separates two valid readings | Passed in a host bench replay. Two timeouts or a farther reading cleared the pending detection. Open-side confirmation still required consecutive open pings. |
| S5-I12 | Boot the sensor-only test and report measured range, open space and an echo timeout | Pico build and boot-path inspection passed. A host replay printed the expected 200 mm, 400 mm, no-target and timeout reports. Physical ranging is pending. |

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

## Integrated controller and interrupt regression

| Check | Result |
|---|---|
| Controller dispatcher with simulated motor acknowledgements | Passed. Checks unavailable and confirmed open ranges, loss of range between START and dispatch, STOP priority, motor settling, scan dispatch after the motor stops, and cancellation on link loss. |
| Host echo, avoidance, and Buddy 4 safety regressions | Passed. |
| Dashboard unit and browser tests | All 26 unit tests passed. The Chromium test with Mosquitto and a simulated Pico confirmed that echo loss disables START, a clear path re-enables it, and STOP remains available. |
| Pico W firmware | MQTT with and without Subsystem 4, single-core UART, sensor-only, and stationary servo-scan profiles produced UF2 files. |
| Robot | On-car timing, forward echo and detour behavior still need measurement. |

The controller snapshot decreased from 124 to 52 bytes. The mission state and stop reason each use 3 bits, each blocker uses 4 bits, each IR darkness reading uses 10 bits, and each readiness flag uses 1 bit. The MQTT controller message is 31 bytes, its command payload is 1 byte, and the S2 queue stores 2-byte command words. The MQTT build with Subsystem 4 used 82,992 bytes of static RAM on baseline `9a56007` and 82,912 bytes after the change, measured as ELF data plus BSS with Pico SDK 2.2.0 and Arm GCC 12.2.1. These figures exclude runtime allocations.

Run the dispatcher regression from the repository root.

```sh
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 -DTM_MQTT=1 -Iinclude -Iconfig \
  testing/subsystem5/test_controller_events.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_controller_events
/tmp/test_controller_events
```

With the integrated MQTT firmware and the wheels raised, hold a flat target about 45 cm straight ahead. Check that the USB serial `[S5] forward` log reports a measured range and that START becomes available once the other checks pass. Confirm STOP clears the wheel commands. Then verify the module's empty-track response before attempting a grounded detour.

## Diagonal bypass and stopped reset

Host regressions covered both bypass sides, the measured-width requirement, a blocked return corridor, line recovery and the original heading. The S2 host check verified that each new turn uses the correct direction and 45° encoder targets. Controller and dashboard tests covered reset for a failed detour with no motor fault bits, clearing a stopped motor fault, and reporting a reset timeout. Track clearance, measured turn angles, and time to complete the photo's workflow still need measurements on the car.

## Scan retention and recovery

Controller regression replayed a failed width estimate at 208 mm followed by a 606 mm forward reading and a STOP request. The sweep and failure cause remained available through STOP. Reset cleared the detour without starting motion. The browser replay with Mosquitto showed the two distances separately and verified that Reset remained available with an older motor-mode report. Host tests also checked one stationary retry, 45° side selection, and departure after a bounded retrace. The physical width estimate and detour still need a track test.

## Flat-face width regression

A synthetic sweep modelled a 220 mm face at 209 mm with stable corner echoes extending beyond the aimed rays. The previous estimator reported 415 mm on those inputs. The current estimator reports 220 mm and retains the same outer sweep bounds for motion planning. The host tests also passed for a ray-like scan and an ambiguous sweep with no measurable edge-range change. These are simulated returns. Physical accuracy still needs a stationary check with a measured box.

The follow-up host test covers a narrow face at 215 mm with only 4–5 mm of corner-range change. It also recovers the earlier synthetic 220 mm width when one adjacent bearing misses and another outer sample sees the front instead of the corner. A scan with no verified background on one side still reports width as unavailable and records which side failed.

The two recorded track sweeps reproduced a left sweep-limit stop. At 150°, the near return was 240–245 mm, while the right sweep found background at 925–942 mm at 30°. The replay leaves the width unconfirmed and verifies the right bracket. The controller regression covers a 30 mm retrace followed by a fresh scan when authorized, and a stopped outcome when a retrace is unavailable or the second scan still reaches the limit. The measured 230–234 mm departure readings do not qualify for the detour.

The motor-off bench capture showed a 197 mm centre return, mixed near and far echoes at 135°, and five returns of 800–2013 mm at 140°. Replaying those readings kept 135° unresolved and confirmed background at 140°, within 10° of the last confirmed hit. The original replay estimated 192 mm. With the corner agreement check, the same replay leaves the width unconfirmed because the left corner ranges are inconsistent. The independent 45° departure checks reported unavailable on the left and 208 mm on the right.

A second bench capture used a measured 80 mm object and reported 212 mm. The replay confirms both edge brackets but rejects the width. The final near returns on each side are 9 mm farther than the preceding near returns, so they do not establish stable corner distances. The replay preserves the measured background and conservative motion bounds. The synthetic 87 mm and 220 mm regressions still pass.

## Law-of-cosines width verification

On 2026-11-20, the scan, avoidance and controller host regressions passed on `4f720de`. The scan regression verifies the expected 80 mm result for unequal ranges of 180 mm and 220 mm, separated by 20°. It also checks equal ranges, missing or clipped edges, and a single obstacle hit. The Pico W test and stationary bench builds produced UF2 files. [Test and build output](logs/2026-11-20_width.txt)

The recorded sweep of the physical 80 mm target yielded a 283 mm estimate on replay. The ultrasonic returns spanned 85°, so the reported distance between the two aimed rays overestimated the target's physical width. Accuracy and avoidance on the car still need a hardware run with this firmware.

## Bypass with a partial sweep

Host tests on 20 November 2026 verified lateral-span estimates, left and right detours with an unconfirmed edge, and line recovery. The dispatcher issued a right pivot after a fresh scan with a clear right departure and no width estimate. Both blocked paths and missing clearance still produced a stopped outcome. The integrated MQTT, self-test and stationary bench builds produced Pico W UF2 images. Track testing is pending. [Test and build output](logs/2026-11-20_partial_profile.txt)

## Automatic avoidance from START

The host dispatcher accepted START both with a close range and with a missing echo. A close range began a sweep and sent the chosen pivot without a forward line-follow command first. Two open pulses recovered the waiting mission without another START. Replayed track sweeps with short diagonal clearances selected the R-side lateral departure. A simulated close return after the pivot prevented sideways travel. The local Edge and Mosquitto test exercised the dashboard START and STOP commands with simulated Pico telemetry. All three Pico W builds produced UF2 images. On-car testing is pending. [Test and build output](logs/2026-11-20_automatic_avoidance.txt)

## Missing forward echo after a sweep

The host dispatcher replayed a saved sweep with a 196 mm nearest return, 294 mm estimated width and 208/651 mm left/right departure ranges. A missing forward echo held the vehicle stationary with the servo centred. A fresh forward echo resumed planning from the saved sweep and dispatched a right pivot without requesting another sweep. The planner test also verified that an expired sweep stays in the wait phase during continued echo loss. The integrated Pico W MQTT firmware built successfully. On-car testing is pending. [Test and build output](logs/2026-11-20_forward_echo.txt)

## Close obstacle with a clear departure side

The host planner replayed a 64 mm near return, a 78 mm width estimate, a clipped left edge and 68/839 mm left/right departure readings. With a verified straight approach long enough to retrace, it requested a 60 mm reverse. A fresh scan with enough turning room then selected a 90° right pivot. With no approach to retrace, an unchanged forward reading kept the servo centred. A change of at least 30 mm allowed another sweep. The scan, planner, dispatcher and 16 dashboard tests passed. The integrated Pico W MQTT image linked and generated a UF2. Track testing is pending. [Test and build output](logs/2026-11-20_close_obstacle.txt)

## Corrected width and successive 90° detours

Host tests verified that full and partial sweeps report half the previous width estimate. The planner selected T with equal usable clearances, followed the mirrored left and right 90° routes, centred on black tape, and requested a fresh sweep when another obstacle was directly ahead after returning to the line. The controller test verified dispatch of the existing 90° motor command. Pico W MQTT, stationary bench and self-test images built successfully. Track testing is pending. [Test and build output](logs/2026-11-20_90_degree_detours.txt)
