# Subsystem 5 tests

## Integration

The Pico W scan task owns GP6 (trigger), GP7 (echo through a 5 V to 3.3 V divider), and GP12 (50 Hz servo). The hardware counts echo width on PWM slice 3B while the scan task sleeps. PWM slice 6A controls the servo. This leaves the motor encoder interrupt and motor PWM slices available to Subsystem 2.

The task ranges forward while the vehicle is running. A return at or below 150 mm blocks motion. A valid forward return above 200 mm clears the interlock. A missing or invalid forward echo keeps the interlock engaged and sets the ultrasonic echo fault. A stopped vehicle with zero measured wheel RPM can sweep 30°, 60°, 90°, 120°, and 150°, then rescan around the closest coarse return at 5° increments. Width is estimated from the lateral separation of the first and last clustered returns. Side clearances are the next observed ranges outside that cluster. The controller requires a new START after an obstacle stop in the MQTT build. Autonomous bypass and line reacquisition remain open.

## Data representation

| Field | Range | Bits in MQTT |
|---|---|---:|
| Closest range | 0–4000 mm | 12 |
| Estimated width | 0–2000 mm, 2047 unavailable | 11 |
| Left and right clearance | 0–4000 mm, 4095 unavailable | 12 each |
| Scan angle | 30–150°, transmitted as 0–120 | 7 |
| Ultrasonic echo fault | Flag in existing vehicle fault mask | 1 |

The profile packet grew from six bytes to seven bytes to carry the measured angle. The dashboard accepts both lengths. The producer and MQTT task share a mutex-protected vehicle snapshot. The on-wire encoding uses shifts and masks, independent of C bit-field layout.

For identical MQTT builds with Arm GCC 14.2.1 and SDK 2.2.0, ELF section sizes were:

| Build | Text | Data | BSS |
|---|---:|---:|---:|
| ZIP baseline | 366,664 B | 112 B | 82,780 B |
| Subsystem 5 | 368,726 B | 112 B | 82,772 B |

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
