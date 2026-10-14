# SAC 2026: Wall-E competition firmware (unknown-course baseline)

An independent ESP-IDF project inside `Wall-E/sac_2026/`, based on the
existing Wall-E five-sensor LSA and two motor driver APIs. The original
`1_led_matrix/` through `6_self_balancing/` examples and component submodule
are not changed. Designed for **two driven wheels plus a caster**.

## What this version actually does

- Calibrates each of the five line sensors from measured black and white
  surface readings at boot; stops if their contrast is inadequate.
- Estimates a continuous, signed line position rather than using asymmetric
  binary weights; flags wide and disconnected patterns.
- Supports auto-inversion of white/black line polarity with confirmation.
- Uses filtered, elapsed-time-aware PD steering and adaptive speed; has
  independently signed wheel commands and bounded PWM slew.
- Detects candidate junctions from 5-bit sensor patterns, advances slowly
  to inspect them, and chooses an exit via a **simple priority policy**, not
  a pre-programmed `SAC_NODE_ROUTE` and **not micromouse/graph exploration**.
- Performs timed pivots with line reacquisition confirmation and bounded
  recovery after losing the line; stops if it cannot recover.
- Supplies optional digital GPIO adapters for object detection, drop-zone
  confirmation, and the finish marker, with dedicated approach/push/U-turn
  states. Disabled until hardware pins and marker semantics are confirmed.
- Implements event and serial diagnostic logging and CPU-only unit tests.

## Known gaps: do not call this competition ready

The competition rules specify object detection and pushing into a drop zone,
checkpoint traversal, an inverted line segment, and a sharp snake-road turn.
They do not define the final junction layout or the electrical interfaces of
an obstacle sensor or END marker. The five LSA sensors look at the floor; they
are not a verified forward obstacle detector.

* **No guaranteed unknown-maze solution.** Exit priority (straight/left/right
  or left/right-first) is a practical heuristic. On a looped or deliberately
  tricky course, it may revisit a loop or miss a required task. The physical
  junction observation window must be tested to verify branch detection.
* **Obstacle sensor not configured.** `SAC_OBJECT_GPIO=-1`. Set to a verified
  allowed ESP32 GPIO and attach a compatible digital obstacle sensor only
  after confirming wiring, voltage compatibility and interference with SRA
  board peripherals.
* **Drop zone and finish semantics unknown.** `SAC_DROPZONE_GPIO=-1`,
  `SAC_FINISH_GPIO=-1` by default. Digital inputs are optional adapters; if
  the actual arena uses a floor marking or some other signal, replace them
  with a verified detector. A timed push is *not* considered successful unless
  `SAC_ALLOW_UNVERIFIED_TIMED_PUSH` is manually enabled after physical tests.
* **Checkpoint recognition is not implemented**, since the competition rules
  do not define a uniquely distinguishable marker.
* **No encoders/IMU angle feedback** is used; turns are timed plus LSA
  confirmation. Motor wiring, polarity, LSA sensor ordering, calibration,
  steering sign, PWM values, turn durations and stop behavior need testing.
* This code **has not been compiled with your local ESP-IDF installation nor
  flashed or run on your physical board**. The earlier package compiled on
  your laptop; these additional changes require a NEW `idf.py build`.

## CPU-only setup & tests (no GPU needed)

From the repository root:

```bash
cd ~/Wall-E
make -C sac_2026/tests clean test
source ~/esp/esp-idf/export.sh
cd sac_2026
idf.py build
```

`idf.py build` does not flash the robot. Do not flash while there are
compile errors.

## Before bench testing

1. Keep `SAC_MOTORS_ARMED=0` in `main/sac_config.h` for the first flash.
2. Check five-sensor white/black calibration using the serial log.
3. Identify which motor driver output connects to each actual wheel and
   verify the motor directions with the wheels raised.
4. Verify whether line sensor element 0 is physically the **leftmost** sensor
   when looking in the robot's direction of travel. Reverse the ordering in
   the sensor adapter if not.
5. Validate that left-of-center error actually steers left with the physical
   drivetrain, correcting `SAC_STEERING_SIGN` or motor wiring as needed.
6. Calibrate gains and safe PWM limits starting at reduced speeds.
7. Observe 5-bit line sensor masks over straight lines, left/right junctions,
   crossroads and tight bends. Adjust `SAC_NODE_CONFIRM_SAMPLES`,
   `SAC_NODE_SCAN_MIN_MS`, `SAC_NODE_SCAN_MAX_MS` and policy appropriately.
8. Test inversion transitions in both directions. Check false polarity
   switches at junctions and gaps.
9. Test line-loss recovery and pivot timeouts at a safe clearance, then
   full snake-road handling and repeated obstacle runs.
10. Integrate and verify a suitable object sensor and drop-zone detector.
    Do **not** claim a successful push based only on a timer.

## Configuration reference

All parameters are in `main/sac_config.h`. The most relevant are:

- `SAC_MOTORS_ARMED`: 0 keeps motor outputs disarmed (default).
- `SAC_JUNCTION_POLICY`: 0 = L-S-R, 1 = R-S-L, 2 = S-L-R (default).
- `SAC_OBJECT_GPIO`, `SAC_DROPZONE_GPIO`, `SAC_FINISH_GPIO`: -1 disables
  input; set only after checking the SRA board pinout and available pins.
- `SAC_ALLOW_UNVERIFIED_TIMED_PUSH`: 0 by default; do not enable unless
  physical experiments justify the mode.
- `SAC_FINISH_REQUIRE_PUSH`: prevents an END marker from ending the run
  before a detected, confirmed push.

The best junction policy cannot be chosen without more detailed competition
rules and physical track trials. This firmware does **not** know the final
course and does **not** depend on memorized junction indices.

## Deployment and Git

To apply this upgrade to your existing clone, follow the command sequence in
`UPGRADE_NOTES.md` or use the separately supplied `.patch`. Prefer creating a
feature branch and opening a pull request against **your private repository**.

ESP-IDF build products (`build/`, `sdkconfig`, compiled host tests) must not be
committed. Retain the original Wall-E project's MIT license and attribution.
