# SAC 2026 firmware for the SRA Wall-E board

This is a **new ESP-IDF project** designed to live at `Wall-E/sac_2026/`. It keeps
SRA's original examples and git submodules untouched.

## What works in the code now

- ESP32 SRA board integration using verified `enable_line_sensor`,
  `read_line_sensor`, `enable_motor_driver`, and `set_motor_speed` APIs.
- Startup calibration of five LSA channels from known white and black surfaces;
  fails closed on inadequate contrast.
- Analog five-sensor centroid with symmetric weights and rejection of ambiguous
  disconnected patterns.
- Normal/inverted line detection using multiple consistent readings.
- Time-aware PD steering, filtered derivative, adaptive speed and PWM ramping.
- Signed wheel commands, turns, an ordered node action string, cautious search
  after losing the line, and an explicit fault-stop state.
- An optional finish-marker state, disabled until the actual END marking is known.
- Telemetry over serial, host-compiled logic tests.

## Intentionally unfinished: NOT COMPETITION-READY

**No front obstacle sensor or drop-zone confirmation has been integrated.** The
`main/sac_obstacle.c` driver currently always returns `false` and the object
state machine is disabled. SAC invalidates runs with zero successful pushes;
this is a critical blocker, not a cosmetic TODO. The timed PUSH routine is a
proposed maneuver only, not proof that the object entered the drop zone.

The actual final maze, junction styles, motor-to-wheel mapping, speed/timing,
end marker, and sensor geometry are not specified sufficiently in the SAC PDF.
`SAC_NODE_ROUTE` defaults to straight and cannot solve an unknown maze. The
present node detector recognizes broad patches; T-shaped/side branches may
need a more capable classifier informed by physical tests.

**No ESP-IDF compiler is installed in the artifact-building environment.**
The logic tests pass under native GCC. Building and flashing with your
`components/sra-board-component` submodule is the next required validation.

## Install into your existing fork

From your Ubuntu laptop, in a new development branch:

```bash
cd ~/Wall-E
git switch -c feature/sac-firmware
# Download the supplied ZIP to ~/Downloads/SAC_2026_firmware.zip
# If sac_2026 already exists, BACK IT UP OUTSIDE this repository first:
[ ! -e sac_2026 ] || mv sac_2026 "$HOME/sac_2026_backup_$(date +%Y%m%d_%H%M%S)"
unzip ~/Downloads/SAC_2026_firmware.zip -d ~/Wall-E
```

The ZIP has a top-level `sac_2026/` folder.

## Test on laptop — CPU-only, GPU not required

```bash
cd ~/Wall-E/sac_2026/tests
make test
```

## Build on laptop — CPU-only, GPU not required

```bash
source ~/esp/esp-idf/export.sh
cd ~/Wall-E/sac_2026
idf.py set-target esp32
idf.py build
```

**Read the build output.** Do not flash if the build fails. If you find a
submodule version incompatibility, share the compiler error; don't upgrade
or edit the pinned SRA component until you've identified the failure.

## Bring-up procedure — physical ESP32 required, GPU not required

1. Review `main/sac_config.h`. `SAC_MOTORS_ARMED` is intentionally **0**.
2. Compile and flash the DISARMED firmware to the board using `idf.py flash monitor`.
3. With the bot safely supported, position ALL five sensors on a white patch
   when requested in the serial log, then on black. Ensure uniform coverage.
4. Check individual ADC readings/contrast and line positions while manually
   moving the sensor across the tape. Verify left is negative, right positive.
5. Lift the drive wheels off the floor. Confirm `MOTOR_A_0` is physically the
   left wheel and `MOTOR_A_1` is right. Check the direction of both motors.
6. Fix `SAC_LEFT_MOTOR_REVERSED`, `SAC_RIGHT_MOTOR_REVERSED`, or the motor
   handle assignment if the wheels rotate incorrectly.
7. Set `SAC_MOTORS_ARMED` to **1** only after the checks. Build and flash again.
8. Test at low speed on a short straight line, then gentle curves. If steering
   diverges from the tape, immediately stop, swap `SAC_STEERING_SIGN`, and retest.
9. Test inversion, junctions, snake curve, and timed pivots. Do not trust any
   default PWM/gain/time values as calibrated measurements.
10. Confirm safe stop behavior before any obstacle experiments.

Calibration occurs on EVERY boot. Motor outputs stay at zero during it.

## Collaboration

```bash
cd ~/Wall-E
git add sac_2026
git commit -m "Add SAC 2026 competition firmware baseline"
git push -u origin feature/sac-firmware
```

Open a pull request **into your fork**, not `SRA-VJTI/Wall-E`.
Review the rules about code visibility between competing teams before publishing.

## Sources and assumptions

- Competition rules: user-provided `SAC_2026.pdf`, pages 2–7.
- Wall-E references: `2_LSA/main/main.c`, `3_MPU/main/main.c`,
  `5_line_following/main/line_following.c`, `5_line_following/CMakeLists.txt`,
  `5_line_following/main/CMakeLists.txt`.
- Hardware API reference: SRA Board Components' `lsa.h`, `motor_driver.h`,
  `sra_board.h`, and the `components/sra-board-component` submodule.
- Retains the parent MIT-licensed hardware library as an unchanged submodule.

See `AUDIT_AND_ROADMAP.md` for rule-to-feature mapping and the missing work.
