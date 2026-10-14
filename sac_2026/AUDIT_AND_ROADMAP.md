# SAC 2026 implementation audit and remaining work

## Repository coverage

| Original area | Competition relevance | Action |
| --- | --- | --- |
| `1_led_matrix` | Status or debugging only | Optional after driving works |
| `2_LSA` | Five-channel floor sensing | Reuse raw ADC API; replace binary threshold logic |
| `3_MPU` | Roll/pitch interface | Not used for balancing on caster chassis; do not assume read_mpu6050 gives yaw |
| `4_PWM` | Motor controls | Use proven `set_motor_speed` driver via a signed-command wrapper |
| `5_line_following` | Primary firmware starting point | New calibrated perception/PD modules, no network requirement |
| `6_self_balancing` | Two-wheel balancing demo | Not used with a passive caster |
| `components/sra-board-component` | Required hardware driver submodule | Do not modify during first iteration |
| `documentation`, install scripts | Set-up and API notes | Keep for reference; new project builds separately |

This audit is based on published examples and API references; it is not a
line-by-line inspection of the complete upstream submodule history.

## SAC rulebook traceability

| Rulebook challenge | Present code | What's still needed |
| --- | --- | --- |
| Follow line | Analog centroid and PD speed control | Physical calibration, tune PID/PWM and height |
| Nodes and junctions | Broad-patch debounce, ordered route table | Detect L/T/cross nodes on real course; map unknown maze |
| Colour-blind path | Two-polarity confidence and hysteresis | Test transitions, backgrounds and intersections |
| Checkpoints | Event logs only | Identify checkpoint marker(s) if any, verify passage |
| Obstacle detection | Driver interface stub only | Pick allowed sensor and pins, write driver, validate false positives |
| Push into drop zone | Approach/push/U-turn timed states | Confirm actual drop-zone entry and obstacle count; test mechanics |
| Snake road | Lower speed at large curvature/error | Tune LSA offset, wheel geometry and PWM on actual curve |
| END zone | Optional wide marker, disabled | Learn marker geometry, avoid confusion with ordinary nodes |
| Maximum time/repositions | Timed safe recovery, fault stop | Confirm published TBD limits; add overall mission timeout |

## Recommended next tasks ordered by expected effect

1. **Critical**: validate ESP-IDF build against checked-out submodule.
2. **Critical**: calibrate LSA, verify sensor order and both motors. Check whether
   the line is white or black at the START zone.
3. **Critical**: record raw traces of actual junctions and curvature; improve
   topology classification and prevent duplicate node events.
4. **Critical**: inspect object size, drop-zone dimensions, available pins and
   allowed sensor inventory. Add a REAL obstacle sensor and success check.
5. **Critical**: map route only when maze visibility/rules are confirmed;
   otherwise implement exploration with edge-visited tracking after proving
   junction classification and reliable U-turns.
6. **High**: determine start, checkpoint and END marker signatures independently.
7. **High**: perform 10 consecutive full dry runs without reposition or assist.
8. **High**: improve battery stability, chassis traction and pushing bumper.
9. **Medium**: encoder feedback and yaw gyro only if hardware/time permit.
10. **Low**: camera, learning-based detectors, Wi-Fi tuning UI.

## Integration notes

- Signed PWM percentages are NOT closed-loop velocity measurements.
- The default timed turns have no guarantee of 90/180-degree accuracy.
- Don't add a raw 5-V ultrasonic echo wire to an ESP32 GPIO; a voltage-safe
  interface is required if such a sensor is selected.
- `SAC_FINISH_MARKER_ENABLED` should remain off until there is evidence that a
  held-wide signature uniquely represents END instead of a junction.
- The scoring table has a 100-point STOP entry but an example uses 20. Confirm
  with organizers before making strategy decisions.
- The printed rules leave maximum assist/reposition counts and time TBD.

## Suggested division of team work

- Controls: `sac_controller.*`, tuned with measured traces.
- Perception/navigation: `sac_perception.*`, `sac_navigation.*`.
- Hardware/object task: `sac_drive.*`, `sac_obstacle.*`, bumper/sensor.
- Validation: tests, track construction, logged runs, scoring record.
