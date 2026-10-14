# Changes from the first SAC firmware (October 14, 2026)

This upgrade is **not** a micromouse solver. It implements a configurable,
short-horizon line-junction policy suitable for initial SAC experimentation.

## New code

- `main/sac_junction.h/.c`: candidate-junction confirmation from sensor masks,
  edge-branch capture during a controlled scan, detection of the forward exit,
  and configurable left/straight/right priority.
- `main/sac_obstacle.h/.c`: optional ESP-IDF digital GPIO inputs for object,
  drop-zone and END signals. Defaults are disabled because the actual sensors
  and wiring have not been specified.
- Revised `main/sac_navigation.h/.c`: SENSOR-DRIVEN `NODE_SCAN`, branch
  selection, pivot with departure/reacquisition checks, node rearm debounce,
  object-presence confirmation, push verification with timeout, optional
  protected completion, and guarded U-turn/END conditions.
- Revised `main/sac_perception.h/.c`: exposes the five-bit sensor activation
  mask so junction logic can observe the whole pattern.
- Revised `main/main.c`: carries validated obstacle/finish input signals into
  the navigation loop, logs selected maneuvers and confirmed pushes.
- Revised `main/sac_config.h`: added policy/timing/IO settings; removed
  `SAC_NODE_ROUTE`, `SAC_OBSTACLE_SENSOR_READY` and the speculative wide
  marker stop logic.
- Revised CMake and host tests.

## Hardware and competition caveats

All PID, speed, maneuver time and sensor thresholds remain untested defaults.
A five-element line sensor array does not uniquely identify every branching
geometry from one sample; the junction scanner is an experimental starting
point and must be tuned with the actual robot.

Obstacle pushing remains *functionally incomplete* until a real object sensor,
pusher mechanism, and defensible drop-zone confirmation are integrated. The
firmware intentionally stops on push timeout rather than claiming points.
The END zone marker and checkpoints are not sufficiently specified for an
unambiguous sensor-only detector.

## Validation in the artifact environment

`make -C tests clean test` passes the synthetic perception, PD, junction,
recovery, push and finish tests under native GCC with warnings-as-errors.
An ESP-IDF build and hardware tests must be performed on the user's laptop.
