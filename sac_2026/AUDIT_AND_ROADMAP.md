# SAC 2026 reference and implementation audit

Original reference: `SRA-VJTI/Wall-E`, notably `5_line_following/`, `2_LSA/`,
`4_PWM/` and `components/sra-board-component`. The stock demo uses thresholded
line sensing, a single PID-based forwarding mode, and no complete SAC behavior
controller. We preserve the original demos and board components unchanged.

Competition requirements from `SAC_2026.pdf` (7 pages):

| Challenge | Current implementation | Still needed |
| --- | --- | --- |
| Normal marked-line following | Analog centroid + PD + speed adjustment | Real calibration/tuning |
| Junctions in unknown final course | Sensor-mask classifier, slow scan, configurable exit priority | Robust classification from physical traces; policy may not reach END |
| Inverted path | Confirmed automatic polarity hypothesis | Physical transitions |
| Snake road | Adaptive speed and line-loss recovery | Curve/turn geometry and tested gains |
| Object detection | Optional digital GPIO adapter | Select/attach permitted forward sensor |
| Drop-zone pushing | Approach/push states with proof/timeout | Physical mechanism and verified completion signal |
| U-turn | Timed turn with line reacquisition | Encoder/IMU optional; real angle tuning |
| Checkpoints | Not implemented | Marker definition from organizers |
| END | Optional explicit digital marker, disabled | Confirm how END differs from nodes |

Competition scoring and failure conditions are described on PDF pages 5–7.
The PDF does not specify the actual final route, obstacle sensor, electronics
pinout, distinctive checkpoint or end detector, or all final time/reposition
limits. We deliberately avoid inventing those details.

## Further work when hardware becomes available

1. Record serial `mask`, raw ADC readings and error at every real junction.
2. Confirm LSA ordering and wheel orientation and lower the starting speeds.
3. Validate that `NODE_SCAN` makes branch decisions at all intersection types.
4. Confirm color inversion handling across transitions and wide nodes.
5. Select and wire obstacle/destination/END sensors or implement appropriate
   floor-marking detection based on actual rules and allowed hardware.
6. Mechanically test the caster, traction and front bumper with obstacles.
7. Run at least 10 consecutive successful trials per feature before speed
   tuning; keep failures and battery voltage in your test log.
8. If the unknown final maze topology defeats a priority rule, reconsider
   navigation policy using actual observed requirements, rather than adding
   graph search prematurely.
