#ifndef SAC_CONFIG_H
#define SAC_CONFIG_H

/* Competition defaults are deliberately conservative. Verify on the robot. */
#define SAC_MOTORS_ARMED                    0
#define SAC_CONTROL_PERIOD_MS             10
#define SAC_LOG_INTERVAL_MS              250

/* All five sensors must be exposed to each calibration surface at boot. */
#define SAC_CALIBRATION_SAMPLES           120
#define SAC_CALIBRATION_INTERVAL_MS       10
#define SAC_CALIBRATION_CONTRAST_MIN    200.0f
#define SAC_SENSOR_THRESHOLD            0.53f
#define SAC_SWITCH_CONFIRM_SAMPLES         6
#define SAC_INITIAL_LINE_IS_WHITE          1

/* The board's MOTOR_A_0/1 wheel assignments have not been verified. */
#define SAC_LEFT_MOTOR_REVERSED             0
#define SAC_RIGHT_MOTOR_REVERSED            0
#define SAC_STEERING_SIGN                1.0f
#define SAC_PWM_LIMIT                    70.0f
#define SAC_PWM_SLEW_PER_TICK             7.0f

/* The following values are UNTESTED starting points. */
#define SAC_KP                            10.0f
#define SAC_KD                             0.25f
#define SAC_DERIVATIVE_FILTER              0.75f
#define SAC_SPEED_STRAIGHT                53.0f
#define SAC_SPEED_CURVE_MIN               38.0f
#define SAC_CURVE_ERROR_COST               4.0f
#define SAC_CURVE_DERIVATIVE_COST          0.04f
#define SAC_TURN_PWM                      43.0f
#define SAC_CREEP_PWM                     36.0f
#define SAC_SEARCH_PWM                    31.0f
#define SAC_LOST_CONFIRM_MS              100u
#define SAC_RECOVERY_TIMEOUT_MS         1800u

/* Junctions: sensor-driven, with NO pre-mapped route. Policies:
   0: Left > Straight > Right; 1: Right > Straight > Left;
   2: Straight > Left > Right. A fixed priority is NOT a complete maze solver. */
#define SAC_JUNCTION_POLICY                 2
#define SAC_NODE_CONFIRM_SAMPLES            3
#define SAC_NODE_COOLDOWN_MS              550u
#define SAC_NODE_SCAN_MIN_MS              140u
#define SAC_NODE_SCAN_MAX_MS              360u
#define SAC_NODE_STRAIGHT_CENTER_MAX      0.80f
#define SAC_MIN_PIVOT_MS                  230u
#define SAC_MAX_PIVOT_MS                 1100u
#define SAC_UTURN_MIN_MS                  650u
#define SAC_UTURN_MAX_MS                 2300u
#define SAC_TURN_REACQUIRE_SAMPLES          3

/* If a track has true dead ends, permit bounded U-turn after failed search.
   Disabled until dead ends vs sensor dropout can be validated. */
#define SAC_DEADEND_UTURN_ENABLED           0

/* This is deliberately GPIO-neutral until your permitted sensors and pins
   are confirmed. Object sensor can be an active-high/low digital module.
   Pins -1 disable reading. Do not reuse SRA board reserved GPIO pins. */
#define SAC_OBJECT_GPIO                    -1
#define SAC_OBJECT_ACTIVE_LEVEL             1
#define SAC_DROPZONE_GPIO                  -1
#define SAC_DROPZONE_ACTIVE_LEVEL           1
#define SAC_FINISH_GPIO                    -1
#define SAC_FINISH_ACTIVE_LEVEL             1
#define SAC_OBJECT_CONFIRM_MS             100u
#define SAC_OBJECT_APPROACH_MS            250u
#define SAC_PUSH_TIMEOUT_MS              1400u
#define SAC_PUSH_CONFIRM_MS                50u
#define SAC_OBJECT_REARM_MS               350u
/* A time limit alone does NOT establish a successful object push. */
#define SAC_ALLOW_UNVERIFIED_TIMED_PUSH     0
#define SAC_FINISH_CONFIRM_MS             200u
#define SAC_FINISH_REQUIRE_PUSH             1

#endif
