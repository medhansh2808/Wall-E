#ifndef SAC_CONFIG_H
#define SAC_CONFIG_H

/* Review all of these values against your physical robot before enabling the motors. */
#define SAC_MOTORS_ARMED                 0
#define SAC_CONTROL_PERIOD_MS           10
#define SAC_LOG_INTERVAL_MS             250

/* Calibrate all five sensors over WHITE, then over BLACK, when the firmware starts. */
#define SAC_CALIBRATION_SAMPLES         120
#define SAC_CALIBRATION_INTERVAL_MS     10
#define SAC_CALIBRATION_CONTRAST_MIN    200.0f
#define SAC_SENSOR_THRESHOLD            0.53f
#define SAC_SWITCH_CONFIRM_SAMPLES      6
#define SAC_INITIAL_LINE_IS_WHITE       1

/* Motor A0 is ASSUMED left, motor A1 right; verify before driving. */
#define SAC_LEFT_MOTOR_REVERSED         0
#define SAC_RIGHT_MOTOR_REVERSED        0
#define SAC_STEERING_SIGN               1.0f
#define SAC_PWM_LIMIT                  70.0f
#define SAC_PWM_SLEW_PER_TICK           7.0f

/* Conservative initial tuning, NOT measured or optimized values. */
#define SAC_KP                         10.0f
#define SAC_KD                          0.25f
#define SAC_DERIVATIVE_FILTER           0.75f
#define SAC_SPEED_STRAIGHT             53.0f
#define SAC_SPEED_CURVE_MIN            38.0f
#define SAC_CURVE_ERROR_COST            4.0f
#define SAC_CURVE_DERIVATIVE_COST       0.04f
#define SAC_TURN_PWM                   43.0f
#define SAC_CREEP_PWM                  36.0f
#define SAC_SEARCH_PWM                 31.0f
#define SAC_LOST_CONFIRM_MS           100u
#define SAC_RECOVERY_TIMEOUT_MS      1800u
#define SAC_NODE_CONFIRM_SAMPLES         3
#define SAC_NODE_COOLDOWN_MS          550u
#define SAC_NODE_CLEARANCE_MS         160u
#define SAC_MIN_PIVOT_MS              230u
#define SAC_MAX_PIVOT_MS             1100u
#define SAC_UTURN_MIN_MS              650u
#define SAC_UTURN_MAX_MS             2300u

/* Branch choices in the order nodes are ENCOUNTERED. Empty = go straight.
 * 'L' left, 'R' right, 'S' straight, 'U' U-turn. The practice map is NOT assumed
 * to be the final maze; configure this only once the competition rules allow it.
 */
#define SAC_NODE_ROUTE                  ""

/* Finish-marker semantics are not specified by SAC. Disabled by default. */
#define SAC_FINISH_MARKER_ENABLED        0
#define SAC_FINISH_AFTER_N_NODES         0
#define SAC_FINISH_WIDE_HOLD_MS        400u

/* No forward object sensor exists in the verified five-LSA hardware interface.
 * Leave this off until an approved distance sensor and its driver are integrated.
 */
#define SAC_OBSTACLE_SENSOR_READY        0
#define SAC_OBJECT_CONFIRM_MS         100u
#define SAC_OBJECT_APPROACH_MS        250u
#define SAC_PUSH_DURATION_MS         1000u

#endif
