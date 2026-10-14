#include "sac_controller.h"
#include "sac_config.h"
#include <math.h>
#include <string.h>

static float limit(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
void sac_controller_reset(sac_controller_t *c) { memset(c, 0, sizeof(*c)); }

sac_wheels_t sac_controller_update(sac_controller_t *c, float error, float dt)
{
    if (dt < 0.004f || dt > 0.1f) dt = (float)SAC_CONTROL_PERIOD_MS / 1000.0f;
    error = limit(error, -2, 2);
    float derivative = c->initialized ? (error - c->last_error) / dt : 0;
    c->filtered_d = SAC_DERIVATIVE_FILTER * c->filtered_d +
                    (1.0f - SAC_DERIVATIVE_FILTER) * derivative;
    c->last_error = error;
    c->initialized = 1;

    float speed = limit(SAC_SPEED_STRAIGHT - SAC_CURVE_ERROR_COST * fabsf(error) -
                        SAC_CURVE_DERIVATIVE_COST * fabsf(c->filtered_d),
                        SAC_SPEED_CURVE_MIN, SAC_SPEED_STRAIGHT);
    float turn = SAC_STEERING_SIGN * (SAC_KP * error + SAC_KD * c->filtered_d);
    return (sac_wheels_t){limit(speed + turn, -SAC_PWM_LIMIT, SAC_PWM_LIMIT),
                          limit(speed - turn, -SAC_PWM_LIMIT, SAC_PWM_LIMIT)};
}
