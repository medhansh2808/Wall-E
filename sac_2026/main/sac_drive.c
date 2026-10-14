#include "sac_drive.h"
#include "sac_config.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#if SAC_MOTORS_ARMED
static float clip(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float slew(float from, float to)
{
    return from + clip(to - from, -SAC_PWM_SLEW_PER_TICK, SAC_PWM_SLEW_PER_TICK);
}
static esp_err_t set_signed(motor_handle_t m, float value, bool reverse)
{
    float pwm = clip(reverse ? -value : value, -SAC_PWM_LIMIT, SAC_PWM_LIMIT);
    if (fabsf(pwm) < 0.5f) return set_motor_speed(m, MOTOR_STOP, 0.0f);
    return set_motor_speed(m, pwm >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD, fabsf(pwm));
}
#endif
esp_err_t sac_drive_init(sac_drive_t *d)
{
    memset(d, 0, sizeof(*d));
    esp_err_t err = enable_motor_driver(&d->left, MOTOR_A_0);
    if (err != ESP_OK) return err;
    err = enable_motor_driver(&d->right, MOTOR_A_1);
    if (err != ESP_OK) return err;
    return sac_drive_stop(d);
}
esp_err_t sac_drive_stop(sac_drive_t *d)
{
    d->applied = (sac_wheels_t){0, 0};
    esp_err_t first = set_motor_speed(d->left, MOTOR_STOP, 0.0f);
    esp_err_t second = set_motor_speed(d->right, MOTOR_STOP, 0.0f);
    return first != ESP_OK ? first : second;
}
esp_err_t sac_drive_apply(sac_drive_t *d, sac_wheels_t cmd)
{
#if !SAC_MOTORS_ARMED
    (void)cmd;
    return sac_drive_stop(d);
#else
    d->applied.left = slew(d->applied.left, clip(cmd.left, -SAC_PWM_LIMIT, SAC_PWM_LIMIT));
    d->applied.right = slew(d->applied.right, clip(cmd.right, -SAC_PWM_LIMIT, SAC_PWM_LIMIT));
    esp_err_t first = set_signed(d->left, d->applied.left, SAC_LEFT_MOTOR_REVERSED);
    esp_err_t second = set_signed(d->right, d->applied.right, SAC_RIGHT_MOTOR_REVERSED);
    return first != ESP_OK ? first : second;
#endif
}
