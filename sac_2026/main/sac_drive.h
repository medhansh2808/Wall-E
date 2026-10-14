#ifndef SAC_DRIVE_H
#define SAC_DRIVE_H
#include "esp_err.h"
#include "sra_board.h"
#include "sac_controller.h"

typedef struct {
    motor_handle_t left;
    motor_handle_t right;
    sac_wheels_t applied;
} sac_drive_t;
esp_err_t sac_drive_init(sac_drive_t *d);
esp_err_t sac_drive_apply(sac_drive_t *d, sac_wheels_t requested);
esp_err_t sac_drive_stop(sac_drive_t *d);
#endif
