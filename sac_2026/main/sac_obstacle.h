#ifndef SAC_OBSTACLE_H
#define SAC_OBSTACLE_H
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool object_present;
    bool dropzone_reached;
    bool finish_present;
} sac_task_inputs_t;

/* GPIO-based adapter is optional. Every disabled input reads false. */
esp_err_t sac_obstacle_init(void);
sac_task_inputs_t sac_obstacle_read(void);

#endif
