#ifndef SAC_CONTROLLER_H
#define SAC_CONTROLLER_H

typedef struct {
    float last_error;
    float filtered_d;
    int initialized;
} sac_controller_t;

typedef struct {
    float left;
    float right;
} sac_wheels_t;

void sac_controller_reset(sac_controller_t *c);
sac_wheels_t sac_controller_update(sac_controller_t *c, float error, float dt_seconds);

#endif
