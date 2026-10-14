#ifndef SAC_NAVIGATION_H
#define SAC_NAVIGATION_H
#include <stdbool.h>
#include <stdint.h>
#include "sac_controller.h"
#include "sac_perception.h"

typedef enum {
    SAC_FOLLOW, SAC_NODE_CLEAR, SAC_NODE_TURN, SAC_RECOVER,
    SAC_OBJECT_APPROACH, SAC_OBJECT_PUSH, SAC_UTURN, SAC_FINISHED, SAC_FAULT
} sac_state_t;

typedef struct {
    sac_state_t state;
    sac_controller_t controller;
    uint32_t state_since_ms;
    uint32_t last_node_ms;
    uint32_t lost_since_ms;
    uint32_t wide_since_ms;
    uint32_t object_since_ms;
    int node_count;
    unsigned route_index;
    unsigned node_streak;
    unsigned narrow_streak;
    bool node_armed;
    bool timer_started;
    float last_error;
    char maneuver;
} sac_nav_t;

void sac_nav_init(sac_nav_t *n, uint32_t now_ms);
sac_wheels_t sac_nav_step(sac_nav_t *n, const sac_observation_t *obs,
                          bool object_present, uint32_t now_ms, float dt_seconds);
const char *sac_nav_state_name(sac_state_t state);

#endif
