#ifndef SAC_JUNCTION_H
#define SAC_JUNCTION_H
#include <stdbool.h>
#include "sac_perception.h"

typedef struct {
    bool left;
    bool right;
    bool straight;
    unsigned candidate_samples;
    unsigned centered_samples;
} sac_junction_t;

void sac_junction_reset(sac_junction_t *j);
/* Updates branch hints; returns true after a repeated, plausible junction pattern. */
bool sac_junction_observe(sac_junction_t *j, const sac_observation_t *obs);
/* Called while creeping forward to determine whether the straight exit persists. */
void sac_junction_scan(sac_junction_t *j, const sac_observation_t *obs);
/* Returns 'L','R','S' or '?' if nothing can be established. */
char sac_junction_choose(const sac_junction_t *j, int policy);

#endif
