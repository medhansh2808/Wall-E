#include "sac_junction.h"
#include "sac_config.h"
#include <math.h>
#include <string.h>

void sac_junction_reset(sac_junction_t *j) { memset(j, 0, sizeof(*j)); }

static void capture_edges(sac_junction_t *j, const sac_observation_t *o)
{
    unsigned m = o->active_mask;
    const bool center = (m & (1u << 2)) != 0;
    /* Requires center plus an outer-edge sensor. Curves may still mimic this;
       repeated readings and the NODE_SCAN state reduce false detections. */
    if (center && o->active_count >= 3 && (m & 0x01u)) j->left = true;
    if (center && o->active_count >= 3 && (m & 0x10u)) j->right = true;
}

bool sac_junction_observe(sac_junction_t *j, const sac_observation_t *o)
{
    unsigned m = o->active_mask;
    bool left_branch = (m & 0x05u) == 0x05u && o->active_count >= 3;
    bool right_branch = (m & 0x14u) == 0x14u && o->active_count >= 3;
    bool candidate = o->wide || left_branch || right_branch;
    if (candidate) {
        capture_edges(j, o);
        if (j->candidate_samples < SAC_NODE_CONFIRM_SAMPLES) j->candidate_samples++;
    } else {
        /* Clear transient false hints before a junction has been confirmed. */
        sac_junction_reset(j);
    }
    return j->candidate_samples >= SAC_NODE_CONFIRM_SAMPLES;
}

void sac_junction_scan(sac_junction_t *j, const sac_observation_t *o)
{
    capture_edges(j, o);
    if (o->has_line && !o->wide &&
        fabsf(o->position) <= SAC_NODE_STRAIGHT_CENTER_MAX &&
        (o->active_mask & (1u << 2))) {
        if (j->centered_samples < 100) j->centered_samples++;
        if (j->centered_samples >= 2) j->straight = true;
    } else {
        j->centered_samples = 0;
    }
}

char sac_junction_choose(const sac_junction_t *j, int policy)
{
    const char *order = policy == 0 ? "LSR" : (policy == 1 ? "RSL" : "SLR");
    for (int k = 0; k < 3; ++k) {
        switch (order[k]) {
        case 'L': if (j->left) return 'L'; break;
        case 'R': if (j->right) return 'R'; break;
        case 'S': if (j->straight) return 'S'; break;
        }
    }
    return '?';
}
