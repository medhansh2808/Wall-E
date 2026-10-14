#include "sac_navigation.h"
#include "sac_config.h"
#include <math.h>
#include <string.h>

static sac_wheels_t wheels(float left, float right) { return (sac_wheels_t){left,right}; }
static bool elapsed(uint32_t now, uint32_t since, uint32_t ms)
{
    return (uint32_t)(now - since) >= ms;
}
static void enter(sac_nav_t *n, sac_state_t state, uint32_t now)
{
    n->state = state;
    n->state_since_ms = now;
    n->turn_center_streak = 0;
    n->turn_departed = false;
    n->dropzone_since_ms = 0;
    sac_controller_reset(&n->controller);
}
void sac_nav_init(sac_nav_t *n, uint32_t now)
{
    memset(n, 0, sizeof(*n));
    n->state = SAC_FOLLOW;
    n->state_since_ms = now;
    n->last_node_ms = now - SAC_NODE_COOLDOWN_MS;
    n->node_armed = true;
    n->object_armed = true;
    sac_junction_reset(&n->junction);
}

static bool turn_complete(sac_nav_t *n, const sac_observation_t *obs,
                          uint32_t now, uint32_t minimum_ms)
{
    bool centered = obs->has_line && !obs->wide &&
                    fabsf(obs->position) < 0.65f && (obs->active_mask & 0x04u);
    if (!centered) n->turn_departed = true;
    if (!elapsed(now, n->state_since_ms, minimum_ms) || !n->turn_departed) {
        n->turn_center_streak = 0;
        return false;
    }
    n->turn_center_streak = centered ? n->turn_center_streak + 1 : 0;
    return n->turn_center_streak >= SAC_TURN_REACQUIRE_SAMPLES;
}

sac_wheels_t sac_nav_step(sac_nav_t *n, const sac_observation_t *obs,
                          sac_events_t ev, uint32_t now, float dt)
{
    if (n->state == SAC_FAULT || n->state == SAC_FINISHED) return wheels(0,0);
    if (obs->polarity_switched) sac_controller_reset(&n->controller);
    if (obs->has_line) n->last_error = obs->position;

    /* Re-arm only after the sensor has been clear long enough, avoiding
       repeated pushes of the same object. */
    if (n->state == SAC_FOLLOW && !ev.object_present) {
        if (!n->object_clear_since_ms) n->object_clear_since_ms = now ? now : 1;
        if (elapsed(now, n->object_clear_since_ms, SAC_OBJECT_REARM_MS))
            n->object_armed = true;
    } else n->object_clear_since_ms = 0;

    if (ev.finish_present && (!SAC_FINISH_REQUIRE_PUSH || n->pushed_count > 0)) {
        if (!n->finish_since_ms) n->finish_since_ms = now ? now : 1;
        if (elapsed(now, n->finish_since_ms, SAC_FINISH_CONFIRM_MS)) {
            enter(n, SAC_FINISHED, now);
            return wheels(0,0);
        }
        /* Halt while confirming the finish marker; do not overshoot END. */
        return wheels(0,0);
    } else n->finish_since_ms = 0;

    if (n->state == SAC_FOLLOW && obs->has_line && !obs->wide &&
        (obs->active_mask & 0x11u) == 0) {
        if (n->narrow_streak < 8) n->narrow_streak++;
        if (n->narrow_streak >= 5) n->node_armed = true;
    } else n->narrow_streak = 0;

    switch (n->state) {
    case SAC_FOLLOW:
        if (n->object_armed && ev.object_present) {
            if (!n->object_since_ms) n->object_since_ms = now ? now : 1;
            if (elapsed(now, n->object_since_ms, SAC_OBJECT_CONFIRM_MS)) {
                n->object_armed = false;
                enter(n, SAC_OBJECT_APPROACH, now);
                return wheels(0,0);
            }
        } else n->object_since_ms = 0;

        if (n->node_armed && elapsed(now, n->last_node_ms, SAC_NODE_COOLDOWN_MS)) {
            if (sac_junction_observe(&n->junction, obs)) {
                n->node_count++;
                n->node_armed = false;
                n->last_node_ms = now;
                enter(n, SAC_NODE_SCAN, now);
                return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
            }
        } else sac_junction_reset(&n->junction);

        if (obs->wide) {
            n->lost_since_ms = 0;
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (!obs->has_line) {
            if (!n->lost_since_ms) n->lost_since_ms = now ? now : 1;
            if (elapsed(now, n->lost_since_ms, SAC_LOST_CONFIRM_MS)) {
                enter(n, SAC_RECOVER, now);
                return wheels(0,0);
            }
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        n->lost_since_ms = 0;
        return sac_controller_update(&n->controller, obs->position, dt);

    case SAC_NODE_SCAN: {
        sac_junction_scan(&n->junction, obs);
        if (!elapsed(now, n->state_since_ms, SAC_NODE_SCAN_MIN_MS))
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        /* Continue briefly while over a wide intersection to disambiguate
           the straight exit; never scan forever. */
        if (obs->wide && !elapsed(now, n->state_since_ms, SAC_NODE_SCAN_MAX_MS))
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        n->maneuver = sac_junction_choose(&n->junction, SAC_JUNCTION_POLICY);
        if (n->maneuver == '?') {
            enter(n, SAC_RECOVER, now);
            return wheels(0,0);
        }
        if (n->maneuver == 'S') {
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        enter(n, SAC_NODE_TURN, now);
        return wheels(0,0);
    }
    case SAC_NODE_TURN:
        if (turn_complete(n, obs, now, SAC_MIN_PIVOT_MS)) {
            sac_junction_reset(&n->junction);
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, SAC_MAX_PIVOT_MS)) {
            enter(n, SAC_RECOVER, now);
            return wheels(0,0);
        }
        return n->maneuver == 'R' ? wheels(SAC_TURN_PWM,-SAC_TURN_PWM)
                                   : wheels(-SAC_TURN_PWM,SAC_TURN_PWM);

    case SAC_RECOVER:
        if (obs->has_line && !obs->wide && fabsf(obs->position) < 1.7f) {
            sac_junction_reset(&n->junction);
            enter(n, SAC_FOLLOW, now);
            n->lost_since_ms = 0;
            return wheels(SAC_CREEP_PWM,SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, SAC_RECOVERY_TIMEOUT_MS)) {
#if SAC_DEADEND_UTURN_ENABLED
            enter(n, SAC_UTURN, now);
#else
            enter(n, SAC_FAULT, now);
#endif
            return wheels(0,0);
        }
        return n->last_error < 0 ? wheels(-SAC_SEARCH_PWM,SAC_SEARCH_PWM)
                                 : wheels(SAC_SEARCH_PWM,-SAC_SEARCH_PWM);

    case SAC_OBJECT_APPROACH:
        if (elapsed(now, n->state_since_ms, SAC_OBJECT_APPROACH_MS)) {
            enter(n, SAC_OBJECT_PUSH, now);
            return wheels(SAC_CREEP_PWM,SAC_CREEP_PWM);
        }
        return wheels(SAC_CREEP_PWM,SAC_CREEP_PWM);

    case SAC_OBJECT_PUSH:
        if (ev.dropzone_reached) {
            if (!n->dropzone_since_ms) n->dropzone_since_ms = now ? now : 1;
            if (elapsed(now, n->dropzone_since_ms, SAC_PUSH_CONFIRM_MS)) {
                n->pushed_count++;
                enter(n, SAC_UTURN, now);
                return wheels(0,0);
            }
        } else n->dropzone_since_ms = 0;
        if (elapsed(now, n->state_since_ms, SAC_PUSH_TIMEOUT_MS)) {
#if SAC_ALLOW_UNVERIFIED_TIMED_PUSH
            n->pushed_count++; /* Only if manually enabled after physical proof. */
            enter(n, SAC_UTURN, now);
#else
            enter(n, SAC_FAULT, now);
#endif
            return wheels(0,0);
        }
        return wheels(SAC_CREEP_PWM,SAC_CREEP_PWM);

    case SAC_UTURN:
        if (turn_complete(n, obs, now, SAC_UTURN_MIN_MS)) {
            sac_junction_reset(&n->junction);
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM,SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, SAC_UTURN_MAX_MS)) {
            enter(n, SAC_RECOVER, now);
            return wheels(0,0);
        }
        return wheels(-SAC_TURN_PWM,SAC_TURN_PWM);

    default:
        enter(n, SAC_FAULT, now);
        return wheels(0,0);
    }
}

const char *sac_nav_state_name(sac_state_t s)
{
    switch (s) {
    case SAC_FOLLOW: return "FOLLOW";
    case SAC_NODE_SCAN: return "NODE_SCAN";
    case SAC_NODE_TURN: return "NODE_TURN";
    case SAC_RECOVER: return "RECOVER";
    case SAC_OBJECT_APPROACH: return "OBJECT_APPROACH";
    case SAC_OBJECT_PUSH: return "OBJECT_PUSH";
    case SAC_UTURN: return "UTURN";
    case SAC_FINISHED: return "FINISHED";
    case SAC_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}
