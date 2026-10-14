#include "sac_navigation.h"
#include "sac_config.h"
#include <math.h>
#include <string.h>

static void enter(sac_nav_t *n, sac_state_t s, uint32_t now)
{
    n->state = s;
    n->state_since_ms = now;
    n->node_streak = 0;
    if (s != SAC_RECOVER) n->lost_since_ms = 0;
    sac_controller_reset(&n->controller);
}
static sac_wheels_t wheels(float l, float r) { return (sac_wheels_t){l, r}; }
static bool elapsed(uint32_t now, uint32_t then, uint32_t milliseconds)
{
    return (uint32_t)(now - then) >= milliseconds;
}
void sac_nav_init(sac_nav_t *n, uint32_t now)
{
    memset(n, 0, sizeof(*n));
    n->state = SAC_FOLLOW;
    n->state_since_ms = now;
    n->last_node_ms = now - SAC_NODE_COOLDOWN_MS;
    n->node_armed = true;
}

sac_wheels_t sac_nav_step(sac_nav_t *n, const sac_observation_t *obs,
                          bool object_present, uint32_t now, float dt)
{
    if (obs->polarity_switched) sac_controller_reset(&n->controller);
    if (obs->has_line) n->last_error = obs->position;
    if (obs->has_line && !obs->wide) {
        if (n->narrow_streak < 4) ++n->narrow_streak;
        if (n->narrow_streak >= 4) n->node_armed = true;
    } else n->narrow_streak = 0;

    /* Finish-zone detection is opt-in because the marker shape is not published. */
#if SAC_FINISH_MARKER_ENABLED
    if (n->state == SAC_FOLLOW && n->node_count >= SAC_FINISH_AFTER_N_NODES && obs->wide) {
        if (!n->wide_since_ms) n->wide_since_ms = now ? now : 1;
        if (elapsed(now, n->wide_since_ms, SAC_FINISH_WIDE_HOLD_MS))
            enter(n, SAC_FINISHED, now);
        else return wheels(0, 0); /* Remain stopped while verifying END marker. */
    } else n->wide_since_ms = 0;
#endif
    if (n->state == SAC_FINISHED || n->state == SAC_FAULT) return wheels(0, 0);

    switch (n->state) {
    case SAC_FOLLOW:
#if SAC_OBSTACLE_SENSOR_READY
        if (object_present) {
            if (!n->object_since_ms) n->object_since_ms = now ? now : 1;
            if (elapsed(now, n->object_since_ms, SAC_OBJECT_CONFIRM_MS)) {
                enter(n, SAC_OBJECT_APPROACH, now);
                return wheels(0, 0);
            }
        } else n->object_since_ms = 0;
#else
        (void)object_present;
#endif
        if (n->node_armed && obs->wide &&
            elapsed(now, n->last_node_ms, SAC_NODE_COOLDOWN_MS)) {
            ++n->node_streak;
            if (n->node_streak >= SAC_NODE_CONFIRM_SAMPLES) {
                ++n->node_count;
                n->node_armed = false;
                n->last_node_ms = now;
                const char *route = SAC_NODE_ROUTE;
                n->maneuver = route[n->route_index] ? route[n->route_index++] : 'S';
                enter(n, SAC_NODE_CLEAR, now);
                return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
            }
        } else n->node_streak = 0;

        /* Junction markings are ambiguous, so hold a straight crawl while wide. */
        if (obs->wide) {
            n->lost_since_ms = 0;
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (!obs->has_line) {
            if (!n->lost_since_ms) n->lost_since_ms = now ? now : 1;
            if (elapsed(now, n->lost_since_ms, SAC_LOST_CONFIRM_MS)) {
                enter(n, SAC_RECOVER, now);
                return wheels(0, 0);
            }
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        n->lost_since_ms = 0;
        return sac_controller_update(&n->controller, obs->position, dt);

    case SAC_NODE_CLEAR:
        if (!elapsed(now, n->state_since_ms, SAC_NODE_CLEARANCE_MS))
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        if (n->maneuver == 'L' || n->maneuver == 'R' || n->maneuver == 'U') {
            enter(n, SAC_NODE_TURN, now);
        } else {
            enter(n, SAC_FOLLOW, now);
        }
        return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);

    case SAC_NODE_TURN: {
        bool is_u = (n->maneuver == 'U');
        uint32_t min_ms = is_u ? SAC_UTURN_MIN_MS : SAC_MIN_PIVOT_MS;
        uint32_t max_ms = is_u ? SAC_UTURN_MAX_MS : SAC_MAX_PIVOT_MS;
        if (elapsed(now, n->state_since_ms, min_ms) && obs->has_line &&
            fabsf(obs->position) < 0.65f) {
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, max_ms)) {
            enter(n, SAC_RECOVER, now);
            return wheels(0, 0);
        }
        return n->maneuver == 'R' ? wheels(SAC_TURN_PWM, -SAC_TURN_PWM)
                                   : wheels(-SAC_TURN_PWM, SAC_TURN_PWM);
    }

    case SAC_RECOVER:
        if (obs->has_line && fabsf(obs->position) < 1.70f) {
            n->lost_since_ms = 0;
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, SAC_RECOVERY_TIMEOUT_MS)) {
            enter(n, SAC_FAULT, now);
            return wheels(0, 0);
        }
        return n->last_error < 0 ? wheels(-SAC_SEARCH_PWM, SAC_SEARCH_PWM)
                                 : wheels(SAC_SEARCH_PWM, -SAC_SEARCH_PWM);

    case SAC_OBJECT_APPROACH:
        if (elapsed(now, n->state_since_ms, SAC_OBJECT_APPROACH_MS)) {
            enter(n, SAC_OBJECT_PUSH, now);
        }
        return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);

    case SAC_OBJECT_PUSH:
        if (elapsed(now, n->state_since_ms, SAC_PUSH_DURATION_MS)) {
            n->maneuver = 'U';
            enter(n, SAC_UTURN, now);
            return wheels(0, 0);
        }
        return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);

    case SAC_UTURN:
        if (elapsed(now, n->state_since_ms, SAC_UTURN_MIN_MS) &&
            obs->has_line && fabsf(obs->position) < 0.65f) {
            enter(n, SAC_FOLLOW, now);
            return wheels(SAC_CREEP_PWM, SAC_CREEP_PWM);
        }
        if (elapsed(now, n->state_since_ms, SAC_UTURN_MAX_MS)) {
            enter(n, SAC_RECOVER, now);
            return wheels(0, 0);
        }
        return wheels(-SAC_TURN_PWM, SAC_TURN_PWM);

    default: return wheels(0, 0);
    }
}
const char *sac_nav_state_name(sac_state_t state)
{
    switch (state) {
    case SAC_FOLLOW: return "FOLLOW";
    case SAC_NODE_CLEAR: return "NODE_CLEAR";
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
