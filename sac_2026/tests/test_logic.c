#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sac_config.h"
#include "sac_perception.h"
#include "sac_controller.h"
#include "sac_navigation.h"

static const sac_calibration_t CAL = {
    .white = {3600,3600,3600,3600,3600},
    .black = {500,500,500,500,500}
};
static void raw_from_mask(int raw[5], unsigned mask, int fg, int bg)
{
    for (int i = 0; i < 5; ++i) raw[i] = (mask & (1u << i)) ? fg : bg;
}
static sac_observation_t observation(float position, bool has_line, bool wide)
{
    return (sac_observation_t){.position = position, .has_line=has_line,
                               .wide=wide, .active_count=wide ? 5 : (has_line ? 1 : 0)};
}
int main(void)
{
    assert(sac_calibration_valid(&CAL, 200));
    sac_calibration_t bad = CAL;
    bad.black[2] = bad.white[2];
    assert(!sac_calibration_valid(&bad, 200));

    sac_perception_t p;
    sac_perception_init(&p, &CAL, true);
    int raw[5];
    raw_from_mask(raw, 1u << 2, 3600, 500);
    sac_observation_t o = sac_perception_process(&p, raw);
    assert(o.has_line && o.active_count == 1 && fabsf(o.position) < 0.01f);
    raw_from_mask(raw, 1u << 0, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.has_line && o.position < -1.9f);
    raw_from_mask(raw, 1u << 4, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.has_line && o.position > 1.9f);

    raw_from_mask(raw, 0x1Fu, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.wide && !o.has_line);
    raw_from_mask(raw, 0x15u, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(!o.has_line);

    /* Inversion: black line on white, while previously following white line. */
    raw_from_mask(raw, 1u << 2, 500, 3600);
    for (int i = 0; i < SAC_SWITCH_CONFIRM_SAMPLES - 1; ++i) {
        o = sac_perception_process(&p, raw);
        assert(!o.wide && !o.polarity_switched);
    }
    o = sac_perception_process(&p, raw);
    assert(o.polarity_switched && !o.white_line && o.has_line);

    /* A brief glitch must not flip the polarity back. */
    raw_from_mask(raw, 1u << 2, 3600, 500);
    (void)sac_perception_process(&p, raw);
    raw_from_mask(raw, 1u << 2, 500, 3600);
    o = sac_perception_process(&p, raw);
    assert(!o.polarity_switched && !o.white_line);

    sac_controller_t c = {0};
    sac_wheels_t cmd = sac_controller_update(&c, 0, .01f);
    assert(fabsf(cmd.left - cmd.right) < .01f);
    sac_controller_reset(&c);
    cmd = sac_controller_update(&c, -1, .01f);
    assert(cmd.left < cmd.right);
    sac_controller_reset(&c);
    cmd = sac_controller_update(&c, 1, .01f);
    assert(cmd.left > cmd.right);

    sac_nav_t nav;
    sac_nav_init(&nav, 1000);
    sac_observation_t center = observation(0, true, false);
    cmd = sac_nav_step(&nav, &center, false, 1010, 0.01f);
    assert(nav.state == SAC_FOLLOW && cmd.left > 0 && cmd.right > 0);
    sac_observation_t junction = observation(0, false, true);
    for (unsigned i = 0; i < SAC_NODE_CONFIRM_SAMPLES; ++i)
        (void)sac_nav_step(&nav, &junction, false, 1020 + i * 10, .01f);
    assert(nav.state == SAC_NODE_CLEAR && nav.node_count == 1);
    (void)sac_nav_step(&nav, &center, false, 1250, .01f);
    assert(nav.state == SAC_FOLLOW);
    /* Remaining on one long, broad patch must not count the same node again. */
    for (unsigned t = 1260; t < 2050; t += 10)
        (void)sac_nav_step(&nav, &junction, false, t, .01f);
    assert(nav.node_count == 1);
    for (unsigned t = 2060; t < 2130; t += 10)
        (void)sac_nav_step(&nav, &center, false, t, .01f);
    assert(nav.node_armed);

    /* Short loss is a crawl; sustained loss enters recovery; unrecovered stops. */
    sac_observation_t missing = observation(0, false, false);
    (void)sac_nav_step(&nav, &missing, false, 2200, .01f);
    assert(nav.state == SAC_FOLLOW);
    (void)sac_nav_step(&nav, &missing, false, 2310, .01f);
    assert(nav.state == SAC_RECOVER);
    (void)sac_nav_step(&nav, &missing, false, 4200, .01f);
    assert(nav.state == SAC_FAULT);
    cmd = sac_nav_step(&nav, &center, false, 4210, .01f);
    assert(cmd.left == 0 && cmd.right == 0);
    /* Test both pivot directions and line-based reacquisition. */
    sac_nav_t pivot;
    sac_nav_init(&pivot, 5000);
    pivot.state = SAC_NODE_TURN;
    pivot.state_since_ms = 5000;
    pivot.maneuver = 'L';
    cmd = sac_nav_step(&pivot, &missing, false, 5100, .01f);
    assert(cmd.left < 0 && cmd.right > 0);
    (void)sac_nav_step(&pivot, &center, false, 5400, .01f);
    assert(pivot.state == SAC_FOLLOW);
    pivot.state = SAC_NODE_TURN;
    pivot.state_since_ms = 5500;
    pivot.maneuver = 'R';
    cmd = sac_nav_step(&pivot, &missing, false, 5600, .01f);
    assert(cmd.left > 0 && cmd.right < 0);
    (void)sac_nav_step(&pivot, &missing, false, 6650, .01f);
    assert(pivot.state == SAC_RECOVER);
    puts("PASS: calibration, centroids, inversions, PD, node debounce, pivots, recovery, fault stop");
    return 0;
}
