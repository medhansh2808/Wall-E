#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sac_config.h"
#include "sac_perception.h"
#include "sac_controller.h"
#include "sac_junction.h"
#include "sac_navigation.h"

static const sac_calibration_t CAL = {
    .white = {3600,3600,3600,3600,3600},
    .black = {500,500,500,500,500}
};
static void sensor_raw(int raw[5], unsigned mask, int fg, int bg)
{
    for (int i = 0; i < 5; ++i) raw[i] = (mask & (1u << i)) ? fg : bg;
}
static sac_observation_t obs(unsigned mask)
{
    int count = 0;
    for (int i = 0; i < 5; i++) if (mask & (1u << i)) count++;
    sac_observation_t o = {0};
    o.active_mask = mask;
    o.active_count = count;
    o.wide = count >= 4;
    o.has_line = count > 0 && count < 4;
    if (mask == 0x01u) o.position = -2;
    if (mask == 0x02u) o.position = -1;
    if (mask == 0x08u) o.position = 1;
    if (mask == 0x10u) o.position = 2;
    return o;
}
static void test_perception(void)
{
    assert(sac_calibration_valid(&CAL, 200));
    sac_calibration_t bad = CAL;
    bad.black[2] = bad.white[2];
    assert(!sac_calibration_valid(&bad, 200));
    sac_perception_t p;
    sac_perception_init(&p, &CAL, true);
    int raw[5];
    sensor_raw(raw, 0x04, 3600, 500);
    sac_observation_t o = sac_perception_process(&p, raw);
    assert(o.has_line && o.active_mask == 0x04 && fabsf(o.position) < .01f);
    sensor_raw(raw, 0x01, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.has_line && o.position < -1.9f);
    sensor_raw(raw, 0x10, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.has_line && o.position > 1.9f);
    sensor_raw(raw, 0x1f, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(o.wide && !o.has_line);
    sensor_raw(raw, 0x15, 3600, 500);
    o = sac_perception_process(&p, raw);
    assert(!o.has_line);
    sensor_raw(raw, 0x04, 500, 3600);
    for (int i = 0; i < SAC_SWITCH_CONFIRM_SAMPLES - 1; ++i) {
        o = sac_perception_process(&p, raw);
        assert(!o.polarity_switched);
    }
    o = sac_perception_process(&p, raw);
    assert(o.polarity_switched && !o.white_line && o.has_line);
}
static void test_controller(void)
{
    sac_controller_t c = {0};
    sac_wheels_t cmd = sac_controller_update(&c, 0, .01f);
    assert(fabsf(cmd.left - cmd.right) < .01f);
    sac_controller_reset(&c);
    cmd = sac_controller_update(&c, -1, .01f);
    assert(cmd.left < cmd.right);
    sac_controller_reset(&c);
    cmd = sac_controller_update(&c, 1, .01f);
    assert(cmd.left > cmd.right);
}
static void test_junction_classification(void)
{
    sac_junction_t j;
    sac_junction_reset(&j);
    sac_observation_t center = obs(0x04);
    sac_observation_t left = obs(0x07);
    sac_observation_t right = obs(0x1c);
    sac_observation_t cross = obs(0x1f);
    assert(!sac_junction_observe(&j, &left));
    assert(!sac_junction_observe(&j, &center));
    for (int i=0; i<SAC_NODE_CONFIRM_SAMPLES; i++)
        (void)sac_junction_observe(&j, &left);
    assert(j.left && !j.right);
    assert(sac_junction_choose(&j, 0) == 'L');
    sac_junction_reset(&j);
    for (int i=0; i<SAC_NODE_CONFIRM_SAMPLES; i++)
        (void)sac_junction_observe(&j, &right);
    assert(j.right && !j.left);
    assert(sac_junction_choose(&j, 1) == 'R');
    sac_junction_reset(&j);
    for (int i=0; i<SAC_NODE_CONFIRM_SAMPLES; i++)
        (void)sac_junction_observe(&j, &cross);
    assert(j.right && j.left);
    assert(sac_junction_choose(&j, 0) == 'L');
    assert(sac_junction_choose(&j, 1) == 'R');
    assert(sac_junction_choose(&j, 2) == 'L');
    sac_junction_scan(&j, &center);
    sac_junction_scan(&j, &center);
    assert(sac_junction_choose(&j, 2) == 'S');
}
static void test_navigation(void)
{
    sac_nav_t nav;
    sac_nav_init(&nav, 1000);
    sac_events_t none = {0};
    sac_observation_t center = obs(0x04);
    sac_observation_t left = obs(0x07);
    sac_observation_t missing = obs(0x00);
    sac_wheels_t cmd = sac_nav_step(&nav, &center, none, 1010, .01f);
    assert(nav.state == SAC_FOLLOW && cmd.left > 0 && cmd.right > 0);
    for (int i=0; i<SAC_NODE_CONFIRM_SAMPLES; i++)
        (void)sac_nav_step(&nav, &left, none, 1020 + i * 10, .01f);
    assert(nav.state == SAC_NODE_SCAN && nav.node_count == 1);
    (void)sac_nav_step(&nav, &left, none, 1230, .01f);
    assert(nav.state == SAC_NODE_TURN && nav.maneuver == 'L');
    cmd = sac_nav_step(&nav, &missing, none, 1240, .01f);
    assert(cmd.left < 0 && cmd.right > 0);
    for (int i=0; i<SAC_TURN_REACQUIRE_SAMPLES; i++)
        (void)sac_nav_step(&nav, &center, none, 1490 + i * 10, .01f);
    assert(nav.state == SAC_FOLLOW);
    /* Can't count a junction twice immediately. */
    for (unsigned t=1530; t<1640; t+=10)
        (void)sac_nav_step(&nav, &left, none, t, .01f);
    assert(nav.node_count == 1);
    (void)sac_nav_step(&nav, &missing, none, 1800, .01f);
    (void)sac_nav_step(&nav, &missing, none, 1920, .01f);
    assert(nav.state == SAC_RECOVER);
    (void)sac_nav_step(&nav, &missing, none, 4000, .01f);
    assert(nav.state == SAC_FAULT);
    cmd = sac_nav_step(&nav, &center, none, 4010, .01f);
    assert(cmd.left == 0 && cmd.right == 0);
}
static void test_straight_junction(void)
{
    sac_nav_t nav;
    sac_nav_init(&nav, 10);
    sac_events_t ev = {0};
    sac_observation_t cross = obs(0x1f), center = obs(0x04);
    for (int i=0; i<SAC_NODE_CONFIRM_SAMPLES; i++)
        (void)sac_nav_step(&nav, &cross, ev, 20+i*10, .01f);
    assert(nav.state == SAC_NODE_SCAN);
    (void)sac_nav_step(&nav, &center, ev, 110, .01f);
    (void)sac_nav_step(&nav, &center, ev, 120, .01f);
    (void)sac_nav_step(&nav, &center, ev, 200, .01f);
    assert(nav.state == SAC_FOLLOW);
}
static void test_push_and_finish(void)
{
    sac_observation_t center = obs(0x04);
    sac_nav_t nav;
    sac_nav_init(&nav, 100);
    sac_events_t ev = {.object_present=true};
    (void)sac_nav_step(&nav, &center, ev, 110, .01f);
    (void)sac_nav_step(&nav, &center, ev, 230, .01f);
    assert(nav.state == SAC_OBJECT_APPROACH);
    (void)sac_nav_step(&nav, &center, ev, 500, .01f);
    assert(nav.state == SAC_OBJECT_PUSH);
    (void)sac_nav_step(&nav, &center, ev, 2000, .01f);
    assert(nav.state == SAC_FAULT && nav.pushed_count == 0);

    sac_nav_init(&nav, 100);
    (void)sac_nav_step(&nav, &center, ev, 110, .01f);
    (void)sac_nav_step(&nav, &center, ev, 230, .01f);
    (void)sac_nav_step(&nav, &center, ev, 500, .01f);
    ev.dropzone_reached = true;
    (void)sac_nav_step(&nav, &center, ev, 510, .01f);
    (void)sac_nav_step(&nav, &center, ev, 570, .01f);
    assert(nav.state == SAC_UTURN && nav.pushed_count == 1);
    sac_observation_t missing = obs(0);
    (void)sac_nav_step(&nav, &missing, ev, 580, .01f);
    for (int i=0; i<SAC_TURN_REACQUIRE_SAMPLES; i++)
        (void)sac_nav_step(&nav, &center, ev, 1250+i*10, .01f);
    assert(nav.state == SAC_FOLLOW);
    (void)sac_nav_step(&nav, &center, ev, 1320, .01f);
    assert(nav.state == SAC_FOLLOW); /* same object not retriggered */
    ev.object_present = false;
    ev.dropzone_reached = false;
    ev.finish_present = true;
    (void)sac_nav_step(&nav, &center, ev, 1330, .01f);
    (void)sac_nav_step(&nav, &center, ev, 1560, .01f);
    assert(nav.state == SAC_FINISHED);
}
int main(void)
{
    assert(SAC_MOTORS_ARMED == 0);
    assert(SAC_ALLOW_UNVERIFIED_TIMED_PUSH == 0);
    test_perception();
    test_controller();
    test_junction_classification();
    test_navigation();
    test_straight_junction();
    test_push_and_finish();
    puts("PASS: perception, PD, junction policy, scan/turn, recovery, verified push, finish");
    return 0;
}
