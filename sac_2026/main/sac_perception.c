#include "sac_perception.h"
#include "sac_config.h"
#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

bool sac_calibration_valid(const sac_calibration_t *cal, float min_adc_contrast)
{
    if (!cal) return false;
    for (int i = 0; i < SAC_SENSORS; ++i) {
        if (!isfinite(cal->white[i]) || !isfinite(cal->black[i]) ||
            fabsf(cal->white[i] - cal->black[i]) < min_adc_contrast) return false;
    }
    return true;
}

void sac_perception_init(sac_perception_t *p, const sac_calibration_t *cal, bool initial_white)
{
    memset(p, 0, sizeof(*p));
    p->calibration = *cal;
    p->white_line = initial_white;
}

static sac_observation_t estimate(const sac_perception_t *p, const int raw[5], bool white_line)
{
    sac_observation_t o = {0};
    float mass = 0.0f;
    float weighted = 0.0f;
    int mask = 0;
    for (int i = 0; i < 5; ++i) {
        float denom = p->calibration.white[i] - p->calibration.black[i];
        float white = clampf(((float)raw[i] - p->calibration.black[i]) / denom, 0, 1);
        float activation = white_line ? white : 1.0f - white;
        o.activation[i] = activation;
        if (activation >= SAC_SENSOR_THRESHOLD) {
            ++o.active_count;
            mask |= 1 << i;
        }
        mass += activation;
        weighted += (float)(i - 2) * activation;
    }
    o.white_line = white_line;
    o.wide = o.active_count >= 4;
    /* A wide marking is useful for nodes, but not a reliable line position. */
    o.has_line = o.active_count >= 1 && o.active_count <= 3;
    if (mass > 0.001f) o.position = clampf(weighted / mass, -2.0f, 2.0f);
    /* Disconnected narrow patterns can be junction fragments; don't steer on them. */
    if (o.has_line) {
        /* Validate contiguous bits without assuming every pattern is a line. */
        int first = 0, last = 4;
        while (first < 5 && !(mask & (1 << first))) ++first;
        while (last >= 0 && !(mask & (1 << last))) --last;
        for (int i = first; i <= last; ++i) {
            if (!(mask & (1 << i))) { o.has_line = false; break; }
        }
    }
    o.confidence = o.has_line ? clampf(mass / (float)o.active_count, 0, 1) : 0;
    return o;
}

sac_observation_t sac_perception_process(sac_perception_t *p, const int raw[5])
{
    sac_observation_t current = estimate(p, raw, p->white_line);
    sac_observation_t alternative = estimate(p, raw, !p->white_line);
    /* Require a convincingly narrow alternative, and no narrow current line. */
    bool candidate = !current.has_line && alternative.has_line &&
                     alternative.confidence >= 0.55f && alternative.active_count <= 2;
    if (candidate) {
        /* Do not confuse an inverted segment with a broad junction marker. */
        current.wide = false;
        current.has_line = false;
        ++p->pending_switch;
    } else {
        p->pending_switch = 0;
    }
    if (p->pending_switch >= SAC_SWITCH_CONFIRM_SAMPLES) {
        p->white_line = !p->white_line;
        p->pending_switch = 0;
        alternative.polarity_switched = true;
        return alternative;
    }
    return current;
}
