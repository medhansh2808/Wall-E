#ifndef SAC_PERCEPTION_H
#define SAC_PERCEPTION_H
#include <stdbool.h>
#include <stdint.h>

#define SAC_SENSORS 5

typedef struct {
    float white[SAC_SENSORS];
    float black[SAC_SENSORS];
} sac_calibration_t;

typedef struct {
    bool has_line;
    bool wide;
    bool polarity_switched;
    bool white_line;
    int active_count;
    float position;      /* [-2,2] left-to-right; negative is left. */
    float confidence;    /* [0,1], conservative measure of line evidence. */
    float activation[SAC_SENSORS];
} sac_observation_t;

typedef struct {
    sac_calibration_t calibration;
    bool white_line;
    unsigned pending_switch;
} sac_perception_t;

bool sac_calibration_valid(const sac_calibration_t *cal, float min_adc_contrast);
void sac_perception_init(sac_perception_t *p, const sac_calibration_t *cal, bool initial_white);
sac_observation_t sac_perception_process(sac_perception_t *p, const int raw[SAC_SENSORS]);

#endif
