#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sra_board.h"

#include "sac_config.h"
#include "sac_controller.h"
#include "sac_drive.h"
#include "sac_navigation.h"
#include "sac_obstacle.h"
#include "sac_perception.h"

static const char *TAG = "SAC_2026";
static uint32_t milliseconds(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void sample_surface(adc_handle_t sensor, float out[SAC_SENSORS], const char *label)
{
    float totals[SAC_SENSORS] = {0};
    ESP_LOGW(TAG, "Place ALL five sensors over %s. Capturing in 3 seconds.", label);
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int sample = 0; sample < SAC_CALIBRATION_SAMPLES; ++sample) {
        line_sensor_array r = read_line_sensor(sensor);
        for (int i = 0; i < SAC_SENSORS; ++i) totals[i] += r.adc_reading[i];
        vTaskDelay(pdMS_TO_TICKS(SAC_CALIBRATION_INTERVAL_MS));
    }
    for (int i = 0; i < SAC_SENSORS; ++i) out[i] = totals[i] / SAC_CALIBRATION_SAMPLES;
    ESP_LOGI(TAG, "%s ADC: %.0f %.0f %.0f %.0f %.0f", label,
             out[0], out[1], out[2], out[3], out[4]);
}

void app_main(void)
{
    ESP_LOGW(TAG, "SAC 2026 bench firmware. Motor armed=%d", SAC_MOTORS_ARMED);
    ESP_LOGW(TAG, "No forward object sensor configured; obstacle task INCOMPLETE.");
    adc_handle_t sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&sensor));
    sac_drive_t drive;
    ESP_ERROR_CHECK(sac_drive_init(&drive));
    ESP_ERROR_CHECK(sac_drive_stop(&drive));

    sac_calibration_t cal;
    memset(&cal, 0, sizeof(cal));
    sample_surface(sensor, cal.white, "WHITE surface");
    sample_surface(sensor, cal.black, "BLACK surface");
    if (!sac_calibration_valid(&cal, SAC_CALIBRATION_CONTRAST_MIN)) {
        ESP_LOGE(TAG, "Calibration contrast too small. Motors will remain stopped.");
        ESP_ERROR_CHECK(sac_drive_stop(&drive));
        return;
    }

    sac_perception_t perception;
    sac_perception_init(&perception, &cal, SAC_INITIAL_LINE_IS_WHITE);
    sac_nav_t navigation;
    sac_nav_init(&navigation, milliseconds());
    ESP_LOGI(TAG, "Calibration passed. Place the bot at START. Starting in 3s.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    TickType_t tick = xTaskGetTickCount();
    uint32_t last_ms = milliseconds();
    uint32_t log_ms = last_ms;
    sac_state_t previous_state = navigation.state;
    for (;;) {
        uint32_t now = milliseconds();
        float dt = (float)((uint32_t)(now - last_ms)) / 1000.0f;
        last_ms = now;
        line_sensor_array r = read_line_sensor(sensor);
        sac_observation_t observation = sac_perception_process(&perception, r.adc_reading);
        sac_wheels_t command = sac_nav_step(&navigation, &observation,
                                             sac_obstacle_present(), now, dt);
        if (navigation.state != previous_state) {
            ESP_LOGI(TAG, "state %s -> %s; nodes=%d", sac_nav_state_name(previous_state),
                     sac_nav_state_name(navigation.state), navigation.node_count);
            previous_state = navigation.state;
        }
        if (observation.polarity_switched)
            ESP_LOGW(TAG, "LINE POLARITY CHANGED: %s", observation.white_line ? "white" : "black");
        esp_err_t err = sac_drive_apply(&drive, command);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor API error: %s", esp_err_to_name(err));
            sac_drive_stop(&drive);
            return;
        }
        if ((uint32_t)(now - log_ms) >= SAC_LOG_INTERVAL_MS) {
            ESP_LOGI(TAG, "state=%s pos=%.2f active=%d conf=%.2f cmd=(%.1f,%.1f) adc=%d,%d,%d,%d,%d",
                     sac_nav_state_name(navigation.state), observation.position,
                     observation.active_count, observation.confidence,
                     command.left, command.right,
                     r.adc_reading[0], r.adc_reading[1], r.adc_reading[2],
                     r.adc_reading[3], r.adc_reading[4]);
            log_ms = now;
        }
        if (navigation.state == SAC_FAULT || navigation.state == SAC_FINISHED) {
            ESP_ERROR_CHECK(sac_drive_stop(&drive));
            ESP_LOGW(TAG, "Final state %s: stopped", sac_nav_state_name(navigation.state));
            return;
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(SAC_CONTROL_PERIOD_MS));
    }
}
