#include "sac_obstacle.h"
#include "sac_config.h"
#include "driver/gpio.h"
#include <stdint.h>

static esp_err_t setup_pin(int pin)
{
    if (pin < 0) return ESP_OK;
    if (!GPIO_IS_VALID_GPIO(pin)) return ESP_ERR_INVALID_ARG;
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&cfg);
}

esp_err_t sac_obstacle_init(void)
{
    esp_err_t e = setup_pin(SAC_OBJECT_GPIO);
    if (e != ESP_OK) return e;
    e = setup_pin(SAC_DROPZONE_GPIO);
    if (e != ESP_OK) return e;
    return setup_pin(SAC_FINISH_GPIO);
}

static bool detected(int pin, int level)
{
    return pin >= 0 && gpio_get_level((gpio_num_t)pin) == level;
}

sac_task_inputs_t sac_obstacle_read(void)
{
    return (sac_task_inputs_t){
        .object_present = detected(SAC_OBJECT_GPIO, SAC_OBJECT_ACTIVE_LEVEL),
        .dropzone_reached = detected(SAC_DROPZONE_GPIO, SAC_DROPZONE_ACTIVE_LEVEL),
        .finish_present = detected(SAC_FINISH_GPIO, SAC_FINISH_ACTIVE_LEVEL)
    };
}
