/* Servo Control Wrapper Implementation using ESP-IDF Servo Component */

#include "servo_control.h"
#include "esp_log.h"

static const char *TAG = "servo_wrapper";
static bool servo_initialized = false;

esp_err_t servo_init(void) {
    if (servo_initialized) {
        ESP_LOGW(TAG, "Servo already initialized");
        return ESP_OK;
    }

    // Configure servo according to the component documentation
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = SERVO_TIMER,
        .channels = {
            .servo_pin = {
                11,  // Use GPIO 11 directly from app_config.h
            },
            .ch = {
                SERVO_CHANNEL,
            },
        },
        .channel_number = 1,
    };

    // Initialize the servo
    esp_err_t ret = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo: %s", esp_err_to_name(ret));
        return ret;
    }

    servo_initialized = true;
    ESP_LOGI(TAG, "Servo initialized on GPIO %d", 11);
    return ESP_OK;
}

esp_err_t servo_set_angle(float angle) {
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (angle > 180.0f) {
        ESP_LOGW(TAG, "Angle %.1f > 180, clamping to 180", angle);
        angle = 180.0f;
    } else if (angle < 0.0f) {
        ESP_LOGW(TAG, "Angle %.1f < 0, clamping to 0", angle);
        angle = 0.0f;
    }

    esp_err_t ret = iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo angle: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Servo angle set to %.1f degrees", angle);
    }
    
    return ret;
}

esp_err_t servo_deinit(void) {
    if (!servo_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = iot_servo_deinit(LEDC_LOW_SPEED_MODE);
    if (ret == ESP_OK) {
        servo_initialized = false;
        ESP_LOGI(TAG, "Servo deinitialized");
    }
    return ret;
}
