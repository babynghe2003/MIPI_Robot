/* Servo Control Wrapper Implementation using ESP-IDF Servo Component (2 servos) */

#include "servo_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "servo_wrapper";
static bool servo_initialized = false;
static float s_target_angle[2] = {0};
static float s_current_angle[2] = {0};
static portMUX_TYPE s_servo_mux = portMUX_INITIALIZER_UNLOCKED;

esp_err_t servo_init(void) {
    if (servo_initialized) {
        ESP_LOGW(TAG, "Servo already initialized");
        return ESP_OK;
    }

    // Configure two servos according to the component documentation
    // Pins: GPIO11 and GPIO4
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = SERVO_TIMER,
        .channels = {
            .servo_pin = {
                11,  // Servo 0 on GPIO 11
                4,   // Servo 1 on GPIO 4
            },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
            },
        },
        .channel_number = 2,
    };

    // Initialize the servo
    esp_err_t ret = iot_servo_init(SERVO_LED_SPEED_MODE, &servo_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo: %s", esp_err_to_name(ret));
        return ret;
    }

    servo_initialized = true;
    ESP_LOGI(TAG, "Servos initialized on GPIO %d and %d", 11, 4);
    return ESP_OK;
}

esp_err_t servo_set_angle_idx(int idx, float angle) {
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (idx < 0 || idx > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (angle > 180.0f) {
        ESP_LOGW(TAG, "Angle %.1f > 180, clamping to 180", angle);
        angle = 180.0f;
    } else if (angle < 0.0f) {
        ESP_LOGW(TAG, "Angle %.1f < 0, clamping to 0", angle);
        angle = 0.0f;
    }

    esp_err_t ret = iot_servo_write_angle(SERVO_LED_SPEED_MODE, idx, angle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo angle: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Servo %d angle set to %.1f degrees", idx, angle);
    }
    if (ret == ESP_OK) {
        portENTER_CRITICAL(&s_servo_mux);
        s_current_angle[idx] = angle;
        portEXIT_CRITICAL(&s_servo_mux);
    }
    
    return ret;
}

static inline float clamp_deg(float a) {
    if (a < 0.0f) return 0.0f;
    if (a > 180.0f) return 180.0f;
    return a;
}

esp_err_t servo_set_target_idx(int idx, float angle) {
    if (!servo_initialized) return ESP_ERR_INVALID_STATE;
    if (idx < 0 || idx > 1) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_servo_mux);
    s_target_angle[idx] = clamp_deg(angle);
    portEXIT_CRITICAL(&s_servo_mux);
    return ESP_OK;
}

float servo_get_current_idx(int idx) {
    if (idx < 0 || idx > 1) return 0.0f;
    portENTER_CRITICAL(&s_servo_mux);
    float v = s_current_angle[idx];
    portEXIT_CRITICAL(&s_servo_mux);
    return v;
}

float servo_get_target_idx(int idx) {
    if (idx < 0 || idx > 1) return 0.0f;
    portENTER_CRITICAL(&s_servo_mux);
    float v = s_target_angle[idx];
    portEXIT_CRITICAL(&s_servo_mux);
    return v;
}

esp_err_t servo_deinit(void) {
    if (!servo_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = iot_servo_deinit(SERVO_LED_SPEED_MODE);
    if (ret == ESP_OK) {
        servo_initialized = false;
        ESP_LOGI(TAG, "Servo deinitialized");
    }
    return ret;
}
