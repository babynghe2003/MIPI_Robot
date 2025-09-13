/* Servo Task Implementation (2 servos)
 * Smoothly animates servos towards targets using easeInOutSine; targets set via X/Y cmds.
 */

#include "servo_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// External function to update robot target angles from robot_task.cpp
extern void robot_update_target_angles_from_servo(float left_offset, float right_offset);

static const char *TAG = "servo_task";
static const float TARGET_EPS_DEG = 0.1f; // epsilon to detect meaningful target changes

// Easing helper: easeInOutSine, t in [0,1]
static inline float ease_in_out_sine(float t) {
    // 0.5 * (1 - cos(pi * t))
    const float PI = 3.14159265358979f;
    if (t <= 0.f) return 0.f;
    if (t >= 1.f) return 1.f;
    return 0.5f * (1.f - cosf(PI * t));
}

/* Servo Task - Sweeps two servos from 0 to 180 degrees */
void servo_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Servo task started");
    
    /* Initialize servo */
    esp_err_t ret = servo_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    // Seed current to initial positions, and set targets to the same
    servo_set_angle_idx(0,103);
    servo_set_angle_idx(1,85);
    servo_set_target_idx(0, 103);
    servo_set_target_idx(1, 85);

    // Animation settings
    const TickType_t tick = pdMS_TO_TICKS(5); // 50 Hz update
    const float duration_ms = 500.0f;          // duration for full tween when target changes
    float t0_ms[2] = {0, 0};                   // tween start time in ms (relative)
    float start_angle[2] = {103, 85};          // angle at tween start
    float last_target[2] = {103, 85};

    TickType_t start_tick = xTaskGetTickCount();
    while (1) {
        // Time since task start in ms
        TickType_t now_tick = xTaskGetTickCount();
        float now_ms = (float)((now_tick - start_tick) * portTICK_PERIOD_MS);

        // For each servo, if target changed, start a new tween
        for (int i = 0; i < 2; ++i) {
            float target = servo_get_target_idx(i);
            float current = servo_get_current_idx(i);
            if (fabsf(target - last_target[i]) > TARGET_EPS_DEG) {
                last_target[i] = target;
                start_angle[i] = current;
                t0_ms[i] = now_ms;
            }

            // Compute progress
            float dt = now_ms - t0_ms[i];
            if (dt < 0.f) dt = 0.f;
            float progress = (dt <= 0.f) ? 0.f : fminf(dt / duration_ms, 1.f);
            if (progress < 1.f) {
                float eased = ease_in_out_sine(progress);
                float new_angle = start_angle[i] + (target - start_angle[i]) * eased;
                servo_set_angle_idx(i, new_angle);

            } else {
                // Only write final target if we are not already effectively there
                if (fabsf(current - target) > TARGET_EPS_DEG) {
                    servo_set_angle_idx(i, target);
                }
            }
        }

        // Update robot target angles based on current servo positions
        float servo0_angle = servo_get_current_idx(0);
        float servo1_angle = servo_get_current_idx(1);
        
        // Calculate offsets according to user specification:
        // target_angle_right offset = servo_0 - 103
        // target_angle_left offset = 85 - servo_1
        float right_offset = servo0_angle - 103.0f;
        float left_offset = 85.0f - servo1_angle;
        
        robot_update_target_angles_from_servo(left_offset, -right_offset);

        vTaskDelay(tick);
    }
}
