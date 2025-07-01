/* BLDC Task Implementation */

#include "bldc_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.h"

static const char *TAG = "bldc_task";

void bldc_task(void *pvParameters) {
    ESP_LOGI(TAG, "BLDC task started");

    // Initialize BLDC system
    if (bldc_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLDC system - task terminating");
        vTaskDelete(NULL);
        return;
    }

    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Sweep parameters: -5 to 5 rad/s in 2 seconds
    const float sweep_duration = 2.0f;    // 2 seconds
    const float min_velocity = -5.0f;     // -5 rad/s
    const float max_velocity = 5.0f;      // 5 rad/s
    const float sweep_range = max_velocity - min_velocity;
    const uint32_t loop_period_ms = 10;   // 10ms loop period (100Hz)
    const uint32_t total_steps = (uint32_t)(sweep_duration * 1000 / loop_period_ms);

    ESP_LOGI(TAG, "Starting velocity sweep: %.1f to %.1f rad/s over %.1f seconds", 
             min_velocity, max_velocity, sweep_duration);

    uint32_t step = 0;
    int64_t start_time = esp_timer_get_time();
    bldc_set_target(20);
    bldc_loop();  // Initial loop to set up motor state
    while (1) {
        // Calculate sweep progress
        // float progress = (float)step / (float)total_steps;
        // if (progress > 1.0f) {
        //     progress = 1.0f;
        //     step = 0;  // Reset for next cycle
        //     start_time = esp_timer_get_time();
        //     ESP_LOGI(TAG, "Sweep cycle completed, restarting...");
        // }

        // // Calculate target velocity
        // float target_velocity = min_velocity + progress * sweep_range;

        // // Set target and run control loop
        // bldc_set_target(target_velocity);
        // bldc_loop();
        bldc_set_target(10);
        bldc_loop();  // Initial loop to set up motor state
        // use bldc_get_encoder_angle
        // ESP_LOGI(TAG, "Encoder angle: %.3f rad (%.1f°)", 
        //          bldc_get_encoder_angle(), 
        //          bldc_get_encoder_angle() * 180.0f / 3.14159f);
        // // Log progress and encoder reading every 200ms
        // if (step % 20 == 0) {
        //     int64_t current_time = esp_timer_get_time();
        //     float elapsed_time = (current_time - start_time) / 1000000.0f;
        //     float encoder_angle = bldc_get_encoder_angle();
        //     ESP_LOGI(TAG, "Sweep: %.1fs, Target: %.2f rad/s, Encoder: %.3f rad (%.1f°), Progress: %.1f%%", 
        //              elapsed_time, target_velocity, encoder_angle, 
        //              encoder_angle * 180.0f / 3.14159f, progress * 100.0f);
        // }

        // step++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
