/* Servo Task Implementation */

#include "servo_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "servo_task";

/* Servo Task - Sweeps servo from 0 to 180 degrees */
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
    
    float angle = 0.0f;
    float step = 5.0f; // Move in 5-degree steps
    int direction = 1; // 1 for increasing, -1 for decreasing
    
    while (1) {
        /* Set servo angle */
        ret = servo_set_angle(angle);
        
        /* Update angle for next iteration */
        angle += direction * step;
        
        /* Reverse direction at limits */
        if (angle >= 180.0f) {
            angle = 180.0f;
            direction = -1;
        } else if (angle <= 0.0f) {
            angle = 0.0f;
            direction = 1;
        }
        
        /* Wait before next movement */
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS_NORMAL));
    }
}
