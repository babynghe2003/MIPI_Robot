/* LED Task Implementation */

#include "led_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led_task";

/* LED Task - Blinks the onboard LED */
void led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED task started");
    
    /* Initialize LED */
    esp_err_t ret = led_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    bool led_state = false;
    
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", led_state ? "ON" : "OFF");
        
        ret = led_set_state(led_state);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED state: %s", esp_err_to_name(ret));
        }
        
        /* Toggle the LED state */
        led_state = !led_state;
        
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BLINK_PERIOD));
    }
}
