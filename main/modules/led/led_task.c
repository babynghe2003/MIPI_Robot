/* LED Task Implementation */

#include "led_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward declarations để tránh circular import
typedef enum {
    ROBOT_STATE_INIT,              
    ROBOT_STATE_I2C_SETUP,         
    ROBOT_STATE_SCAN_DEVICES,      
    ROBOT_STATE_INIT_MPU9250,      
    ROBOT_STATE_CALIBRATING,       
    ROBOT_STATE_INIT_MOTORS,       
    ROBOT_STATE_RUNNING,           
    ROBOT_STATE_ERROR,             
    ROBOT_STATE_SENSOR_ERROR,      
    ROBOT_STATE_MOTOR_ERROR        
} robot_state_t;

typedef struct {
    uint8_t red;
    uint8_t green; 
    uint8_t blue;
    uint32_t period_ms;
    uint8_t duty_cycle;
    bool blink;
} led_rgb_pattern_t;

// External functions từ task_manager (để tránh circular import)
extern robot_state_t task_manager_get_robot_state(void);
extern led_rgb_pattern_t task_manager_get_led_rgb_pattern(void);

static const char *TAG = "led_task";

/* LED Task - Reflects robot task status */
void led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED task started - will reflect robot status");
    
    /* Initialize LED */
    esp_err_t ret = led_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    bool led_state = false;
    robot_state_t last_state = ROBOT_STATE_INIT;
    TickType_t last_toggle_time = xTaskGetTickCount();
    
    while (1) {
        // Lấy pattern hiện tại từ robot state
        led_rgb_pattern_t pattern = task_manager_get_led_rgb_pattern();
        robot_state_t current_state = task_manager_get_robot_state();
        
        // Log khi state thay đổi
        if (current_state != last_state) {
            ESP_LOGI(TAG, "LED pattern changed for robot state %d: RGB(%d,%d,%d), period=%ums, blink=%s", 
                     current_state, pattern.red, pattern.green, pattern.blue, 
                     (unsigned int)pattern.period_ms, pattern.blink ? "YES" : "NO");
            last_state = current_state;
        }
        
        TickType_t current_time = xTaskGetTickCount();
        led_set_color(pattern.red, pattern.green, pattern.blue);
        if (pattern.blink) {
            // Chế độ chớp tắt
            if ((current_time - last_toggle_time) >= pdMS_TO_TICKS(pattern.period_ms)) {
                led_state = !led_state;
                last_toggle_time = current_time;
                
                ret = led_set_state(led_state);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set LED state: %s", esp_err_to_name(ret));
                }
                
                ESP_LOGD(TAG, "LED %s (state=%d, RGB(%d,%d,%d))", 
                         led_state ? "ON" : "OFF", current_state, 
                         pattern.red, pattern.green, pattern.blue);
            }
            
            // Ngủ ngắn để không waste CPU
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            // Chế độ sáng liên tục
            if (!led_state) {
                led_state = true;
                ret = led_set_state(led_state);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set LED state: %s", esp_err_to_name(ret));
                }
                ESP_LOGD(TAG, "LED ON steady (state=%d, RGB(%d,%d,%d))", 
                         current_state, pattern.red, pattern.green, pattern.blue);
            }
            
            // Ngủ lâu hơn cho steady mode
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
