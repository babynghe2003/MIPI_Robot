/* MPU9250 Display Task Implementation */

#include "mpu9250_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.h"

static const char *TAG = "mpu9250_display";

void mpu9250_display_task(void *pvParameters) {
    ESP_LOGI(TAG, "MPU9250 display task started (100ms interval)");
    
    // Error tracking
    uint32_t error_count = 0;
    uint32_t loop_count = 0;
    
    while (1) {
        // Read sensor data
        esp_err_t ret = mpu9250_read_sensor();
        if (ret == ESP_OK) {
            loop_count++;
            
            // Log sensor data every 10 cycles (1 second at 100ms interval)
            if (loop_count % 10 == 0) {
                // Get angles
                float angle_x, angle_y, angle_z;
                mpu9250_get_angles(&angle_x, &angle_y, &angle_z);
                
                // Get accelerometer data
                float accel_x, accel_y, accel_z;
                mpu9250_get_accel(&accel_x, &accel_y, &accel_z);
                
                // Get gyroscope data
                float gyro_x, gyro_y, gyro_z;
                mpu9250_get_gyro(&gyro_x, &gyro_y, &gyro_z);
                
                // Get temperature
                float temperature = mpu9250_get_temp();
                
                // Calculate error rate
                float error_rate = (float)error_count / (float)(loop_count + error_count) * 100.0f;
                
                ESP_LOGI(TAG, "Loop %lu: Angles(%.1f,%.1f,%.1f)° Accel(%.2f,%.2f,%.2f) Gyro(%.2f,%.2f,%.2f) Temp:%.1f°C Err:%.2f%%", 
                         loop_count, angle_x, angle_y, angle_z,
                         accel_x, accel_y, accel_z,
                         gyro_x, gyro_y, gyro_z,
                         temperature, error_rate);
            }
        } else {
            error_count++;
            if (error_count % 10 == 0) {
                ESP_LOGE(TAG, "MPU9250 read errors: %lu/%lu (%.2f%%)", 
                         error_count, loop_count + error_count, 
                         (float)error_count / (float)(loop_count + error_count) * 100.0f);
            }
        }
        
        // 100ms delay as specified in the logs
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
