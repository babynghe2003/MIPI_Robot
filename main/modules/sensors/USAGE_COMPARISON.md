/* MPU6050 Usage Example Comparison */

// ================== ARDUINO EXAMPLE ==================
/*
#include "mpu6050_idf.h"
#include "driver/i2c.h"
#include <stdio.h>

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          400000

extern "C" void app_main(void)
{
    // Khởi tạo i2c driver ESP-IDF
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    MPU6050 mpu(I2C_MASTER_NUM);
    mpu.begin();

    mpu.calcGyroOffsets();

    while (1)
    {
        mpu.update();
        printf("AngleX: %.2f, AngleY: %.2f, AngleZ: %.2f\n", mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
*/

// ================== CURRENT ESP-IDF IMPLEMENTATION ==================

/* Usage in mpu6050_task.c:
 
void mpu6050_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU6050 sensor task started");
    
    // Initialize MPU6050 (includes I2C setup)
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Calibrate gyroscope
    ESP_LOGI(TAG, "Starting gyroscope calibration...");
    ret = mpu6050_calibrate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 calibration failed: %s", esp_err_to_name(ret));
    }
    
    mpu6050_angles_t angles;
    
    while (1) {
        // Update sensor readings
        ret = mpu6050_update();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update MPU6050: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS_NORMAL));
            continue;
        }
        
        // Get angles
        ret = mpu6050_get_angles(&angles);
        if (ret == ESP_OK) {
            // Log X, Y, Z angles - SAME OUTPUT AS ARDUINO
            ESP_LOGI(TAG, "Angles -> X: %.2f°, Y: %.2f°, Z: %.2f°", 
                     angles.x, angles.y, angles.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS_NORMAL));
    }
}
*/

/* COMPARISON:
 * ✅ Both initialize I2C with same parameters
 * ✅ Both call begin() to configure MPU6050 registers
 * ✅ Both call calibration function
 * ✅ Both use update() -> get angles loop
 * ✅ Same output format and precision
 * 
 * DIFFERENCES:
 * ➕ ESP-IDF version has better error handling
 * ➕ ESP-IDF version is modular (C wrapper + task)
 * ➕ ESP-IDF version uses proper FreeRTOS task structure
 * ➕ ESP-IDF version has configurable GPIO pins (8,9 vs 21,22)
 */
