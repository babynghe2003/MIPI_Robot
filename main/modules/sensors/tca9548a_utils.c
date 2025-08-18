/* TCA9548A Utility Functions */

#include "tca9548a.h"
#include "app_config.h"
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "TCA9548A_Utils";

/**
 * @brief Scan all channels on TCA9548A and report found devices
 * @param bus_handle I2C bus handle
 * @param mux_addr TCA9548A address
 */
void tca9548a_scan_all_channels(i2c_bus_handle_t bus_handle, uint8_t mux_addr) {
    ESP_LOGI(TAG, "=== TCA9548A Full Channel Scan ===");
    
    tca9548a_handle_t* mux_handle = tca9548a_init(bus_handle, mux_addr);
    if (!mux_handle) {
        ESP_LOGE(TAG, "Failed to initialize TCA9548A for scanning");
        return;
    }
    
    uint8_t devices[20];  // Max 20 devices per channel
    
    for (uint8_t channel = 0; channel <= 7; channel++) {
        int device_count = tca9548a_scan_channel(mux_handle, channel, devices, 20);
        
        if (device_count > 0) {
            ESP_LOGI(TAG, "Channel %d: %d device(s) found", channel, device_count);
            for (int i = 0; i < device_count; i++) {
                const char* device_name = "Unknown";
                
                // Identify common devices
                switch (devices[i]) {
                    case 0x68:
                    case 0x69:
                        device_name = "MPU9250/MPU6050/MPU6500";
                        break;
                    case 0x77:
                    case 0x76:
                        device_name = "BMP280/BME280";
                        break;
                    case 0x48:
                    case 0x49:
                    case 0x4A:
                    case 0x4B:
                        device_name = "ADS1115/ADS1015";
                        break;
                    case 0x3C:
                    case 0x3D:
                        device_name = "SSD1306 OLED";
                        break;
                    case 0x50:
                    case 0x51:
                    case 0x52:
                    case 0x53:
                    case 0x54:
                    case 0x55:
                    case 0x56:
                    case 0x57:
                        device_name = "EEPROM";
                        break;
                    case 0x70:
                    case 0x71:
                    case 0x72:
                    case 0x73:
                    case 0x74:
                    case 0x75:
                        device_name = "TCA9548A/PCA9548A";
                        break;
                }
                
                ESP_LOGI(TAG, "  0x%02X - %s", devices[i], device_name);
            }
        } else {
            ESP_LOGI(TAG, "Channel %d: No devices found", channel);
        }
    }
    
    // Disable all channels after scanning
    tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_NONE);
    tca9548a_deinit(mux_handle);
    
    ESP_LOGI(TAG, "=== Scan Complete ===");
}

/**
 * @brief Quick test of TCA9548A multiplexer initialization
 * @return ESP_OK if multiplexer is working
 */
esp_err_t tca9548a_quick_test(void) {
    ESP_LOGI(TAG, "Testing TCA9548A multiplexer...");
    
    // Initialize I2C bus
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };

    i2c_bus_handle_t bus_handle = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (!bus_handle) {
        ESP_LOGE(TAG, "Failed to create I2C bus for testing");
        return ESP_ERR_INVALID_STATE;
    }

    // Test TCA9548A
    tca9548a_handle_t* mux_handle = tca9548a_init(bus_handle, TCA9548A_I2C_ADDR);
    if (!mux_handle) {
        ESP_LOGE(TAG, "TCA9548A test FAILED - unable to initialize");
        i2c_bus_delete(&bus_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // Test channel selection
    esp_err_t ret = ESP_OK;
    for (uint8_t ch = 0; ch <= 7; ch++) {
        esp_err_t ch_ret = tca9548a_select_channel(mux_handle, ch);
        if (ch_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select channel %d", ch);
            ret = ch_ret;
            break;
        }
    }

    // Disable all channels
    tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_NONE);
    
    // Cleanup
    tca9548a_deinit(mux_handle);
    i2c_bus_delete(&bus_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TCA9548A test PASSED - all channels working");
    }
    
    return ret;
}
