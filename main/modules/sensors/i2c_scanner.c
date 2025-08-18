/* I2C Bus Debug Scanner */

#include "esp_log.h"
#include "i2c_bus.h"
#include "app_config.h"

static const char *TAG = "I2C_Scanner";

/**
 * @brief Scan I2C bus for all devices (similar to Arduino I2C Scanner)
 * @param bus_handle I2C bus handle
 */
void i2c_scan_bus(i2c_bus_handle_t bus_handle) {
    ESP_LOGI(TAG, "=== I2C Bus Scanner ===");
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    int device_count = 0;
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Create temporary device handle for testing
        i2c_bus_device_handle_t test_device = i2c_bus_device_create(bus_handle, addr, 0);
        if (test_device) {
            // Try to write/read from the device to test if it exists
            uint8_t test_data = 0x00;
            esp_err_t write_ret = i2c_bus_write_bytes(test_device, NULL_I2C_MEM_ADDR, 1, &test_data);
            
            if (write_ret == ESP_OK) {
                device_count++;
                const char* device_name = "Unknown";
                
                // Identify common devices
                switch (addr) {
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
                        device_name = "TCA9548A/PCA9548A Multiplexer";
                        break;
                }
                
                ESP_LOGI(TAG, "I2C device found at address 0x%02X - %s", addr, device_name);
            }
            
            i2c_bus_device_delete(&test_device);
        }
    }
    
    if (device_count == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    } else {
        ESP_LOGI(TAG, "Scan complete. Found %d device(s)", device_count);
    }
    
    ESP_LOGI(TAG, "=== End of I2C Scan ===");
}

/**
 * @brief Simple I2C scanner with independent bus creation
 */
void i2c_scanner_standalone(void) {
    ESP_LOGI(TAG, "Starting standalone I2C scanner...");
    
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
        ESP_LOGE(TAG, "Failed to create I2C bus for scanning");
        return;
    }

    // Scan the bus
    i2c_scan_bus(bus_handle);
    
    // Cleanup
    i2c_bus_delete(&bus_handle);
}
