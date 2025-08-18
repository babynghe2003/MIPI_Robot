/* TCA9548A I2C Multiplexer Control Header */

#ifndef TCA9548A_H
#define TCA9548A_H

#include "esp_err.h"
#include "i2c_bus.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// TCA9548A I2C address (can be 0x70-0x77 depending on A0,A1,A2 pins)
#define TCA9548A_DEFAULT_ADDR   0x70

// TCA9548A channels (0-7)
#define TCA9548A_CHANNEL_0      0
#define TCA9548A_CHANNEL_1      1
#define TCA9548A_CHANNEL_2      2
#define TCA9548A_CHANNEL_3      3
#define TCA9548A_CHANNEL_4      4
#define TCA9548A_CHANNEL_5      5
#define TCA9548A_CHANNEL_6      6
#define TCA9548A_CHANNEL_7      7

// Special channels
#define TCA9548A_CHANNEL_NONE   0xFF  // Disable all channels

typedef struct {
    i2c_bus_handle_t bus_handle;
    i2c_bus_device_handle_t device_handle;
    uint8_t current_channel;
    bool initialized;
} tca9548a_handle_t;

/**
 * @brief Initialize TCA9548A multiplexer
 * @param bus_handle I2C bus handle
 * @param addr TCA9548A I2C address (default: 0x70)
 * @return TCA9548A handle or NULL on failure
 */
tca9548a_handle_t* tca9548a_init(i2c_bus_handle_t bus_handle, uint8_t addr);

/**
 * @brief Select channel on TCA9548A
 * @param handle TCA9548A handle
 * @param channel Channel number (0-7) or TCA9548A_CHANNEL_NONE to disable all
 * @return ESP_OK on success
 */
esp_err_t tca9548a_select_channel(tca9548a_handle_t* handle, uint8_t channel);

/**
 * @brief Get currently selected channel
 * @param handle TCA9548A handle
 * @return Current channel number or TCA9548A_CHANNEL_NONE
 */
uint8_t tca9548a_get_current_channel(tca9548a_handle_t* handle);

/**
 * @brief Scan for devices on a specific channel
 * @param handle TCA9548A handle
 * @param channel Channel to scan
 * @param devices Array to store found device addresses
 * @param max_devices Maximum number of devices to find
 * @return Number of devices found
 */
int tca9548a_scan_channel(tca9548a_handle_t* handle, uint8_t channel, uint8_t* devices, int max_devices);

/**
 * @brief Deinitialize TCA9548A multiplexer
 * @param handle TCA9548A handle
 * @return ESP_OK on success
 */
esp_err_t tca9548a_deinit(tca9548a_handle_t* handle);

/**
 * @brief Scan all channels on TCA9548A and report found devices
 * @param bus_handle I2C bus handle
 * @param mux_addr TCA9548A address
 */
void tca9548a_scan_all_channels(i2c_bus_handle_t bus_handle, uint8_t mux_addr);

/**
 * @brief Quick test of TCA9548A multiplexer initialization
 * @return ESP_OK if multiplexer is working
 */
esp_err_t tca9548a_quick_test(void);

/**
 * @brief Scan I2C bus for all devices (debug function)
 * @param bus_handle I2C bus handle
 */
void i2c_scan_bus(i2c_bus_handle_t bus_handle);

/**
 * @brief Simple I2C scanner with independent bus creation
 */
void i2c_scanner_standalone(void);

#ifdef __cplusplus
}
#endif

#endif // TCA9548A_H
