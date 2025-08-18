/* AS5600 Sensor - Custom implementation for shared I2C bus */

#ifndef AS5600_H
#define AS5600_H

#include "i2c_bus.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// AS5600 I2C address
#define AS5600_I2C_ADDR         0x36

// AS5600 register addresses
#define AS5600_REG_RAW_ANGLE    0x0C    // Raw angle register (2 bytes)
#define AS5600_REG_STATUS       0x0B    // Status register
#define AS5600_REG_MAGNITUDE    0x1B    // Magnitude register (2 bytes)

typedef struct {
    i2c_bus_handle_t bus_handle;
    i2c_bus_device_handle_t device_handle;
    uint16_t raw_angle;
    float angle_rad;
    bool initialized;
    const char* name;  // "left" or "right"
} as5600_t;

/**
 * @brief Initialize AS5600 sensor with shared I2C bus
 * @param bus_handle Shared I2C bus handle
 * @param name Sensor name ("left" or "right")
 * @return AS5600 handle or NULL on failure
 */
as5600_t* as5600_init(i2c_bus_handle_t bus_handle, const char* name);

/**
 * @brief Read angle from AS5600
 * @param sensor AS5600 sensor
 * @return ESP_OK on success
 */
esp_err_t as5600_read(as5600_t* sensor);

/**
 * @brief Get angle in radians
 * @param sensor AS5600 sensor
 * @return Angle in radians (0 to 2π)
 */
float as5600_get_angle_rad(as5600_t* sensor);

/**
 * @brief Get angle in degrees
 * @param sensor AS5600 sensor
 * @return Angle in degrees (0 to 360)
 */
float as5600_get_angle_deg(as5600_t* sensor);

/**
 * @brief Get raw angle value (0-4095)
 * @param sensor AS5600 sensor
 * @return Raw angle value
 */
uint16_t as5600_get_raw_angle(as5600_t* sensor);

/**
 * @brief Check if AS5600 is connected and responding
 * @param sensor AS5600 sensor
 * @return ESP_OK if connected
 */
esp_err_t as5600_test_connection(as5600_t* sensor);

/**
 * @brief Deinitialize AS5600 sensor (does not delete I2C bus)
 * @param sensor AS5600 sensor
 * @return ESP_OK on success
 */
esp_err_t as5600_deinit(as5600_t* sensor);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
