/* MPU9250 Sensor Control Header */

#ifndef MPU9250_CONTROL_H
#define MPU9250_CONTROL_H

#include "esp_err.h"
#include "app_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_angles_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_accel_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_gyro_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_mag_t;

/**
 * @brief Initialize I2C, TCA9548A multiplexer and MPU9250 sensor
 * @param use_multiplexer Enable TCA9548A multiplexer support
 * @param mux_channel Channel number on TCA9548A (0-7, ignored if use_multiplexer is false)
 */
esp_err_t mpu9250_init(bool use_multiplexer, uint8_t mux_channel);

/**
 * @brief Initialize I2C and MPU9250 sensor (legacy - without multiplexer)
 */
esp_err_t mpu9250_init_legacy(void);

/**
 * @brief Read sensor data
 */
esp_err_t mpu9250_read_sensor(void);

/**
 * @brief Get current angles (filtered) in degrees
 */
esp_err_t mpu9250_get_angles(mpu9250_angles_t *angles);

/**
 * @brief Get accelerometer data in m/s²
 */
esp_err_t mpu9250_get_accel(mpu9250_accel_t *accel);

/**
 * @brief Get gyroscope data in rad/s
 */
esp_err_t mpu9250_get_gyro(mpu9250_gyro_t *gyro);

/**
 * @brief Get magnetometer data in µT
 */
esp_err_t mpu9250_get_mag(mpu9250_mag_t *mag);

/**
 * @brief Get temperature in °C
 */
esp_err_t mpu9250_get_temperature(float *temp);

/**
 * @brief Calibrate gyroscope offsets
 */
esp_err_t mpu9250_calibrate_gyro(void);

/**
 * @brief Calibrate magnetometer (hard iron and soft iron)
 */
esp_err_t mpu9250_calibrate_mag(void);

/**
 * @brief Reset angles to current accelerometer reading
 */
esp_err_t mpu9250_reset_angles(void);

/**
 * @brief Deinitialize MPU9250
 */
esp_err_t mpu9250_deinit(void);

/**
 * @brief MPU9250 update task - high frequency data acquisition (1ms)
 */
void mpu9250_update_task(void *pvParameters);

/**
 * @brief MPU9250 display task - low frequency data logging (100ms)
 */
void mpu9250_display_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // MPU9250_CONTROL_H
