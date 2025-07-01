/* BLDC Control Header */

#ifndef BLDC_CONTROL_H
#define BLDC_CONTROL_H

#include "esp_err.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLDC motor and driver
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bldc_init(void);

/**
 * @brief Run BLDC control loop
 * @return ESP_OK on success
 */
esp_err_t bldc_loop(void);

/**
 * @brief Set target value for BLDC motor
 * @param target Target value (velocity in rad/s or voltage in V)
 * @return ESP_OK on success
 */
esp_err_t bldc_set_target(float target);

/**
 * @brief Get AS5600 encoder angle reading
 * @return Angle in radians, -1.0 on error
 */
float bldc_get_encoder_angle(void);

/**
 * @brief Deinitialize BLDC motor
 * @return ESP_OK on success
 */
esp_err_t bldc_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // BLDC_CONTROL_H
