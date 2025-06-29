/* LED Control Header */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "esp_err.h"
#include "led_strip.h"
#include "app_config.h"

/**
 * @brief Initialize LED strip
 */
esp_err_t led_init(void);

/**
 * @brief Set LED on/off state
 */
esp_err_t led_set_state(bool state);

/**
 * @brief Set LED RGB color (for addressable LEDs)
 */
esp_err_t led_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Deinitialize LED
 */
esp_err_t led_deinit(void);

#endif // LED_CONTROL_H
