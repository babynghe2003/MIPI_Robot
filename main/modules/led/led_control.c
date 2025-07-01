/* LED Control Implementation */

#include "led_control.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "led_control";
static led_strip_handle_t led_strip = NULL;
static bool led_initialized = false;

esp_err_t led_init(void) {
    if (led_initialized) {
        ESP_LOGW(TAG, "LED already initialized");
        return ESP_OK;
    }

#ifdef CONFIG_BLINK_LED_STRIP
    ESP_LOGI(TAG, "Initializing addressable LED strip");
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_PIN,
        .max_leds = 1,
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT LED strip device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "RMT LED strip device created successfully");
#endif

    led_strip_clear(led_strip);
    
#endif

    led_initialized = true;
    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_GPIO_PIN);
    return ESP_OK;
}

esp_err_t led_set_state(bool state) {
    if (!led_initialized) {
        ESP_LOGE(TAG, "LED not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_BLINK_LED_STRIP
    if (state) {
        led_strip_set_pixel(led_strip, 0, 5,5,10);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
#endif

    return ESP_OK;
}

esp_err_t led_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (!led_initialized) {
        ESP_LOGE(TAG, "LED not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_BLINK_LED_STRIP
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
    return ESP_OK;
#else
    ESP_LOGW(TAG, "Color setting only available for addressable LEDs");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t led_deinit(void) {
    if (!led_initialized) {
        return ESP_OK;
    }

#ifdef CONFIG_BLINK_LED_STRIP
    if (led_strip) {
        led_strip_clear(led_strip);
        // Note: led_strip component doesn't provide explicit delete function
        led_strip = NULL;
    }
#endif

    led_initialized = false;
    ESP_LOGI(TAG, "LED deinitialized");
    return ESP_OK;
}
