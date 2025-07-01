/* BLDC Control Implementation - C++ wrapper for SimpleFOC */

#include "bldc_control.h"
#include "esp_log.h"
#include "driver/gpio.h"

// C++ SimpleFOC includes
#include "esp_simplefoc.h"

static const char *TAG = "bldc_control";

// Global SimpleFOC objects (similar to example main.cpp)
static BLDCMotor motor(7, 10, 150);  // 7 pole pairs
static BLDCDriver3PWM driver(5, 6, 7);  // pins A=5, B=6, C=7
static AS5600 as5600(I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);  // Use I2C_NUM_1 to avoid conflict with MPU9250

static bool bldc_initialized = false;
static float target_value = 0.0f;

extern "C" {

esp_err_t bldc_init(void) {
    if (bldc_initialized) {
        ESP_LOGW(TAG, "BLDC already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLDC motor system...");

    // Initialize AS5600 sensor
    ESP_LOGI(TAG, "Initializing AS5600 sensor...");
    as5600.init();
    ESP_LOGI(TAG, "AS5600 sensor initialized successfully");
    
    // Link sensor to motor
    motor.linkSensor(&as5600);
    ESP_LOGI(TAG, "AS5600 sensor linked to motor");

    // Configure driver (similar to example)
    driver.voltage_power_supply = 12;  // 12V supply
    
    ESP_LOGI(TAG, "Initializing BLDC driver...");
    driver.init();
    
    // Link driver to motor
    motor.linkDriver(&driver);
    ESP_LOGI(TAG, "Driver linked to motor");

    // Configure motor for velocity control (as requested)
    motor.controller = MotionControlType::torque;
    
    motor.voltage_limit = 6;           // Motor voltage limit as requested

    ESP_LOGI(TAG, "Motor configuration: P=%.1f, I=%.1f, D=%.3f", 
             motor.PID_velocity.P, motor.PID_velocity.I, motor.PID_velocity.D);

    // Initialize motor
    ESP_LOGI(TAG, "Initializing motor...");
    motor.init();
    
    // Initialize FOC
    ESP_LOGI(TAG, "Initializing FOC...");
    motor.initFOC();

    bldc_initialized = true;
    ESP_LOGI(TAG, "BLDC motor system initialized successfully");
    
    return ESP_OK;
}

esp_err_t bldc_loop(void) {
    if (!bldc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Run FOC loop (similar to example)
    motor.loopFOC();
    motor.move(target_value);
    
    return ESP_OK;
}

esp_err_t bldc_set_target(float target) {
    if (!bldc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    target_value = target;
    ESP_LOGD(TAG, "Target set to: %.2f", target_value);
    return ESP_OK;
}

float bldc_get_encoder_angle(void) {
    if (!bldc_initialized) {
        return -1.0f;
    }
    
    // Get angle from AS5600 sensor
    return as5600.getSensorAngle();
}

esp_err_t bldc_deinit(void) {
    if (!bldc_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLDC motor system");
    bldc_initialized = false;
    target_value = 0.0f;
    
    return ESP_OK;
}

} // extern "C"
