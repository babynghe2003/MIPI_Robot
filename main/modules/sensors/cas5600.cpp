/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cas5600.h"
#include "esp_check.h"

#define PI 3.14159265358979f
static const char *TAG = "AS5600";

CAS5600::CAS5600()
{
    _is_installed = false;
}

CAS5600::~CAS5600()
{
    if (_is_installed) {
        deinit();
    }
}

void CAS5600::setBus(i2c_bus_handle_t i2c_bus){
    _i2c_bus = i2c_bus;
}

void CAS5600::init()
{
    ESP_RETURN_ON_FALSE(_i2c_bus != NULL,, TAG, "I2C bus create fail");
    _i2c_device = i2c_bus_device_create(_i2c_bus, 0x36, 0);
    ESP_RETURN_ON_FALSE(_i2c_device != NULL,, TAG, "AS5600 device create fail");
    _is_installed = true;
}

void CAS5600::deinit()
{
    i2c_bus_device_delete(&_i2c_device);
    ESP_RETURN_ON_FALSE(_i2c_device == NULL,, TAG, "AS5600 device delete fail");
    _is_installed = false;
}

float CAS5600::getSensorAngle()
{
    uint8_t raw_angle_buf[2] = {0};
    if (i2c_bus_read_bytes(_i2c_device, 0x0C, 2, raw_angle_buf) != ESP_OK) {
        return -1.0f;
    }
    _raw_angle = (uint16_t)(raw_angle_buf[0] << 8 | raw_angle_buf[1]);
    _angle = (((int)_raw_angle & 0b0000111111111111) * 360.0f / 4096.0f) * (PI / 180.0f);
    return _angle;
}
