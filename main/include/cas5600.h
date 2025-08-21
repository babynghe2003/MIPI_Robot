/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "common/base_classes/Sensor.h"
#include "driver/gpio.h"
#include "i2c_bus.h"

class CAS5600 : public Sensor {
public:
    /**
     * @brief Construct a new as5600 object
     *
     */
    CAS5600();

    /**
     * @brief Destroy the as5600 object
     *
     */
    ~CAS5600();

    /**
     * @brief Init i2c for as5600
     *
     * @param i2c_bus
     */
    void init();

    /**
     * @brief Set i2c for as5600
     *
     * @param i2c_bus
     */
    void setBus(i2c_bus_handle_t i2c_bus);
    /**
     * @brief Deinit i2c for as5600
     *
     */

    void deinit();

    /**
     * @brief Get the output of as5600
     *
     * @return float
     */
    float getSensorAngle();

private:
    i2c_bus_handle_t _i2c_bus;
    i2c_bus_device_handle_t _i2c_device;
    bool _is_installed;
    uint16_t _raw_angle;
    float _angle;
};
