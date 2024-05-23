/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file i2c_master_ext.c
 *
 * ESP-IDF driver extension for i2c peripheral drivers
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "i2c_master_ext.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/i2c_master.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "i2c_master_ext";

/*
* functions and subrountines
*/

esp_err_t i2c_master_bus_read_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint8_t *data) {
    i2c_uint8_t tx = { reg_addr };
    i2c_uint8_t rx = { 0 };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT8_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint8 - rx[0] %02x", rx[0]);

    *data = rx[0];

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint16_t *data) {
    i2c_uint8_t tx = { reg_addr };
    i2c_uint16_t rx = { 0, 0 };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT16_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint16 - rx[0] %02x | rx[1] %02x", rx[0], rx[1]);

    *data = rx[0] | (rx[1] << 8);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_byte16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint16_t *data) {
    i2c_uint8_t tx = { reg_addr };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT16_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint16 - data[0] %02x | data[1] %02x", *data[0], *data[1]);

    return ESP_OK;   
}

esp_err_t i2c_master_bus_read_uint24(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint24_t *data) {
    i2c_uint8_t tx = { reg_addr };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT24_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint24 - data[0] %02x | data[1] %02x | data[2] %02x", *data[0], *data[1], *data[2]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_uint32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint32_t *data) {
    i2c_uint8_t tx = { reg_addr };
    i2c_uint32_t rx = { 0, 0, 0, 0 };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT32_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint32 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x", rx[0], rx[1], rx[2], rx[3]);

    *data = rx[0] | (rx[1] << 8) | (rx[2] << 16) | (rx[3] << 24);

    return ESP_OK;
}

/*
esp_err_t i2c_master_bus_read_uint48(i2c_master_dev_handle_t handle, const uint8_t command, uint8_t *data) {
    i2c_uint8_t tx = {command};
    i2c_uint48_t rx = { 0, 0, 0, 0, 0, 0 };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT48_SIZE, -1) );

    //memcpy(data, rx, I2C_UINT48_SIZE);

    ESP_LOGD(TAG, "i2c_master_bus_read_uint48 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x", rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);

    return ESP_OK;
}
*/

esp_err_t i2c_master_bus_read_uint48(i2c_master_dev_handle_t handle, const uint8_t command, i2c_uint48_t *data) {
    i2c_uint8_t tx = {command};

    ESP_ERROR_CHECK( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT48_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint48 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x", *data[0], *data[1], *data[2], *data[3], *data[4], *data[5]);

    return ESP_OK;
}


esp_err_t i2c_master_bus_write_cmd(i2c_master_dev_handle_t handle, const uint8_t command) {
    i2c_uint8_t tx = { command };

    ESP_ERROR_CHECK( i2c_master_transmit(handle, tx, I2C_UINT8_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_write_cmd - tx[0] %02x", tx[0]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_write_uint8(i2c_master_dev_handle_t handle, const uint8_t command, const uint8_t data) {
    i2c_uint16_t tx = { command, data };

    ESP_ERROR_CHECK( i2c_master_transmit(handle, tx, I2C_UINT16_SIZE, -1) );

    ESP_LOGD(TAG, "i2c_master_bus_write_uint8 - tx[0] %02x | tx[1] %02x", tx[0], tx[1]);

    return ESP_OK;
}