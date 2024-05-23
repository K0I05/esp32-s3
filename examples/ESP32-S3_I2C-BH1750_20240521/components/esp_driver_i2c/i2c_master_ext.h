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
 * @file i2c_master_ext.h
 * @defgroup i2c_master i2c_master_ext
 * @{
 *
 * ESP-IDF driver extension for i2c peripheral drivers
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __I2C_MASTER_EXT_H__
#define __I2C_MASTER_EXT_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_UINT64_SIZE	(8)
#define I2C_UINT48_SIZE (6)
#define I2C_UINT32_SIZE	(4)
#define I2C_UINT24_SIZE	(3)
#define I2C_UINT16_SIZE	(2)
#define I2C_UINT8_SIZE	(1)

typedef uint8_t     i2c_uint64_t[I2C_UINT64_SIZE];
typedef uint8_t     i2c_uint48_t[I2C_UINT48_SIZE];
typedef uint8_t     i2c_uint32_t[I2C_UINT32_SIZE];
typedef uint8_t     i2c_uint24_t[I2C_UINT24_SIZE];
typedef uint8_t     i2c_uint16_t[I2C_UINT16_SIZE];
typedef uint8_t     i2c_uint8_t[I2C_UINT8_SIZE];


//typedef uint8_t     i2c_uint48_t[I2C_UINT48_SIZE];

/**
 * @brief I2C device data (1-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data data (1-byte) read from device
 * @return ESP_OK: init success. ESP_FAIL: not success.
 */
esp_err_t i2c_master_bus_read_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint8_t *data);

/**
 * @brief I2C device data (2-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data data (2-byte) read from device
 * @return ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint16_t *data);

esp_err_t i2c_master_bus_read_byte16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint16_t *data);

/**
 * @brief I2C device data (3-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data data (3-byte) read from device
 * @return ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint24(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint24_t *data);


/**
 * @brief I2C device data (4-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data data (4-byte) read from device
 * @return ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint32_t *data);


/**
 * @brief I2C device data (6-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] command device command (1-byte)
 * @param[out] data data (6-byte) read from device
 * @return ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
//esp_err_t i2c_master_bus_read_uint48(i2c_master_dev_handle_t handle, const uint8_t command, uint8_t *data);
esp_err_t i2c_master_bus_read_uint48(i2c_master_dev_handle_t handle, const uint8_t command, i2c_uint48_t *data);


/**
 * @brief I2C device write command (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] command device command (1-byte)
 * @return ESP_OK: init success.
 */
esp_err_t i2c_master_bus_write_cmd(i2c_master_dev_handle_t handle, const uint8_t command);

/**
 * @brief I2C device write command (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @return ESP_OK: init success.
 */
esp_err_t i2c_master_bus_write_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint8_t data);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __I2C_MASTER_EXT_H__
