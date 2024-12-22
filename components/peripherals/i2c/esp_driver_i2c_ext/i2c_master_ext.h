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

#define I2C_XFR_TIMEOUT_MS  (500)          //!< I2C transaction timeout in milliseconds

#define I2C_UINT72_SIZE	    (9)
#define I2C_UINT64_SIZE	    (8)
#define I2C_UINT56_SIZE     (7)
#define I2C_UINT48_SIZE     (6)
#define I2C_UINT40_SIZE     (5)
#define I2C_UINT32_SIZE	    (4)
#define I2C_UINT24_SIZE	    (3)
#define I2C_UINT16_SIZE	    (2)
#define I2C_UINT8_SIZE	    (1)

typedef uint8_t             i2c_uint72_t[I2C_UINT72_SIZE];
typedef uint8_t             i2c_uint64_t[I2C_UINT64_SIZE];
typedef uint8_t             i2c_uint56_t[I2C_UINT56_SIZE];
typedef uint8_t             i2c_uint48_t[I2C_UINT48_SIZE];
typedef uint8_t             i2c_uint40_t[I2C_UINT40_SIZE];
typedef uint8_t             i2c_uint32_t[I2C_UINT32_SIZE];
typedef uint8_t             i2c_uint24_t[I2C_UINT24_SIZE];
typedef uint8_t             i2c_uint16_t[I2C_UINT16_SIZE];
typedef uint8_t             i2c_uint8_t[I2C_UINT8_SIZE];

/* 4-byte conversion to float IEEE754 */
typedef union {
    uint8_t bytes[4];
    float   value;
} i2c_bytes_to_float_t;

/* 4-byte conversion to uint32_t */
typedef union {
    uint8_t  bytes[4];
    uint32_t value;
} i2c_bytes_to_uint32_t;

/* 4-byte conversion to int32_t */
typedef union {
    uint8_t  bytes[4];
    int32_t  value;
} i2c_bytes_to_int32_t;

/* 2-byte conversion to uint16_t */
typedef union {
    uint8_t  bytes[2];
    uint16_t value;
} i2c_bytes_to_uint16_t;

/* 2-byte conversion to int16_t */
typedef union {
    uint8_t  bytes[2];
    int16_t  value;
} i2c_bytes_to_int16_t;


/**
 * @brief Converts `uint8_t` type to binary as a string.
 * 
 * @param n `uint8_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint8_to_binary(uint8_t n);

/**
 * @brief Converts `uint16_t` type to binary as a string.
 * 
 * @param n `uint16_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint16_to_binary(uint16_t n);

/**
 * @brief Converts `uint32_t` type to binary as a string.
 * 
 * @param n `uint32_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint32_to_binary(uint32_t n);

uint16_t bytes_to_uint16_t(const uint8_t* bytes);

uint32_t bytes_to_uint32_t(const uint8_t* bytes);

uint64_t bytes_to_uint64_t(const uint8_t* bytes);

int16_t bytes_to_int16_t(const uint8_t* bytes);

int32_t bytes_to_int32_t(const uint8_t* bytes);

int64_t bytes_to_int64_t(const uint8_t* bytes);

void uint16_t_to_bytes(const uint16_t value, uint8_t* bytes);

void uint32_t_to_bytes(const uint32_t value, uint8_t* bytes);

void uint64_t_to_bytes(const uint64_t value, uint8_t* bytes);

void int16_t_to_bytes(const int16_t value, uint8_t* bytes);

void int32_t_to_bytes(const int32_t value, uint8_t* bytes);

void int64_t_to_bytes(const int64_t value, uint8_t* bytes);

void copy_bytes(const uint8_t* source, uint8_t* destination, uint16_t data_length);

/**
 * @brief I2C master bus device detection that prints the address (1-byte) of detected devices.
 * 
 * @param handle master bus handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_detect_devices(i2c_master_bus_handle_t handle);

/**
 * @brief I2C device `uint8_t` data (1-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint8_t` (1-byte) data read from device
 * @return esp_err_t ESP_OK on success, ESP_FAIL not successful.
 */
esp_err_t i2c_master_bus_read_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint8_t *const data);

/**
 * @brief I2C device `uint16_t` data (2-byte little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint16_t` (2-byte little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint16_t *const data);

/**
 * @brief I2C device `i2c_uint16_t` data (2-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint16_t` (2-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint16_t *const data);

/**
 * @brief I2C device `i2c_uint24_t` data (3-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint24_t` (3-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte24(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint24_t *const data);

/**
 * @brief I2C device `i2c_uint24_t` data (3-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint24_t` (3-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte24(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint24_t *const data);


/**
 * @brief I2C device `uint32_t` data (4-byte little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint32_t` (4-byte little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint32_t *const data);

/**
 * @brief I2C device `i2c_uint32_t` data (4-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint32_t` (4-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint32_t *const data);

/**
 * @brief I2C device `i2c_uint48_t` data (6-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint48_t` (6-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte48(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint48_t *const data);

/**
 * @brief I2C device `i2c_uint48_t` data (6-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint48_t` (6-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte48(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint48_t *const data);

/**
 * @brief I2C device `i2c_uint64_t` data (8-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint64_t` (8-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte64(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint64_t *const data);

/**
 * @brief I2C device `i2c_uint64_t` data (8-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint64_t` (8-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte64(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint64_t *const data);


/**
 * @brief I2C device write command (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] command device command address (1-byte)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_cmd(i2c_master_dev_handle_t handle, const uint8_t command);

/**
 * @brief I2C device write command (2-byte little endian).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] command device command address (2-byte little endian)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write16_cmd(i2c_master_dev_handle_t handle, const uint16_t command);


/**
 * @brief I2C device write register (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[in] data data to write (1-byte)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint8_t data);

/**
 * @brief I2C device write register (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[in] data data to write (2-byte little endian)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint16_t data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __I2C_MASTER_EXT_H__
