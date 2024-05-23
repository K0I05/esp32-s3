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
 * @file sht4x.h
 * @defgroup drivers sht4x
 * @{
 *
 * ESP-IDF driver for sht4x sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SHT4X_H__
#define __SHT4X_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SHT4X definitions
*/
#define I2C_SHT4X_CRC8_G_POLYNOM        UINT8_C(0x31)     //!< sht4x I2C CRC8 polynomial
#define I2C_SHT4X_FREQ_HZ               (100000)          //!< sht4x I2C default clock frequency (100KHz)

#define I2C_SHT4X_ADDR_LO               UINT8_C(0x44)     //!< sht4x I2C address when ADDR pin floating/low
#define I2C_SHT4X_ADDR_HI               UINT8_C(0x45)     //!< sht4x I2C address when ADDR pin high

/*
 * macros
*/
#define I2C_SHT4X_CONFIG_DEFAULT {                                      \
        .dev_config.device_address     = I2C_SHT4X_ADDR_LO,             \
        .heater                        = I2C_SHT4X_HEATER_OFF,          \
        .repeatability                 = I2C_SHT4X_REPEAT_HIGH, }

/*
 * SHT4X enumerator and sructure declerations
*/
typedef struct i2c_sht4x_t i2c_sht4x_t;
typedef struct i2c_sht4x_t *i2c_sht4x_handle_t;

/*
 * supported commands
*/
typedef enum {
    /* soft reset */
    I2C_SHT4X_CMD_RESET                     = 0x94,
    /* read serial number */
    I2C_SHT4X_CMD_SERIAL                    = 0x89,
    /* measure with high precisioh (high repeatability) */
    I2C_SHT4X_CMD_MEAS_HIGH                 = 0xFD,
    I2C_SHT4X_CMD_MEAS_MED                  = 0xF6,
    I2C_SHT4X_CMD_MEAS_LOW                  = 0xE0,
    I2C_SHT4X_CMD_MEAS_H_HIGH_LONG          = 0x39,
    I2C_SHT4X_CMD_MEAS_H_HIGH_SHORT         = 0x32,
    I2C_SHT4X_CMD_MEAS_H_MED_LONG           = 0x2F,
    I2C_SHT4X_CMD_MEAS_H_MED_SHORT          = 0x24,
    I2C_SHT4X_CMD_MEAS_H_LOW_LONG           = 0x1E,
    I2C_SHT4X_CMD_MEAS_H_LOW_SHORT          = 0x15
} i2c_sht4x_commands_t;

/*
 * possible heater modes
 */
typedef enum {
    I2C_SHT4X_HEATER_OFF = 0,      /*!< heater is off, default */
    I2C_SHT4X_HEATER_HIGH_LONG,    /*!< high power (~200mW), 1 second pulse */
    I2C_SHT4X_HEATER_HIGH_SHORT,   /*!< high power (~200mW), 0.1 second pulse */
    I2C_SHT4X_HEATER_MEDIUM_LONG,  /*!< medium power (~110mW), 1 second pulse */
    I2C_SHT4X_HEATER_MEDIUM_SHORT, /*!< medium power (~110mW), 0.1 second pulse */
    I2C_SHT4X_HEATER_LOW_LONG,     /*!< low power (~20mW), 1 second pulse */
    I2C_SHT4X_HEATER_LOW_SHORT,    /*!< low power (~20mW), 0.1 second pulse */
} i2c_sht4x_heater_modes_t;

/*
 * possible repeatability modes
 */
typedef enum {
    I2C_SHT4X_REPEAT_HIGH = 0,
    I2C_SHT4X_REPEAT_MEDIUM,
    I2C_SHT4X_REPEAT_LOW
} i2c_sht4x_repeat_modes_t;

/**
 * @brief i2c sht4x device configuration.
 */
typedef struct {
    i2c_device_config_t      dev_config;       /*!< configuration for sht4x device */
    i2c_sht4x_repeat_modes_t repeatability;    /*!< used repeatability */
    i2c_sht4x_heater_modes_t heater;           /*!< used measurement mode */
} i2c_sht4x_config_t;

/**
 * @brief i2c sh4xt device handle.
 */
struct i2c_sht4x_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_sht4x_repeat_modes_t repeatability;   /*!< used repeatability */
    i2c_sht4x_heater_modes_t heater;          /*!< used measurement mode */
    uint32_t                 serial_number;    /*!< sht4x device serial number */
};


/**
 * @brief initializes an sht4x device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] sht4x_config configuration of sht4x device
 * @param[out] sht4x_handle sht4x device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_sht4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sht4x_config_t *sht4x_config, i2c_sht4x_handle_t *sht4x_handle);

/**
 * @brief soft-reset sensor
 *
 * @param[in] sht4x_handle sht4x device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_sht4x_reset(i2c_sht4x_handle_t sht4x_handle);

/**
 * @brief read serial number from sensor
 *
 * @param[in] sht4x_config configuration of sht4x device
 * @param[out] serial_number serial number of sht4x device
 * @return ESP_OK: init success.
 */
esp_err_t i2c_sht4x_read_serial_number(i2c_sht4x_handle_t sht4x_handle, uint32_t *serial_number);

/**
 * @brief high-level measurement function
 *
 * this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. starts a measurement
 * 2. waits using `vTaskDelay()` until measurement results are available
 * 3. returns the results in kind of floating point sensor values
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param[in] sht4x_handle sht4x device handle
 * @param[out] temperature temperature in degree Celsius
 * @param[out] humidity humidity in percent
 * @return ESP_OK: init success.
 */
esp_err_t i2c_sht4x_measure(i2c_sht4x_handle_t sht4x_handle, float *temperature, float *humidity);

/**
 * @brief removes an sht4x device from master bus.
 *
 * @param[in] sht4x_handle sht4x device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_sht4x_rm(i2c_sht4x_handle_t sht4x_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SHT4X_H__
