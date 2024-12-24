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
 * @file sgp4x.h
 * @defgroup drivers sgp4x
 * @{
 *
 * ESP-IDF driver for sgp4x sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SGP4X_H__
#define __SGP4X_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * SGP4X definitions
 */
#define I2C_SGP4X_SCL_SPEED_HZ          UINT32_C(100000) //!< sgp4x I2C default clock frequency (100KHz)

#define I2C_SGP4X_DEV_ADDR              UINT8_C(0x59) //!< sgp4x I2C address

/*
 * SGP4X macro definitions
 */
#define I2C_SGP41_CONFIG_DEFAULT {                          \
    .dev_config.device_address = I2C_SGP4X_DEV_ADDR,        \
    .dev_config.scl_speed_hz   = I2C_SGP4X_SCL_SPEED_HZ,    \
    .dev_version               = I2C_SGP4X_VERSION_SGP41 }


/**
 * @brief SGP4X I2C versions enumerator.
 */
typedef enum i2c_sgp4x_version_tag {
    I2C_SGP4X_VERSION_SGP40,  /*!< not implemented */
    I2C_SGP4X_VERSION_SGP41
} i2c_sgp4x_versions_t;

/**
 * @brief SGP4X I2C self-test result structure.
 */
typedef union __attribute__((packed)) {
    struct {
        bool voc_pixel_failed:1;    /*!< one or more tests have failed when true     (bit:0)  */
        bool nox_pixel_failed:1;    /*!< one or more tests have failed when true     (bit:1) */
        uint8_t reserved:6;         /*!< reserved      (bit:2-7) */
    } pixels;
    uint8_t integrity;
} i2c_sgp4x_self_test_result_t;

/**
 * @brief SGP4X I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t     dev_config; /*!< I2C configuration for sgp4x device */
    i2c_sgp4x_versions_t    dev_version; /*!< version of the sgp4x */
} i2c_sgp4x_config_t;

/**
 * @brief SGP4X I2C device structure.
 */
struct i2c_sgp4x_t {
    i2c_master_dev_handle_t     i2c_dev_handle; /*!< I2C device handle */
    i2c_sgp4x_versions_t        dev_version; /*!< version of the sgp4x device */
    uint64_t                    serial_number;
};

/**
 * @brief SGP4X I2C device structure definition.
 */
typedef struct i2c_sgp4x_t i2c_sgp4x_t;

/**
 * @brief SGP4X I2C device handle definition.
 */
typedef struct i2c_sgp4x_t *i2c_sgp4x_handle_t;


/**
 * @brief Initializes an SGP4X device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] sgp4x_config configuration of SGP4X device.
 * @param[out] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sgp4x_config_t *sgp4x_config, i2c_sgp4x_handle_t *sgp4x_handle);

/**
 * @brief Starts the conditioning with temperature and humidity compensation, i.e., the VOC pixel will be operated at 
 * the same temperature as it is by calling the sgp41_measure_raw_signals function while the NOx pixel will be operated 
 * at a different temperature for conditioning.  The conditioning should be ran when the device is initially powered for
 * 10-seconds but do not run the conditioning longer than 10-seconds, otherwise damage may occur to the SGP4X.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[in] temperature Temperature compensation in degree celcius.
 * @param[in] humidity Humidity compensation in percentage.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_execute_compensated_conditioning(i2c_sgp4x_handle_t sgp4x_handle, const float temperature, const float humidity, uint16_t *sraw_voc);

/**
 * @brief Starts the conditioning, i.e., the VOC pixel will be operated at the same temperature as it is by calling the
 * sgp41_measure_raw_signals function while the NOx pixel will be operated at a different temperature for conditioning.
 * The conditioning should be ran when the device is initially powered for 10-seconds but do not run the conditioning 
 * longer than 10-seconds, otherwise damage may occur to the SGP4X.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_execute_conditioning(i2c_sgp4x_handle_t sgp4x_handle, uint16_t *sraw_voc);

/**
 * @brief Starts and/or continues the VOC and NOX measurement mode with temperature and humidity compensation.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[in] temperature Temperature compensation in degree celcius.
 * @param[in] humidity Humidity compensation in percentage.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @param[out] sraw_nox Raw signal of NOX in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_measure_compensated_raw_signals(i2c_sgp4x_handle_t sgp4x_handle, const float temperature, const float humidity, uint16_t *sraw_voc, uint16_t *sraw_nox);

/**
 * @brief Starts and/or continues the VOC and NOX measurement mode using default temperature and humidity compensation.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @param[out] sraw_nox Raw signal of NOX in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_measure_raw_signals(i2c_sgp4x_handle_t sgp4x_handle, uint16_t *sraw_voc, uint16_t *sraw_nox);

/**
 * @brief Performs the built-in self-test that checks for integrity of 
 * both hotplate and MOX material.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[out] result Results of the self-tests.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_execute_self_test(i2c_sgp4x_handle_t sgp4x_handle, i2c_sgp4x_self_test_result_t *const result);

/**
 * @brief Turns the hotplate off, stops the measurement, and SGP4X enters idle mode.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_turn_heater_off(i2c_sgp4x_handle_t sgp4x_handle);

/**
 * @brief Issues soft-reset and initializes SGP4X.  See datasheet for details.
 *
 * @param[in] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_reset(i2c_sgp4x_handle_t sgp4x_handle);

/**
 * @brief Removes an SGP4X device from master bus.
 *
 * @param[in] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_remove(i2c_sgp4x_handle_t sgp4x_handle);

/**
 * @brief Removes an SGP4X device from master bus and frees handle.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sgp4x_delete(i2c_sgp4x_handle_t sgp4x_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __SGP4X_H__
