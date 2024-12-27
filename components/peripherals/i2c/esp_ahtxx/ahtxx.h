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
 * @file ahtxx.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for ahtxx sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AHTXX_H__
#define __AHTXX_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * AHTXX definitions
 */
#define I2C_AHTXX_SCL_SPEED_HZ          UINT32_C(100000) //!< ahtxx I2C default clock frequency (100KHz)

#define I2C_AHTXX_DEV_ADDR              UINT8_C(0x38) //!< ahtxx I2C address


/*
 * AHTXX macro definitions
 */
#define I2C_AHT2X_CONFIG_DEFAULT {                      \
    .dev_config.device_address = I2C_AHTXX_DEV_ADDR,    \
    .dev_config.scl_speed_hz   = I2C_AHTXX_SCL_SPEED_HZ,\
    .aht_type = I2C_AHTXX_AHT2X }

#define I2C_AHT10_CONFIG_DEFAULT {                      \
    .dev_config.device_address = I2C_AHTXX_DEV_ADDR,    \
    .dev_config.scl_speed_hz   = I2C_AHTXX_SCL_SPEED_HZ,\
    .aht_type = I2C_AHTXX_AHT10 }

/*
* AHTXX enumerator and sructure declerations
*/
typedef enum {
    I2C_AHTXX_AHT10,
    I2C_AHTXX_AHT2X
} i2c_ahtxx_types_t;

/**
 * @brief AHTXX I2C commands enumerator.
 */
typedef enum {
    I2C_AHTXX_CMD_AHT10_INIT    = (0xe1),   /*!< aht10 initialization command + 0x08 + 0x00 */
    I2C_AHTXX_CMD_AHT2X_INIT    = (0xbe),   /*!< aht2x initialization command + 0x08 + 0x00 */
    I2C_AHTXX_CMD_STATUS        = (0x71),       /*!< ahtxx status register command */
    I2C_AHTXX_CMD_TRIGGER_MEAS  = (0xac), /*!< ahtxx measurement trigger command + 0x33 + 0x00 */
    I2C_AHTXX_CMD_RESET         = (0xba)         /*!< ahtxx soft-reset command */
} i2c_ahtxx_commands_t;

/**
 * @brief AHTXX I2C status register structure.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:3; /*!< reserved                       (bit:0-2)  */
        bool calibrated:1;   /*!< aht2x is calibrated when true  (bit:3) */
        uint8_t reserved2:3; /*!< reserved                       (bit:4-6) */
        bool busy:1;         /*!< aht2x is busy when true        (bit:7) */
    } bits;
    uint8_t reg;
} i2c_ahtxx_status_register_t;

/**
 * @brief AHTXX I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t dev_config; /*!< I2C configuration for aht2x device */
    i2c_ahtxx_types_t   aht_type;
} i2c_ahtxx_config_t;

/**
 * @brief AHTXX I2C device structure.
 */
struct i2c_ahtxx_t {
    i2c_master_dev_handle_t     i2c_dev_handle; /*!< I2C device handle */
    i2c_ahtxx_types_t           aht_type;
    i2c_ahtxx_status_register_t status_reg; /*!< status register */
};

/**
 * @brief AHTXX I2C device structure definition.
 */
typedef struct i2c_ahtxx_t i2c_ahtxx_t;

/**
 * @brief AHTXX I2C device handle definition.
 */
typedef struct i2c_ahtxx_t *i2c_ahtxx_handle_t;


/**
 * @brief AHTXX initialization and calibration setup.  This is a one-time call at start-up if the device isn't initialized and calibrated.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_setup(i2c_ahtxx_handle_t ahtxx_handle);

/**
 * @brief Read status register from AHTXX.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_status_register(i2c_ahtxx_handle_t ahtxx_handle);

/**
 * @brief Initializes an AHTXX device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ahtxx_config Configuration of AHTXX device.
 * @param[out] ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_init(i2c_master_bus_handle_t bus_handle, const i2c_ahtxx_config_t *ahtxx_config, i2c_ahtxx_handle_t *ahtxx_handle);

/**
 * @brief Reads temperature and relative humidity from AHTXX.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @param temperature Temperature in degree Celsius.
 * @param humidity Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_measurement(i2c_ahtxx_handle_t ahtxx_handle, float *const temperature, float *const humidity);

/**
 * @brief Similar to `i2c_aht2x_read_measurement` but it includes dewpoint in the results.
 *
 * @param[in] ahtxx_handle AHTXX device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dewpoint temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_measurements(i2c_ahtxx_handle_t ahtxx_handle, float *const temperature, float *const humidity, float *const dewpoint);

/**
 * @brief Reads busy status flag from AHTXX.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_busy_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const busy);

/**
 * @brief Reads calibration status flag from AHTXX.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_calibration_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const calibrated);

/**
 * @brief Reads busy and calibrated status flags from AHTXX.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const busy, bool *const calibrated);

/**
 * @brief Issues soft-reset and initializes AHTXX.  See datasheet for details.
 *
 * @param ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_reset(i2c_ahtxx_handle_t ahtxx_handle);

/**
 * @brief Removes an AHTXX device from master bus.
 *
 * @param[in] ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_remove(i2c_ahtxx_handle_t ahtxx_handle);

/**
 * @brief Removes an AHTXX device from master bus and frees handle.
 * 
 * @param ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_delete(i2c_ahtxx_handle_t ahtxx_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AHTXX_H__
