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
 * @file bh1750.h
 * @defgroup drivers bh1750
 * @{
 *
 * ESP-IDF driver for bh1750 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BH1750_H__
#define __BH1750_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BH1750 definitions
*/
#define I2C_BH1750_SCL_SPEED_HZ         UINT32_C(100000)    //!< bh1750 I2C default clock frequency (100KHz)

#define I2C_BH1750_DEV_ADDR_LO          UINT8_C(0x23)       //!< bh1750 I2C address when ADDR pin floating/low
#define I2C_BH1750_DEV_ADDR_HI          UINT8_C(0x5C)       //!< bh1750 I2C address when ADDR pin high



/*
 * macros definitions
*/

#define I2C_BH1750_CONFIG_DEFAULT {                                     \
        .dev_config.device_address  = I2C_BH1750_DEV_ADDR_LO,           \
        .dev_config.scl_speed_hz    = I2C_BH1750_SCL_SPEED_HZ,          \
        .mode                       = I2C_BH1750_MODE_CM_HI_RESOLUTION, \
        .power_enabled              = true,                             \
        .set_timespan               = false }

/*
 * BH1750 enumerator and sructure declerations
*/

/**
 * @brief BH1750 measurement modes enumerator.
 * 
 */
typedef enum {
    I2C_BH1750_MODE_OM_HI_RESOLUTION  = (0b00100000), //!< one time measurement high resolution (1 lx) mode, goes into power down mode after measurement
    I2C_BH1750_MODE_OM_HI2_RESOLUTION = (0b00100001), //!< one time measurement high resolution (0.5 lx) mode 2, goes into power down mode after measurement
    I2C_BH1750_MODE_OM_LO_RESOLUTION  = (0b00100011), //!< one time measurement low resolution (4 lx) mode, goes into power down mode after measurement
    I2C_BH1750_MODE_CM_HI_RESOLUTION  = (0b00010000), //!< continuous measurement high resolution (1 lx) mode
    I2C_BH1750_MODE_CM_HI2_RESOLUTION = (0b00010001), //!< continuous measurement high resolution (0.5 lx) mode 2
    I2C_BH1750_MODE_CM_LO_RESOLUTION  = (0b00010011)  //!< continuous measurement low resolution (4 lx) mode
} i2c_bh1750_measurement_modes_t;

/**
 * @brief BH1750 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t             dev_config;     /*!< configuration for bh1750 device */
    i2c_bh1750_measurement_modes_t  mode;           /*!< bh1750 measurement mode */
    uint8_t                         timespan;       /*!< bh1750 measurement time duration */
    bool                            set_timespan;   /*!< set bh1750 measurement timespan when true */
    bool                            power_enabled;  /*!< bh1750 powered up at initialization */
} i2c_bh1750_config_t;

/**
 * @brief BH1750 I2C device handle structure.
 */
struct i2c_bh1750_t {
    i2c_master_dev_handle_t         i2c_dev_handle;  /*!< I2C device handle */
    i2c_bh1750_measurement_modes_t  mode;           /*!< bh1750 measurement mode */
    uint8_t                         timespan;       /*!< bh1750 measurement time duration */
    bool                            power_enabled;  /*!< bh1750 powered up at initialization */
};

/**
 * @brief BH1750 I2C device structure definition.
 */
typedef struct i2c_bh1750_t i2c_bh1750_t;

/**
 * @brief BH1750 I2C device handle definition.
 */
typedef struct i2c_bh1750_t *i2c_bh1750_handle_t;


/**
 * @brief Writes measurement mode to bh1750.
 *
 * @param[in] bh1750_handle bh1750 device handle.
 * @param[in] mode bh1750 measurement mode.
 * @return ESP_OK on success.
 */
esp_err_t i2c_bh1750_set_measurement_mode(i2c_bh1750_handle_t bh1750_handle, const i2c_bh1750_measurement_modes_t mode);

/**
 * @brief sets bh1750 sensor measurement time. see datasheet for details.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @param[in] timespan bh1750 measurement time duration from 31 to 254 (typical 69)
 * @return ESP_OK on success.
 */
esp_err_t i2c_bh1750_set_measurement_time(i2c_bh1750_handle_t bh1750_handle, const uint8_t timespan);

/**
 * @brief initializes an BH1750 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] bh1750_config configuration of BH1750 device
 * @param[out] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_init(i2c_master_bus_handle_t bus_handle, const i2c_bh1750_config_t *bh1750_config, i2c_bh1750_handle_t *bh1750_handle);

/**
 * @brief soft-reset BH1750 sensor. Reset command is not acceptable in power-down mode.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_reset(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief power-up BH1750 sensor.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @return esp_err_t  ESP_OK on success.
 */
esp_err_t i2c_bh1750_power_up(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief power-down BH1750 sensor.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_power_down(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief measure BH1750 illuminance.  BH1750 goes into power-down mode after measurement when one-time measurements are configured.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @param[out] lux BH1750 illuminance measurement
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_get_ambient_light(i2c_bh1750_handle_t bh1750_handle, float *const lux);

/**
 * @brief removes an BH1750 device from master bus.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_rm(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief removes an BH1750 device from master bus and frees handle.
 *
 * @param[in] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bh1750_del(i2c_bh1750_handle_t bh1750_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BH1750_H__
