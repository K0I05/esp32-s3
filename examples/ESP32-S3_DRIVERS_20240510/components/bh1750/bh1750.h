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
#define I2C_BH1750_DATA_RATE_HZ         (100000)          //!< bh1750 I2C default clock frequency (100KHz)

#define I2C_BH1750_ADDR_LO              UINT8_C(0x23)     //!< bh1750 I2C address when ADDR pin floating/low
#define I2C_BH1750_ADDR_HI              UINT8_C(0x5C)     //!< bh1750 I2C address when ADDR pin high

#define I2C_BH1750_OPCODE_HIGH          0x0
#define I2C_BH1750_OPCODE_HIGH2         0x1
#define I2C_BH1750_OPCODE_LOW           0x3

#define I2C_BH1750_OPCODE_CONT          UINT8_C(0x10)
#define I2C_BH1750_OPCODE_OT            UINT8_C(0x20)

#define I2C_BH1750_OPCODE_MT_HI         UINT8_C(0x40)
#define I2C_BH1750_OPCODE_MT_LO         UINT8_C(0x60)

#define I2C_BH1750_CMD_POWER_DOWN       UINT8_C(0x00)
#define I2C_BH1750_CMD_POWER_UP         UINT8_C(0x01)
#define I2C_BH1750_CMD_RESET            UINT8_C(0x07)
#define I2C_BH1750_CMD_MEAS_CM_HIGH     UINT8_C(0x10)
#define I2C_BH1750_CMD_MEAS_CM2_HIGH    UINT8_C(0x11)
#define I2C_BH1750_CMD_MEAS_CM_LOW      UINT8_C(0x13)
#define I2C_BH1750_CMD_MEAS_OM_HIGH     UINT8_C(0x20)
#define I2C_BH1750_CMD_MEAS_OM2_HIGH    UINT8_C(0x21)
#define I2C_BH1750_CMD_MEAS_OM_LOW      UINT8_C(0x23)

/*
 * macros
*/
#define I2C_BH1750_CONFIG_DEFAULT {                                     \
        .dev_config.device_address     = I2C_BH1750_ADDR_LO,            \
        .mode                          = I2C_BH1750_MODE_CONTINUOUS,    \
        .resolution                    = I2C_BH1750_RES_HIGH, }

/*
 * SHT4X enumerator and sructure declerations
*/
typedef struct i2c_bh1750_t i2c_bh1750_t;
typedef struct i2c_bh1750_t *i2c_bh1750_handle_t;

/**
 * possible measurement modes
 */
typedef enum {
    I2C_BH1750_MODE_ONE_TIME = 0, //!< one time measurement
    I2C_BH1750_MODE_CONTINUOUS    //!< continuous measurement
} i2c_bh1750_modes_t;

/**
 * possible measurement resolutions
 */
typedef enum {
    I2C_BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is usually 16 ms
    I2C_BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is usually 120 ms
    I2C_BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is usually 180 ms
} i2c_bh1750_resolutions_t;

/**
 * @brief i2c bh1750 device configuration.
 */
typedef struct {
    i2c_device_config_t      dev_config;        /*!< configuration for bh1750 device */
    i2c_bh1750_modes_t       mode;              /*!< bh1750 measurement mode */
    i2c_bh1750_resolutions_t resolution;        /*!< bh1750 measurement resolution */
} i2c_bh1750_config_t;

/**
 * @brief i2c bh1750 device configuration parameters.
 */
typedef struct {
    i2c_bh1750_modes_t       mode;              /*!< bh1750 measurement mode */
    i2c_bh1750_resolutions_t resolution;        /*!< bh1750 measurement resolution */
} i2c_bh1750_params_t;

/**
 * @brief i2c bh1750 device handle.
 */
struct i2c_bh1750_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_bh1750_params_t      *dev_params;     /*!< bh1750 device configuration parameters */
};

/**
 * @brief initializes an bh1750 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] bh1750_config configuration of bh1750 device
 * @param[out] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_init(i2c_master_bus_handle_t bus_handle, const i2c_bh1750_config_t *bh1750_config, i2c_bh1750_handle_t *bh1750_handle);

/**
 * @brief soft-reset bh1750 sensor
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_reset(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief power-up bh1750 sensor
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_power_up(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief power-down bh1750 sensor
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_power_down(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief measure bh1750 illuminance.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @param[out] illuminance bh1750 illuminance measurement
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_get_measurement(i2c_bh1750_handle_t bh1750_handle, uint16_t *illuminance);

/**
 * @brief removes an bh1750 device from master bus.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_rm(i2c_bh1750_handle_t bh1750_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BH1750_H__
