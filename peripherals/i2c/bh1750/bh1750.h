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
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BH1750 definitions
*/
#define I2C_BH1750_TX_DATA_SIZE     UINT8_C(1)        //!< bh1750 I2C transmit data size in bytes
#define I2C_BH1750_RX_DATA_SIZE     UINT8_C(2)        //!< bh1750 I2C recieve data size in bytes
#define I2C_BH1750_FREQ_HZ          (100000)          //!< bh1750 I2C default clock frequency (100KHz)

#define I2C_BH1750_ADDR_LO          UINT8_C(0x23)     //!< bh1750 I2C address when ADDR pin floating/low
#define I2C_BH1750_ADDR_HI          UINT8_C(0x5C)     //!< bh1750 I2C address when ADDR pin high

#define I2C_BH1750_OPCODE_HIGH      0x0
#define I2C_BH1750_OPCODE_HIGH2     0x1
#define I2C_BH1750_OPCODE_LOW       0x3

#define I2C_BH1750_OPCODE_CONT      0x10
#define I2C_BH1750_OPCODE_OT        0x20

#define I2C_BH1750_OPCODE_MT_HI     0x40
#define I2C_BH1750_OPCODE_MT_LO     0x60

/*
 * SHT4X enumerator and sructure declerations
*/
typedef uint8_t i2c_bh1750_tx_data_t[I2C_BH1750_TX_DATA_SIZE];
typedef uint8_t i2c_bh1750_rx_data_t[I2C_BH1750_RX_DATA_SIZE];

typedef struct i2c_bh1750_t i2c_bh1750_t;
typedef struct i2c_bh1750_t *i2c_bh1750_handle_t;

/*
 * supported commands
*/
typedef enum {
    I2C_BH1750_CMD_POWER_DOWN               = 0x00,
    I2C_BH1750_CMD_POWER_UP                 = 0x01,
    I2C_BH1750_CMD_RESET                    = 0x07,
    I2C_BH1750_CMD_MEAS_CM_HIGH             = 0x10,
    I2C_BH1750_CMD_MEAS_CM2_HIGH            = 0x11,
    I2C_BH1750_CMD_MEAS_CM_LOW              = 0x13,
    I2C_BH1750_CMD_MEAS_OM_HIGH             = 0x20,
    I2C_BH1750_CMD_MEAS_OM2_HIGH            = 0x21,
    I2C_BH1750_CMD_MEAS_OM_LOW              = 0x23
} i2c_bh1750_commands_t;

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
    i2c_device_config_t      dev_config;  /*!< configuration for bh1750 device */
    i2c_bh1750_modes_t       mode;        /*!< bh1750 measurement mode */
    i2c_bh1750_resolutions_t resolution;  /*!< bh1750 measurement resolution */
} i2c_bh1750_config_t;

/**
 * @brief i2c bh1750 device handle.
 */
struct i2c_bh1750_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_bh1750_modes_t       mode;            /*!< bh1750 measurement mode */
    i2c_bh1750_resolutions_t resolution;      /*!< bh1750 measurement resolution */
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
 * @brief configures bh1750 sensor measurement mode and resolution
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_setup(i2c_bh1750_handle_t bh1750_handle);

/**
 * @brief sets bh1750 sensor measurement time. see datasheet for details.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @param[in] time bh1750 measurement timespan
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_set_measurement_time(i2c_bh1750_handle_t bh1750_handle, uint8_t time);

/**
 * @brief measure bh1750 illuminance.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @param[out] illuminance bh1750 illuminance measurement
 * @return ESP_OK: init success.
 */
esp_err_t i2c_bh1750_measure(i2c_bh1750_handle_t bh1750_handle, uint16_t *illuminance);

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
