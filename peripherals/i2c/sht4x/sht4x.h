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
#define I2C_SHT4X_CRC8_G_POLYNOM        UINT8_C(0x31)   //!< sht4x I2C CRC8 polynomial
#define I2C_SHT4X_DATA_RATE_HZ          (100000)        //!< sht4x I2C default clock frequency (100KHz)

#define I2C_SHT4X_ADDR_LO               UINT8_C(0x44)   //!< sht4x I2C address when ADDR pin floating/low
#define I2C_SHT4X_ADDR_HI               UINT8_C(0x45)   //!< sht4x I2C address when ADDR pin high

#define I2C_SHT4X_CMD_RESET             UINT8_C(0x94)   //!< sht4x I2C soft-reset command 
#define I2C_SHT4X_CMD_SERIAL            UINT8_C(0x89)   //!< sht4x I2C serial number request command
#define I2C_SHT4X_CMD_MEAS_HIGH         UINT8_C(0xFD)   //!< sht4x I2C high resolution measurement command
#define I2C_SHT4X_CMD_MEAS_MED          UINT8_C(0xF6)   //!< sht4x I2C medium resolution measurement command
#define I2C_SHT4X_CMD_MEAS_LOW          UINT8_C(0xE0)   //!< sht4x I2C low resolution measurement command
#define I2C_SHT4X_CMD_MEAS_H_HIGH_LONG  UINT8_C(0x39)   //!< sht4x I2C high resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_HIGH_SHORT UINT8_C(0x32)   //!< sht4x I2C high resolution measurement command with heater enabled short pulse
#define I2C_SHT4X_CMD_MEAS_H_MED_LONG   UINT8_C(0x2F)   //!< sht4x I2C medium resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_MED_SHORT  UINT8_C(0x24)   //!< sht4x I2C medium resolution measurement command with heater enabled short pulse
#define I2C_SHT4X_CMD_MEAS_H_LOW_LONG   UINT8_C(0x1E)   //!< sht4x I2C low resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_LOW_SHORT  UINT8_C(0x15)   //!< sht4x I2C low resolution measurement command with heater enabled short pulse

#define I2C_SHT4X_RESET_DELAY_MS        (10)
#define I2C_SHT4X_CMD_DELAY_MS          (20)
#define I2C_SHT4X_TX_RX_DELAY_MS        (10)


/*
 * SHT4X macro definitions
*/

/**
 * @brief Macro that initializes `i2c_sht4x_config_t` to default configuration settings.
 */
#define I2C_SHT4X_CONFIG_DEFAULT {                                      \
        .dev_config.device_address     = I2C_SHT4X_ADDR_LO,             \
        .heater                        = I2C_SHT4X_HEATER_OFF,          \
        .repeatability                 = I2C_SHT4X_REPEAT_HIGH, }

/*
 * SHT4X enumerator and sructure declerations
*/

/** 
 * @brief SHT4X heater modes enumerator.
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

/**
  * @brief SHT4X repeatability modes enumerator.
 */
typedef enum {
    I2C_SHT4X_REPEAT_HIGH = 0,                  /*!< high repeatability */
    I2C_SHT4X_REPEAT_MEDIUM,                    /*!< medium repeatability */
    I2C_SHT4X_REPEAT_LOW                        /*!< low repeatability */
} i2c_sht4x_repeat_modes_t;


/**
 * @brief SHT4X I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t      dev_config;       /*!< configuration for sht4x device */
    i2c_sht4x_repeat_modes_t repeatability;    /*!< measurement repeatability mode */
    i2c_sht4x_heater_modes_t heater;           /*!< measurement heater mode */
} i2c_sht4x_config_t;

/**
 * @brief SHT4X I2C device structure.
 */
struct i2c_sht4x_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_sht4x_repeat_modes_t repeatability;   /*!< measurement repeatability mode */
    i2c_sht4x_heater_modes_t heater;          /*!< measurement heater mode */
    uint32_t                 serial_number;   /*!< sht4x device serial number */
};

/**
 * @brief SHT4X I2C device structure definition.
 */
typedef struct i2c_sht4x_t i2c_sht4x_t;
/**
 * @brief SHT4X I2C device handle structure.
 */
typedef struct i2c_sht4x_t *i2c_sht4x_handle_t;


/**
 * @brief Initializes an SHT4X device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] sht4x_config configuration of sht4x device.
 * @param[out] sht4x_handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sht4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sht4x_config_t *sht4x_config, i2c_sht4x_handle_t *sht4x_handle);

/**
 * @brief Issues soft-reset to SHT4X.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sht4x_reset(i2c_sht4x_handle_t sht4x_handle);

/**
 * @brief Read high-level measurements from SHT4X.
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @param[out] temperature temperature in degree Celsius.
 * @param[out] humidity humidity in percent.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sht4x_get_measurement(i2c_sht4x_handle_t sht4x_handle, float *temperature, float *humidity);

/**
 * @brief Similar to `i2c_sht4x_read_measurement` but it includes dewpoint in the results.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @param[out] temperature temperature in degree Celsius.
 * @param[out] humidity humidity in percent.
 * @param[out] dewpoint calculated dewpoint temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sht4x_get_measurements(i2c_sht4x_handle_t sht4x_handle, float *temperature, float *humidity, float *dewpoint);

/**
 * @brief Removes an SHT4X device from master I2C bus.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_sht4x_rm(i2c_sht4x_handle_t sht4x_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SHT4X_H__
