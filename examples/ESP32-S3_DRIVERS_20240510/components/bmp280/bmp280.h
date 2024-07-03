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
 * @file bmp280.h
 * @defgroup drivers bmp280
 * @{
 *
 * ESP-IDF driver for bmp280 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP280_H__
#define __BMP280_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMP280 definitions
*/
#define I2C_BMP280_DATA_RATE_HZ               (100000)          //!< bmp280 I2C default clock frequency (100KHz)

/*
 * BMP280 macros
*/
#define I2C_BMP280_PARAMS_DEFAULT {                                     \
        .mode = I2C_BMP280_MODE_NORMAL,                                 \
        .filter = I2C_BMP280_FILTER_OFF,                                \
        .oversampling_pressure = I2C_BMP280_OVERSAMPLING_STANDARD,      \
        .oversampling_temperature = I2C_BMP280_OVERSAMPLING_STANDARD,   \
        .standby = I2C_BMP280_STANDBY_250, }

#define I2C_BMP280_CONFIG_DEFAULT {                                     \
        .dev_config.device_address = I2C_BMP280_ADDR_HI,                \
        .dev_params = I2C_BMP280_PARAMS_DEFAULT, }


/*
 * BMP280 enumerator and sructure declerations
*/

/*
 * supported device addresses
*/
#define I2C_BMP280_ADDR_LO          0x76 //!< bmp280 I2C address when ADDR pin floating/low
#define I2C_BMP280_ADDR_HI          0x77 //!< bmp280 I2C address when ADDR pin high

/**
 * possible BMP280 registers
 */
#define I2C_BMP280_REG_TEMP_XLSB    0xFC /* bits: 7-4 */
#define I2C_BMP280_REG_TEMP_LSB     0xFB
#define I2C_BMP280_REG_TEMP_MSB     0xFA
#define I2C_BMP280_REG_TEMP         (I2C_BMP280_REG_TEMP_MSB)
#define I2C_BMP280_REG_PRESS_XLSB   0xF9 /* bits: 7-4 */
#define I2C_BMP280_REG_PRESS_LSB    0xF8
#define I2C_BMP280_REG_PRESS_MSB    0xF7
#define I2C_BMP280_REG_PRESSURE     (I2C_BMP280_REG_PRESS_MSB)
#define I2C_BMP280_REG_CONFIG       0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define I2C_BMP280_REG_CTRL         0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define I2C_BMP280_REG_STATUS       0xF3 /* bits: 3 measuring; 0 im_update */
#define I2C_BMP280_REG_CTRL_HUM     0xF2 /* bits: 2-0 osrs_h; */
#define I2C_BMP280_REG_RESET        0xE0
#define I2C_BMP280_REG_ID           0xD0
#define I2C_BMP280_REG_CALIB        0x88
#define I2C_BMP280_REG_HUM_CALIB    0x88
#define I2C_BMP280_RESET_VALUE      0xB6

#define I2C_BMP280_TYPE_BMP280      0x58  //!< BMP280
#define I2C_BMP280_TYPE_BME280      0x60  //!< BME280

typedef struct i2c_bmp280_t i2c_bmp280_t;
typedef struct i2c_bmp280_t *i2c_bmp280_handle_t;

/**
 * possible modes of operation
 */
typedef enum {
    I2C_BMP280_MODE_SLEEP = 0,  //!< sleep mode
    I2C_BMP280_MODE_FORCED = 1, //!< measurement is initiated by user
    I2C_BMP280_MODE_NORMAL = 3  //!< continues measurement
} i2c_bmp280_modes_t;

/**
 * possible filter settings
 */
typedef enum {
    I2C_BMP280_FILTER_OFF = 0,
    I2C_BMP280_FILTER_2 = 1,
    I2C_BMP280_FILTER_4 = 2,
    I2C_BMP280_FILTER_8 = 3,
    I2C_BMP280_FILTER_16 = 4
} i2c_bmp280_filters_t;

/**
 * possible pressure oversampling settings
 */
typedef enum {
    I2C_BMP280_OVERSAMPLING_SKIPPED = 0,          //!< no measurement
    I2C_BMP280_OVERSAMPLING_ULTRA_LOW_POWER = 1,  //!< oversampling x1
    I2C_BMP280_OVERSAMPLING_LOW_POWER = 2,        //!< oversampling x2
    I2C_BMP280_OVERSAMPLING_STANDARD = 3,         //!< oversampling x4
    I2C_BMP280_OVERSAMPLING_HIGH_RES = 4,         //!< oversampling x8
    I2C_BMP280_OVERSAMPLING_ULTRA_HIGH_RES = 5    //!< oversampling x16
} i2c_bmp280_oversampling_t;

/**
 * possible stand-by-time between measurements in normal mode
 */
typedef enum {
    I2C_BMP280_STANDBY_05 = 0,      //!< stand by time 0.5ms
    I2C_BMP280_STANDBY_62 = 1,      //!< stand by time 62.5ms
    I2C_BMP280_STANDBY_125 = 2,     //!< stand by time 125ms
    I2C_BMP280_STANDBY_250 = 3,     //!< stand by time 250ms
    I2C_BMP280_STANDBY_500 = 4,     //!< stand by time 500ms
    I2C_BMP280_STANDBY_1000 = 5,    //!< stand by time 1s
    I2C_BMP280_STANDBY_2000 = 6,    //!< stand by time 2s BMP280, 10ms BME280
    I2C_BMP280_STANDBY_4000 = 7,    //!< stand by time 4s BMP280, 20ms BME280
} i2c_bmp280_standbytimes_t;

/**
 * configuration parameters for BMP280 module.
 * use macro ::I2C_BMP280_PARAMS_DEFAULT to use default configuration.
 */
typedef struct {
    i2c_bmp280_modes_t        mode;
    i2c_bmp280_filters_t      filter;
    i2c_bmp280_oversampling_t oversampling_pressure;
    i2c_bmp280_oversampling_t oversampling_temperature;
    i2c_bmp280_standbytimes_t standby;
} i2c_bmp280_params_t;

/**
 * @brief BMP280 temperature and pressure calibration factors structure.
 */
typedef struct {
    /* temperature and pressure compensation */
    uint16_t                dig_T1;
    int16_t                 dig_T2;
    int16_t                 dig_T3;
    uint16_t                dig_P1;
    int16_t                 dig_P2;
    int16_t                 dig_P3;
    int16_t                 dig_P4;
    int16_t                 dig_P5;
    int16_t                 dig_P6;
    int16_t                 dig_P7;
    int16_t                 dig_P8;
    int16_t                 dig_P9;
} i2c_bmp280_cal_factors_t;

typedef struct {
    i2c_device_config_t     dev_config;         /*!< I2C configuration for bmp280 device */
    i2c_bmp280_params_t     dev_params;         /*!< bmp280 device configuration parameters */
} i2c_bmp280_config_t;

struct i2c_bmp280_t {
    i2c_master_dev_handle_t  i2c_dev_handle;    /*!< I2C device handle */
    i2c_bmp280_cal_factors_t *dev_cal_factors;  /*!< bmp280 device calibration factors */
    uint8_t                  dev_type;          /*!< device type, should be bmp280 */
};


/**
 * @brief initializes an bmp280 device onto the master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] bmp280_config configuration of sht4x device
 * @param[out] bmp280_handle bmp280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp280_config_t *bmp280_config, i2c_bmp280_handle_t *bmp280_handle);

/**
 * @brief soft-reset sensor
 *
 * @param[in] bmp280_handle bmp280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_reset(i2c_bmp280_handle_t bmp280_handle);


/**
 * @brief high-level measurement read function for bmp280
 *
 * @param[in] bmp280_handle bmp280 device handle
 * @param[out] temperature temperature in degree Celsius
 * @param[out] pressure pressure in pascal
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_measurement(i2c_bmp280_handle_t bmp280_handle, float *temperature, float *pressure);

/**
 * @brief forces the bmp280 to make a measurement
 *
 * @param[in] bmp280_handle bmp280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_forced_measurement(i2c_bmp280_handle_t bmp280_handle);

/**
 * @brief determines if the bmp280 is busy
 *
 * @param[in] bmp280_handle bmp280 device handle
 * @param[out] busy bmp280 is busy when true
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_is_measuring(i2c_bmp280_handle_t bmp280_handle, bool *busy);

/**
 * @brief removes an bmp280 device from master bus.
 *
 * @param[in] bmp280_handle bmp280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_rm(i2c_bmp280_handle_t bmp280_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP280_H__
