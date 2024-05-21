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
 * @file hmc5883l.h
 * @defgroup drivers hmc5883l
 * @{
 *
 * ESP-IDF driver for hmc5883l sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HMC5883L definitions
*/
#define I2C_HMC5883L_TX_DATA_SIZE          UINT8_C(1)        //!< hmc5883l I2C transmit data size in bytes
#define I2C_HMC5883L_RX_DATA_SIZE          UINT8_C(6)        //!< hmc5883l I2C recieve data size in bytes
#define I2C_HMC5883L_FREQ_HZ               (100000)          //!< hmc5883l I2C default clock frequency (100KHz)

#define I2C_HMC5883L_ADDR                  UINT8_C(0x1e)     //!< hmc5883l I2C address when ADDR pin floating/low
#define I2C_HMC5883L_DEV_ID                UINT32_C(0x00333448)  //!< Chip ID, "H43"
#define I2C_HMC5883L_MEAS_TIMEOUT          (6000)
/*
 * HMC5883L enumerator and sructure declerations
*/
typedef struct i2c_hmc5883l_t i2c_hmc5883l_t;
typedef struct i2c_hmc5883l_t *i2c_hmc5883l_handle_t;

typedef uint8_t i2c_hmc5883l_tx_data_t[I2C_HMC5883L_TX_DATA_SIZE];
typedef uint8_t i2c_hmc5883l_rx_data_t[I2C_HMC5883L_RX_DATA_SIZE];

/*
 * supported registers
*/
typedef enum {
    I2C_HMC5883L_REG_CONFIG_A               = 0x00,
    I2C_HMC5883L_REG_CONFIG_B               = 0x01,
    I2C_HMC5883L_REG_MODE                   = 0x02,
    I2C_HMC5883L_REG_DATA_OUT_X_MSB         = 0x03,
    I2C_HMC5883L_REG_DATA_OUT_X_LSB         = 0x04,
    I2C_HMC5883L_REG_DATA_OUT_Z_MSB         = 0x05,
    I2C_HMC5883L_REG_DATA_OUT_Z_LSB         = 0x06,
    I2C_HMC5883L_REG_DATA_OUT_Y_MSB         = 0x07,
    I2C_HMC5883L_REG_DATA_OUT_Y_LSB         = 0x08,
    I2C_HMC5883L_REG_STATUS                 = 0x09,
    I2C_HMC5883L_REG_IDENT_A                = 0x0a,
    I2C_HMC5883L_REG_IDENT_B                = 0x0b,
    I2C_HMC5883L_REG_IDENT_C                = 0x0c
} i2c_hmc5883l_registers_t;

/*
 * supported read and write commands
*/
typedef enum {
    I2C_HMC5883L_CMD_READ                   = 0x3d,
    I2C_HMC5883L_CMD_WRITE                  = 0x3c
} i2c_hmc5883l_rw_commands_t;

/**
 * possible operating modes
 */
typedef enum {
    I2C_HMC5883L_MODE_CONTINUOUS = 0, //!< Continuous mode
    I2C_HMC5883L_MODE_SINGLE          //!< Single measurement mode, default
} i2c_hmc5883l_modes_t;

/**
 * number of samples averaged per measurement
 */
typedef enum {
    I2C_HMC5883L_SAMPLE_1 = 0, //!< 1 sample, default
    I2C_HMC5883L_SAMPLE_2,     //!< 2 samples
    I2C_HMC5883L_SAMPLE_4,     //!< 4 samples
    I2C_HMC5883L_SAMPLE_8      //!< 8 samples
} i2c_hmc5883l_sample_averages_t;


/**
 * possible data output rate in continuous measurement mode
 */
typedef enum {
    I2C_HMC5883L_DATA_RATE_00_75 = 0, //!< 0.75 Hz
    I2C_HMC5883L_DATA_RATE_01_50,     //!< 1.5 Hz
    I2C_HMC5883L_DATA_RATE_03_00,     //!< 3 Hz
    I2C_HMC5883L_DATA_RATE_07_50,     //!< 7.5 Hz
    I2C_HMC5883L_DATA_RATE_15_00,     //!< 15 Hz, default
    I2C_HMC5883L_DATA_RATE_30_00,     //!< 30 Hz
    I2C_HMC5883L_DATA_RATE_75_00,     //!< 75 Hz
    I2C_HMC5883L_DATA_RATE_220_00     //!< 220 Hz, HMC5983 only
} i2c_hmc5883l_data_rates_t;

/**
 * possible measurement mode of the device (bias)
 */
typedef enum {
    I2C_HMC5883L_BIAS_NORMAL = 0, //!< Default flow, no bias
    I2C_HMC5883L_BIAS_POSITIVE,   //!< Positive bias configuration all axes, used for self test (see datasheet)
    I2C_HMC5883L_BIAS_NEGATIVE    //!< Negative bias configuration all axes, used for self test (see datasheet)
} i2c_hmc5883l_biases_t;

/**
 * possible device gains
 */
typedef enum {
    I2C_HMC5883L_GAIN_1370 = 0, //!< 0.73 mG/LSb, range -0.88..+0.88 G
    I2C_HMC5883L_GAIN_1090,     //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
    I2C_HMC5883L_GAIN_820,      //!< 1.22 mG/LSb, range -1.9..+1.9 G
    I2C_HMC5883L_GAIN_660,      //!< 1.52 mG/LSb, range -2.5..+2.5 G
    I2C_HMC5883L_GAIN_440,      //!< 2.27 mG/LSb, range -4.0..+4.0 G
    I2C_HMC5883L_GAIN_390,      //!< 2.56 mG/LSb, range -4.7..+4.7 G
    I2C_HMC5883L_GAIN_330,      //!< 3.03 mG/LSb, range -5.6..+5.6 G
    I2C_HMC5883L_GAIN_230       //!< 4.35 mG/LSb, range -8.1..+8.1 G
} i2c_hmc5883l_gains_t;

/**
 * raw measurement result
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} i2c_hmc5883l_raw_data_t;

/**
 * measurement result, milligauss
 */
typedef struct {
    float x;
    float y;
    float z;
} i2c_hmc5883l_data_t;


/**
 * @brief i2c hmc5883l device configuration.
 */
typedef struct {
    i2c_device_config_t             dev_config;     /*!< configuration for hmc5883l device */
    i2c_hmc5883l_modes_t            mode;           /*!< operating mode */
    i2c_hmc5883l_sample_averages_t  sample;         /*!< number of samples averaged */
    i2c_hmc5883l_data_rates_t       rate;           /*!< data rate */
    i2c_hmc5883l_gains_t            gain;           /*!< used measurement mode */
    i2c_hmc5883l_biases_t           bias;           /*!< measurement mode (bias) */
} i2c_hmc5883l_config_t;

/**
 * @brief i2c hmc5883l device handle.
 */
struct i2c_hmc5883l_t {
    i2c_master_dev_handle_t         i2c_dev_handle; /*!< I2C device handle */
    i2c_hmc5883l_modes_t            mode;           /*!< operating mode */
    i2c_hmc5883l_sample_averages_t  sample;         /*!< number of samples averaged */
    i2c_hmc5883l_data_rates_t       rate;           /*!< data rate */
    i2c_hmc5883l_gains_t            gain;           /*!< used measurement mode */
    float                           gain_value;     /*!< gain value */
    i2c_hmc5883l_biases_t           bias;           /*!< measurement mode (bias) */
};


/**
 * @brief initializes an hmc5883l device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] hmc5883l_config configuration of hmc5883l device
 * @param[out] hmc5883l_handle hmc5883l device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_hmc5883l_init(i2c_master_bus_handle_t bus_handle, const i2c_hmc5883l_config_t *hmc5883l_config, i2c_hmc5883l_handle_t *hmc5883l_handle);

//esp_err_t i2c_hmc5883l_read_mode(i2c_hmc5883l_handle_t hmc5883l_handle);
//esp_err_t i2c_hmc5883l_write_mode(i2c_hmc5883l_handle_t hmc5883l_handle);

esp_err_t i2c_hmc5883l_read_mode(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_modes_t *mode);
esp_err_t i2c_hmc5883l_write_mode(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_modes_t mode);

esp_err_t i2c_hmc5883l_read_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_sample_averages_t *sample);
esp_err_t i2c_hmc5883l_write_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_sample_averages_t sample);

esp_err_t i2c_hmc5883l_read_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_data_rates_t *rate);
esp_err_t i2c_hmc5883l_write_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_data_rates_t rate);

esp_err_t i2c_hmc5883l_read_bias(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_biases_t *bias);
esp_err_t i2c_hmc5883l_write_bias(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_biases_t bias);

esp_err_t i2c_hmc5883l_read_gain(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_gains_t *gain);
esp_err_t i2c_hmc5883l_write_gain(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_gains_t gain);

esp_err_t i2c_hmc5883l_data_is_locked(i2c_hmc5883l_handle_t hmc5883l_handle, bool *locked);

esp_err_t i2c_hmc5883l_data_is_ready(i2c_hmc5883l_handle_t hmc5883l_handle, bool *ready);

esp_err_t i2c_hmc5883l_read_raw_data(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_raw_data_t *data);

/**
 * @brief removes an hmc5883l device from master bus.
 *
 * @param[in] hmc5883l_handle hmc5883l device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_hmc5883l_rm(i2c_hmc5883l_handle_t hmc5883l_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __HMC5883L_H__
