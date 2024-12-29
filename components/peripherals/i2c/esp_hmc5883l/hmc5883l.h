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
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HMC5883L definitions
*/
#define I2C_HMC5883L_SCL_SPEED_HZ               UINT32_C(100000)                //!< hmc5883l I2C default clock frequency (100KHz)

#define I2C_HMC5883L_DEV_ADDR                   UINT8_C(0x1e)           //!< hmc5883l I2C address when ADDR pin floating/low


/*
 * macro definitions
*/
#define I2C_HMC5883L_CONFIG_DEFAULT {                                   \
        .dev_config.device_address      = I2C_HMC5883L_DEV_ADDR,        \
        .dev_config.scl_speed_hz        = I2C_HMC5883L_SCL_SPEED_HZ,    \
        .mode                           = I2C_HMC5883L_MODE_CONTINUOUS, \
        .sample                         = I2C_HMC5883L_SAMPLE_4,        \
        .rate                           = I2C_HMC5883L_DATA_RATE_15_00, \
        .gain                           = I2C_HMC5883L_GAIN_390,        \
        .bias                           = I2C_HMC5883L_BIAS_NORMAL,     \
        .declination                    = -16.0f }

/*
 * HMC5883L enumerator and sructure declerations
*/

/**
 * HMC5883L possible calibration options
 */
typedef enum {
    I2C_HMC5883L_CAL_GAIN_DIFF = 1, /*!< calculates the diffrence in the gain of the each axis magnetometer axis */
    I2C_HMC5883L_CAL_AXES_MEAN = 2, /*!< calculates the mean of each axes magnetic field, when the Magnetometer is rotated 360 degree */
    I2C_HMC5883L_CAL_BOTH      = 3  /*!< do both */
} i2c_hmc5883l_calibration_options_t;

/**
 * HMC5883L possible operating modes
 */
typedef enum {
    I2C_HMC5883L_MODE_CONTINUOUS = (0b00), //!< Continuous mode
    I2C_HMC5883L_MODE_SINGLE     = (0b01), //!< Single measurement mode, default
    I2C_HMC5883L_MODE_IDLE       = (0b10), //!< Idle mode
    I2C_HMC5883L_MODE_IDLE2      = (0b11), //!< Idle mode
} i2c_hmc5883l_modes_t;

/**
 * HMC5883L number of samples averaged per measurement
 */
typedef enum {
    I2C_HMC5883L_SAMPLE_1 = (0b00), //!< 1 sample, default
    I2C_HMC5883L_SAMPLE_2 = (0b01), //!< 2 samples
    I2C_HMC5883L_SAMPLE_4 = (0b10), //!< 4 samples
    I2C_HMC5883L_SAMPLE_8 = (0b11)  //!< 8 samples
} i2c_hmc5883l_sample_averages_t;

/**
 * HMC5883L possible data output rate in continuous measurement mode
 */
typedef enum {
    I2C_HMC5883L_DATA_RATE_00_75  = (0b000), //!< 0.75 Hz
    I2C_HMC5883L_DATA_RATE_01_50  = (0b001), //!< 1.5 Hz
    I2C_HMC5883L_DATA_RATE_03_00  = (0b010), //!< 3 Hz
    I2C_HMC5883L_DATA_RATE_07_50  = (0b011), //!< 7.5 Hz
    I2C_HMC5883L_DATA_RATE_15_00  = (0b100), //!< 15 Hz, default
    I2C_HMC5883L_DATA_RATE_30_00  = (0b101), //!< 30 Hz
    I2C_HMC5883L_DATA_RATE_75_00  = (0b110), //!< 75 Hz
    I2C_HMC5883L_DATA_RATE_RESERVED = (0b111)  //!< 220 Hz, HMC5983 only
} i2c_hmc5883l_data_rates_t;

/**
 * HMC5883L possible measurement mode of the device (bias)
 */
typedef enum {
    I2C_HMC5883L_BIAS_NORMAL   = (0b00), //!< Default flow, no bias
    I2C_HMC5883L_BIAS_POSITIVE = (0b01),   //!< Positive bias configuration all axes, used for self test (see datasheet)
    I2C_HMC5883L_BIAS_NEGATIVE = (0b10),    //!< Negative bias configuration all axes, used for self test (see datasheet)
    I2C_HMC5883L_BIAS_RESERVED = (0b11)
} i2c_hmc5883l_biases_t;

/**
 * HMC5883L possible device gains
 */
typedef enum {
    I2C_HMC5883L_GAIN_1370 = (0b000), //!< 0.73 mG/LSb, range -0.88..+0.88 G
    I2C_HMC5883L_GAIN_1090 = (0b001), //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
    I2C_HMC5883L_GAIN_820  = (0b010), //!< 1.22 mG/LSb, range -1.9..+1.9 G
    I2C_HMC5883L_GAIN_660  = (0b011), //!< 1.52 mG/LSb, range -2.5..+2.5 G
    I2C_HMC5883L_GAIN_440  = (0b100), //!< 2.27 mG/LSb, range -4.0..+4.0 G
    I2C_HMC5883L_GAIN_390  = (0b101), //!< 2.56 mG/LSb, range -4.7..+4.7 G
    I2C_HMC5883L_GAIN_330  = (0b110), //!< 3.03 mG/LSb, range -5.6..+5.6 G
    I2C_HMC5883L_GAIN_230  = (0b111)  //!< 4.35 mG/LSb, range -8.1..+8.1 G
} i2c_hmc5883l_gains_t;

/**
 * @brief HMC5883L configuration 1 (a) register structure.
 */
typedef union __attribute__((packed)) {
    struct CFG1_REG_BITS_TAG {
        i2c_hmc5883l_biases_t           bias:2;       /*!< measurement configuration, measurement bias (bit:0-1)   */
        i2c_hmc5883l_data_rates_t       data_rate:3;  /*!< data rate at which data is written          (bit:2-4)   */
        i2c_hmc5883l_sample_averages_t  sample_avg:2; /*!< number of samples averaged                  (bit:5-6)   */
        uint8_t                         reserved:1;   /*!< reserved and set to 0                       (bit:7)     */
    } bits;            /*!< represents the 8-bit configuration 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit configuration 1 register as `uint8_t`.   */
} i2c_hmc5883l_configuration1_register_t;

/**
 * @brief HMC5883L configuration 2 (b) register structure.
 */
typedef union __attribute__((packed)) {
    struct CFG2_REG_BITS_TAG {
        uint8_t                         reserved:5;   /*!< reserved and set to 0                       (bit:0-4)     */
        i2c_hmc5883l_gains_t            gain:3;       /*!< gain configuration for all channels         (bit:5-7)   */
    } bits;            /*!< represents the 8-bit configuration 2 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit configuration 2 register as `uint8_t`.   */
} i2c_hmc5883l_configuration2_register_t;

/**
 * @brief HMC5883L mode register structure.
 */
typedef union __attribute__((packed)) {
    struct MODE_REG_BITS_TAG {
        i2c_hmc5883l_modes_t            mode:2;        /*!< operation mode                              (bit:0-1)   */
        uint8_t                         high_speed:6;  /*!< set high to enable i2c high speed (3400khz) (bit:2-7)   */
    } bits;            /*!< represents the 8-bit mode register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit mode register as `uint8_t`.   */
} i2c_hmc5883l_mode_register_t;

/**
 * @brief HMC5883L status register structure.
 */
typedef union __attribute__((packed)) {
    struct STATUS_REG_BITS_TAG {
        bool            data_ready:1;     /*!< data is ready when asserted to true        (bit:0)   */
        bool            data_locked:1;    /*!< data is locked when asserted to true        (bit:1)   */
        uint8_t         reserved:6;       /*!< reserved (bit:2-7)   */
    } bits;            /*!< represents the 8-bit status register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status register as `uint8_t`.   */
} i2c_hmc5883l_status_register_t;

/**
 * HMC5883L raw measurement result
 */
typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} i2c_hmc5883l_axes_data_t;

/**
 * HMC5883L measurement result, milligauss
 */
typedef struct {
    float x_axis;   /*!< x axis mG */
    float y_axis;   /*!< y axis mG */
    float z_axis;   /*!< z axis mG */
    float heading;  /*!< heading in degrees */
} i2c_hmc5883l_magnetic_axes_data_t;

typedef struct {
    float x_axis;   /*!< x axis */
    float y_axis;   /*!< y axis */
    float z_axis;   /*!< z axis */
} i2c_hmc5883l_offset_axes_data_t;

typedef struct {
    float x_axis;   /*!< x axis */
    float y_axis;   /*!< y axis */
    float z_axis;   /*!< z axis */
} i2c_hmc5883l_gain_error_axes_data_t;

/**
 * @brief i2c HMC5883L device configuration structure.
 */
typedef struct {
    i2c_device_config_t             dev_config;     /*!< configuration for hmc5883l device */
    i2c_hmc5883l_modes_t            mode;           /*!< operating mode */
    i2c_hmc5883l_sample_averages_t  sample;         /*!< number of samples averaged */
    i2c_hmc5883l_data_rates_t       rate;           /*!< data rate */
    i2c_hmc5883l_gains_t            gain;           /*!< used measurement mode */
    i2c_hmc5883l_biases_t           bias;           /*!< measurement mode (bias) */
    float                           declination;    /*!< magnetic declination angle http://www.magnetic-declination.com/ */
} i2c_hmc5883l_config_t;

/**
 * @brief i2c HMC5883L device structure.
 */
struct i2c_hmc5883l_t {
    i2c_master_dev_handle_t                 i2c_dev_handle;     /*!< I2C device handle */
    i2c_hmc5883l_configuration1_register_t  config1_reg;
    i2c_hmc5883l_configuration2_register_t  config2_reg;
    i2c_hmc5883l_mode_register_t            mode_reg;
    i2c_hmc5883l_status_register_t          status_reg;
    uint32_t                                dev_id;
    float                                   declination;        /*!< magnetic declination angle http://www.magnetic-declination.com/ */
    bool                                    gain_calibrated;
    bool                                    offset_calibrated;
    i2c_hmc5883l_offset_axes_data_t         offset_axes;
    i2c_hmc5883l_gain_error_axes_data_t     gain_error_axes;
};

/**
 * @brief i2c HMC5883L device definition.
 */
typedef struct i2c_hmc5883l_t i2c_hmc5883l_t;

/**
 * @brief i2c HMC5883L device handle definition.
 */
typedef struct i2c_hmc5883l_t *i2c_hmc5883l_handle_t;


/**
 * @brief Reads configuration 1 register from HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_configuration1_register(i2c_hmc5883l_handle_t hmc5883l_handle);

/**
 * @brief Writes configuration 1 register to HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @param[in] config1_reg Configuration 1 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_configuration1_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_configuration1_register_t config1_reg);

/**
 * @brief Reads configuration 2 register from HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_configuration2_register(i2c_hmc5883l_handle_t hmc5883l_handle);

/**
 * @brief Writes configuration 2 register to HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @param[in] config2_reg Configuration 2 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_configuration2_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_configuration2_register_t config2_reg);

/**
 * @brief Reads mode register from HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_mode_register(i2c_hmc5883l_handle_t hmc5883l_handle);

/**
 * @brief Writes mode register to HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @param[in] mode_reg Mode register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_mode_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_mode_register_t mode_reg);

/**
 * @brief Reads status register from HMC5883L.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_status_register(i2c_hmc5883l_handle_t hmc5883l_handle);

/**
 * @brief Initializes an HMC5883L device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] hmc5883l_config Configuration of HMC5883L device.
 * @param[out] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_init(i2c_master_bus_handle_t bus_handle, const i2c_hmc5883l_config_t *hmc5883l_config, i2c_hmc5883l_handle_t *hmc5883l_handle);

/**
 * @brief Reads uncompensated axes measurements from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param axes_data Uncompensated axes measurements (x, y, and z axes).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_fixed_magnetic_axes(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_axes_data_t *const axes_data);

/**
 * @brief Reads compensated magnetic axes measurements from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param magnetic_axes_data Compensated magnetic axes measurements (x, y, and z axes).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_magnetic_axes(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_magnetic_axes_data_t *const magnetic_axes_data);


/* under test */

esp_err_t i2c_hmc5883l_get_calibrated_offsets(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_calibration_options_t option);



/**
 * @brief Reads data status from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param ready HMC5883L data is ready.
 * @param locked HMC5883L data is locked.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_data_status(i2c_hmc5883l_handle_t hmc5883l_handle, bool *const ready, bool *const locked);

/**
 * @brief Reads operating mode setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param mode HMC5883L operating mode setting.
 * @return esp_err_t ESP_OK on success.
 */

esp_err_t i2c_hmc5883l_get_mode(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_modes_t *const mode);
/**
 * @brief Writes operating mode setting to HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param mode HMC5883L operating mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_mode(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_modes_t mode);

/**
 * @brief Reads samples averaged setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param sample HMC5883L samples averaged setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_sample_averages_t *const sample);

/**
 * @brief Writes samples averaged setting to HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param sample HMC5883L samples averaged setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_sample_averages_t sample);

/**
 * @brief Reads data rate setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param rate HMC5883L data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_data_rates_t *const rate);

/**
 * @brief Writes data rate setting to HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param rate HMC5883L data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_data_rates_t rate);

/**
 * @brief Reads measurement mode bias setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param bias HMC5883L measurement mode bias setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_bias(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_biases_t *const bias);

/**
 * @brief Writes measurement mode bias setting to HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param bias HMC5883L measurement mode bias setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_bias(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_biases_t bias);

/**
 * @brief Reads gain setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param gain HMC5883L gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_gain(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_gains_t *const gain);

/**
 * @brief Writes gain setting to HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param gain HMC5883L gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_set_gain(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_gains_t gain);

/**
 * @brief Reads gain sensitivity setting from HMC5883L.
 * 
 * @param hmc5883l_handle HMC5883L device handle.
 * @param sensitivity HMC5883L gain sensitivity setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_get_gain_sensitivity(i2c_hmc5883l_handle_t hmc5883l_handle, float *const sensitivity);

/**
 * @brief Removes an HMC5883L device from master bus.
 *
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_remove(i2c_hmc5883l_handle_t hmc5883l_handle);

/**
 * @brief Removes an HMC5883L device from master bus and frees handle.
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_hmc5883l_delete(i2c_hmc5883l_handle_t hmc5883l_handle);




#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __HMC5883L_H__
