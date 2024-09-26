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
 * @file mmc56x3.h
 * @defgroup drivers mmc56x3
 * @{
 *
 * ESP-IDF driver for mmc56x3 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MMC56X3_H__
#define __MMC56X3_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MMC56X3 definitions
*/
#define I2C_MMC56X3_DATA_RATE_HZ          (100000)        //!< mmc56x3 I2C default clock frequency (100KHz)

#define I2C_MMC56X3_ADDR                  UINT8_C(0x30)   //!< mmc56x3 I2C address

#define I2C_MMC56X3_STARTUP_DELAY_MS      (10)           //!< mmc56x3 I2C delay in milliseconds start-up
#define I2C_MMC56X3_APPSTART_DELAY_MS     (10)           //!< mmc56x3 I2C delay in milliseconds app-start
#define I2C_MMC56X3_RESET_DELAY_MS        (20)           //!< mmc56x3 I2C delay in milliseconds after reset
#define I2C_MMC56X3_SETRESET_DELAY_MS     (1)            //!< mmc56x3 I2C delay in milliseconds after set-reset transaction
#define I2C_MMC56X3_WRITE_DELAY_MS        (1)            //!< mmc56x3 I2C delay in milliseconds after write transaction
#define I2C_MMC56X3_DATA_READY_DELAY_MS   (1)            //!< mmc56x3 1ms when checking data ready in a loop
#define I2C_MMC56X3_DATA_POLL_TIMEOUT_MS  (100)          //!< mmc56x3 100ms timeout when making a measurement

#define I2C_MMC56X3_REG_XOUT_0_R            UINT8_C(0x00)   //!< mmc56x3 I2C Xout[19:12]
#define I2C_MMC56X3_REG_XOUT_1_R            UINT8_C(0x01)   //!< mmc56x3 I2C Xout[11:4]
#define I2C_MMC56X3_REG_YOUT_0_R            UINT8_C(0x02)   //!< mmc56x3 I2C Yout[19:12]
#define I2C_MMC56X3_REG_YOUT_1_R            UINT8_C(0x03)   //!< mmc56x3 I2C Yout[11:4]
#define I2C_MMC56X3_REG_ZOUT_0_R            UINT8_C(0x04)   //!< mmc56x3 I2C Zout[19:12]
#define I2C_MMC56X3_REG_ZOUT_1_R            UINT8_C(0x05)   //!< mmc56x3 I2C Zout[11:4]
#define I2C_MMC56X3_REG_XOUT_2_R            UINT8_C(0x06)   //!< mmc56x3 I2C Xout[3:0]
#define I2C_MMC56X3_REG_YOUT_2_R            UINT8_C(0x07)   //!< mmc56x3 I2C Yout[3:0]
#define I2C_MMC56X3_REG_ZOUT_2_R            UINT8_C(0x08)   //!< mmc56x3 I2C Zout[3:0]
#define I2C_MMC56X3_REG_TOUT_R              UINT8_C(0x09)   //!< mmc56x3 I2C temperature output
#define I2C_MMC56X3_REG_STATUS_1_R          UINT8_C(0x18)   //!< mmc56x3 I2C device status 1
#define I2C_MMC56X3_REG_ODR_W               UINT8_C(0x1a)   //!< mmc56x3 I2C output data rate
#define I2C_MMC56X3_REG_CONTROL_0_W         UINT8_C(0x1b)   //!< mmc56x3 I2C control register 0
#define I2C_MMC56X3_REG_CONTROL_1_W         UINT8_C(0x1c)   //!< mmc56x3 I2C control register 1
#define I2C_MMC56X3_REG_CONTROL_2_W         UINT8_C(0x1d)   //!< mmc56x3 I2C control register 2
#define I2C_MMC56X3_REG_ST_X_TH_W           UINT8_C(0x1e)   //!< mmc56x3 I2C x-axis selftest threshold
#define I2C_MMC56X3_REG_ST_Y_TH_W           UINT8_C(0x1f)   //!< mmc56x3 I2C y-axis selftest threshold
#define I2C_MMC56X3_REG_ST_Z_TH_W           UINT8_C(0x20)   //!< mmc56x3 I2C z-axis selftest threshold
#define I2C_MMC56X3_REG_ST_X_SV_RW          UINT8_C(0x27)   //!< mmc56x3 I2C x-axis selftest set-value
#define I2C_MMC56X3_REG_ST_Y_SV_RW          UINT8_C(0x28)   //!< mmc56x3 I2C y-axis selftest set-value
#define I2C_MMC56X3_REG_ST_Z_SV_RW          UINT8_C(0x29)   //!< mmc56x3 I2C z-axis selftest set-value
#define I2C_MMC56X3_REG_PRODUCT_ID_R        UINT8_C(0x39)   //!< mmc56x3 I2C product identifier

/*
 * MMC56X3 macro definitions
*/

/**
 * @brief Macro that initializes `i2c_mmc56x3_config_t` to default configuration settings.
 */
#define I2C_MMC56X3_CONFIG_DEFAULT {                            \
        .dev_config.device_address     = I2C_MMC56X3_ADDR,      \
        .data_rate                     = 0,                  \
        .continuous_enabled            = false }

/*
 * MMC56X3 enumerator and sructure declerations
*/


typedef enum {
    I2C_MMC56X3_MEAS_TIME_6_6MS = (0b00),
    I2C_MMC56X3_MEAS_TIME_3_5MS = (0b01),
    I2C_MMC56X3_MEAS_TIME_2MS   = (0b10),
    I2C_MMC56X3_MEAS_TIME_1_2MS = (0b11),
} i2c_mmc56x3_measurement_times_t;

typedef enum {
    I2C_MMC56X3_MEAS_SAMPLE_1 = (0b000),
    I2C_MMC56X3_MEAS_SAMPLE_2 = (0b001),
    I2C_MMC56X3_MEAS_SAMPLE_3 = (0b010),
    I2C_MMC56X3_MEAS_SAMPLE_4 = (0b000),
} i2c_mmc56x3_measurement_samples_t;

/**
 * @brief MMC56X3 status 1 register structure.
 */
typedef union __attribute__((packed)) {
    struct STS_REG_BITS_TAG {
        uint8_t             reserved:4;       /*!< reserved and set to 0                                    (bit:0-3) */
        bool                otp_read_done:1;  /*!< true when otp memory read                                (bit:4)   */
        bool                selftest:1;       /*!< false when selftest pass                                 (bit:5)   */
        bool                data_ready_m:1;   /*!< true when magnetic measurement done and data is ready, reset when any magnetic data register is read (bit:6)   */
        bool                data_ready_t:1;   /*!< true when temperature measurement done and data is ready, reset when temperature data register is read (bit:7)   */
    } bits;            /*!< represents the 8-bit status 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status 1 register as `uint8_t` and has a reset-value of 0x00.   */
} i2c_mmc56x3_status_register_t;

/**
 * @brief MMC56X3 control 0 register structure.
 */
typedef union __attribute__((packed)) {
    struct CTRL0_REG_BITS_TAG {
        bool                sample_m:1;     /*!< perform magnetic measurement when true and self clears at the end of the measurement  (bit:0) */
        bool                sample_t:1;     /*!< perform temperature measurement when true and self clears at the end of the measurement  (bit:1) */
        uint8_t             reserved:1;     /*!< reserved and set to 0                                  (bit:2) */
        bool                do_set:1;       /*!< perform set operation when true and self clears at the end of set operation (bit:3) */
        bool                do_reset:1;     /*!< perform reset operation when true and self clears at the end of reset operation (bit:4) */
        bool                auto_sr_enabled:1;   /*!< enable automatic periodic set/reset when true  (bit:5) */
        bool                auto_st_enabled:1;   /*!< enable automatic selftest when true and self clears at the end of the operation, set registers 0x1e, 0x1f, and 0x20 beforehand (bit:6) */
        bool                continuous_freq_enabled:1;  /*!< enable measurement period calculation when true and self clears once measurement period is calculated (bit:7) */
    } bits;            /*!< represents the 8-bit control 0 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 0 register as `uint8_t` and has a reset-value of 0x00.   */
} i2c_mmc56x3_control0_register_t;

/**
 * @brief MMC56X3 control 1 register structure.
 */
typedef union __attribute__((packed)) {
    struct CTRL1_REG_BITS_TAG {
        i2c_mmc56x3_measurement_times_t bandwidth:2;    /*!< measurement time                           (bit:0-1) */
        bool                            x_disabled:1;    /*!< x channel disabled true                      (bit:2) */
        bool                            y_disabled:1;    /*!< y channel disabled true                      (bit:3) */
        bool                            z_disabled:1;    /*!< z channel disabled true                      (bit:4) */
        bool                            st_enp:1;       /*!< bring selftest coild dc current when true  (bit:5) */
        bool                            st_enm:1;       /*!< same as st_enp but opposite polarity       (bit:6) */
        bool                            sw_reset:1;     /*!< causes software-reset when true            (bit:7) */
    } bits;            /*!< represents the 8-bit control 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 1 register as `uint8_t` and has a reset-value of 0x00.   */
} i2c_mmc56x3_control1_register_t;

/**
 * @brief MMC56X3 control 2 register structure.
 */
typedef union __attribute__((packed)) {
    struct CTRL2_REG_BITS_TAG {
        uint8_t             period_set_samples:3;     /*!< number of samples before set is executed, period set and auto set-reset must be enabled (bit:0-2) */
        bool                period_set_enabled:1;  /*!< perform periodical set when true               (bit:3) */
        bool                continuous_enabled:1;         /*!< continuous mode when true, data period (odr) must be non-zero and contineous frequency enabled beforehand (bit:4) */
        uint8_t             reserved:2;       /*!< reserved and set to 0                          (bit:5-6) */
        bool                h_power_enabled:1;        /*!< achieve 1000Hz odr when true                   (bit:7) */
    } bits;            /*!< represents the 8-bit control 2 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 2 register as `uint8_t` and has a reset-value of 0x00.   */
} i2c_mmc56x3_control2_register_t;


typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} i2c_mmc56x3_magnetic_axes_data_t;

typedef struct {
    uint8_t x_axis;
    uint8_t y_axis;
    uint8_t z_axis;
} i2c_mmc56x3_selftest_axes_data_t;


/**
 * @brief MMC56X3 I2C device structure definition.
 */
typedef struct i2c_mmc56x3_t i2c_mmc56x3_t;
/**
 * @brief MMC56X3 I2C device handle structure.
 */
typedef struct i2c_mmc56x3_t *i2c_mmc56x3_handle_t;

/**
 * @brief MMC56X3 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t             dev_config;             /*!< configuration for mmc56x3 device */
    uint16_t                        data_rate;              /*!< mmc56x3 device data rate, 0-255 or 1000 for 1000Hz data rate configuration */
    bool                            continuous_enabled;     /*!< mmc56x3 device measurement mode configuration */
    i2c_mmc56x3_measurement_times_t measurement_bandwidth;  /*!< mmc56x3 device measurement bandwith configuration */
    bool                            auto_sr_enabled;        /*!< mmc56x3 auto set-reset configuration */

} i2c_mmc56x3_config_t;

/**
 * @brief MMC56X3 I2C device parameters structure.
 */
typedef struct {
    uint8_t                         product_id;             /*!< mmc56x3 product identifier */
    // place-holders from baseline configuration
    uint16_t                        data_rate;              /*!< mmc56x3 device data rate, 0-255 or 1000 for 1000Hz data rate configuration */
    bool                            continuous_enabled;     /*!< mmc56x3 device measurement mode configuration */
    i2c_mmc56x3_measurement_times_t measurement_bandwidth;  /*!< mmc56x3 device measurement bandwith configuration */
    bool                            auto_sr_enabled;        /*!< mmc56x3 auto set-reset configuration */
    bool                            x_axis_disabled;        /*!< mmc56x3 x-axis channel state configuration  */
    bool                            y_axis_disabled;        /*!< mmc56x3 y-axis channel state configuration  */
    bool                            z_axis_disabled;        /*!< mmc56x3 z-axis channel state configuration  */
    // registers 
    i2c_mmc56x3_status_register_t   status_reg;             /*!< mmc56x3 device status register cache, register is read-only */
    i2c_mmc56x3_control0_register_t control0_reg;           /*!< do not use in your app, register is write-only */
    i2c_mmc56x3_control1_register_t control1_reg;           /*!< do not use in your app, register is write-only */
    i2c_mmc56x3_control2_register_t control2_reg;           /*!< do not use in your app, register is write-only */
} i2c_mmc56x3_params_t;

/**
 * @brief MMC56X3 I2C device structure.
 */
struct i2c_mmc56x3_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_mmc56x3_params_t     *dev_params;     /*!< mmc56x3 device configuration parameters */
};

// review register - write only, causing problem when cached in driver handle

esp_err_t i2c_mmc56x3_get_status_register(i2c_mmc56x3_handle_t mmc56x3_handle);

esp_err_t i2c_mmc56x3_set_control0_register(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_control0_register_t control0_reg);

esp_err_t i2c_mmc56x3_set_control1_register(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_control1_register_t control1_reg);

esp_err_t i2c_mmc56x3_set_control2_register(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_control2_register_t control2_reg);

esp_err_t i2c_mmc56x3_get_product_id(i2c_mmc56x3_handle_t mmc56x3_handle);

esp_err_t i2c_mmc56x3_set_mode(i2c_mmc56x3_handle_t mmc56x3_handle, bool continuous);

esp_err_t i2c_mmc56x3_set_data_rate(i2c_mmc56x3_handle_t mmc56x3_handle, uint16_t rate);

/**
 * @brief Writes axes configuration to MMC56X3 to enable or disable axes (x, y, z), axes are enabled by default.
 * 
 * @param mmc56x3_handle MMC56X3 device handle.
 * @param x_axis_disabled X-axis is enabled by default, set to true to disable x-axis.
 * @param y_axis_disabled Y-axis is enabled by default, set to true to disable y-axis.
 * @param z_axis_disabled Y-axis is enabled by default, set to true to disable z-axis.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mmc56x3_set_magnetic_axes(i2c_mmc56x3_handle_t mmc56x3_handle, bool x_axis_disabled, bool y_axis_disabled, bool z_axis_disabled);

esp_err_t i2c_mmc56x3_set_selftest_thresholds(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_selftest_axes_data_t threshold_axes_data);

esp_err_t i2c_mmc56x3_get_selftest_set_values(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_selftest_axes_data_t *set_value_axes_data);

esp_err_t i2c_mmc56x3_set_selftest_set_values(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_selftest_axes_data_t set_value_axes_data);

/**
 * @brief  Pulse large currents through the sense coils to clear any offset.
 * 
 * @param[in] mmc56x3_handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mmc56x3_magnetic_set_reset(i2c_mmc56x3_handle_t mmc56x3_handle);


/**
 * @brief Initializes an MMC56X3 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] mmc56x3_config configuration of MMC56X3 device.
 * @param[out] mmc56x3_handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mmc56x3_init(i2c_master_bus_handle_t bus_handle, const i2c_mmc56x3_config_t *mmc56x3_config, i2c_mmc56x3_handle_t *mmc56x3_handle);

esp_err_t i2c_mmc56x3_get_temperature(i2c_mmc56x3_handle_t mmc56x3_handle, float *temperature);

esp_err_t i2c_mmc56x3_get_magnetic_axes(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_magnetic_axes_data_t *magnetic_axes_data);

esp_err_t i2c_mmc56x3_get_temperature_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *ready);

esp_err_t i2c_mmc56x3_get_magnetic_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *ready);

esp_err_t i2c_mmc56x3_get_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *magnetic_ready, bool *temperature_ready);

esp_err_t i2c_mmc56x3_reset(i2c_mmc56x3_handle_t mmc56x3_handle);

/**
 * @brief Removes an MMC56X3 device from master I2C bus.
 *
 * @param[in] mmc56x3_handle MMC56x3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mmc56x3_rm(i2c_mmc56x3_handle_t mmc56x3_handle);

float i2c_mmc56x3_convert_to_heading(i2c_mmc56x3_magnetic_axes_data_t magnetic_axes_data);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MMC56X3_H__
