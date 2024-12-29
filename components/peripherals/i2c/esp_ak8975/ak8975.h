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
 * @file ak8975.h
 * @defgroup drivers ak8975
 * @{
 *
 * ESP-IDF driver for ak8975 3-axis electronic compass
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AK8975_H__
#define __AK8975_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AK8975 definitions
*/
#define I2C_AK8975_SCL_SPEED_HZ             UINT32_C(100000)//!< ak8975 I2C default clock frequency (100KHz)

#define I2C_AK8975_DEV_ADDR_CAD1_0_CAD0_0   UINT8_C(0x0c)   //!< ak8975 I2C address when CAD1 and CAD0 are low
#define I2C_AK8975_DEV_ADDR_CAD1_0_CAD0_1   UINT8_C(0x0d)   //!< ak8975 I2C address when CAD1 is low and CAD0 is high
#define I2C_AK8975_DEV_ADDR_CAD1_1_CAD0_0   UINT8_C(0x0e)   //!< ak8975 I2C address when CAD1 is high and CAD0 is low
#define I2C_AK8975_DEV_ADDR_CAD1_1_CAD0_1   UINT8_C(0x0f)   //!< ak8975 I2C address when CAD1 and CAD0 are high


/*
 * AK8975 macro definitions
*/

/**
 * @brief Macro that initializes `i2c_ak8975_config_t` to default configuration settings.
 */
#define I2C_AK8975_CONFIG_DEFAULT {                                 \
    .dev_config.scl_speed_hz   = I2C_AK8975_SCL_SPEED_HZ,           \
    .dev_config.device_address = I2C_AK8975_DEV_ADDR_CAD1_0_CAD0_0, }

/*
 * AK8975 enumerator and sructure declerations
*/

/**
 * @brief AK8975 operating modes enumerator.
 */
typedef enum {
    I2C_AK8975_OPMODE_POWER_DOWN    = (0b0000), /*!< power-down mode */
    I2C_AK8975_OPMODE_SINGLE_MEAS   = (0b0001), /*!< single measurement mode */
    I2C_AK8975_OPMODE_SELF_TEST     = (0b1000), /*!< self-test mode */
    I2C_AK8975_OPMODE_FUSE_ROM      = (0b1111)  /*!< fuse rom access mode */
} i2c_ak8975_operating_modes_t;

/**
 * @brief AK8975 control register structure.
 */
typedef union __attribute__((packed)) {
    struct CTRL_REG_BITS_TAG {
        i2c_ak8975_operating_modes_t    mode:4;       /*!< operating mode         (bit:0-3)   */
        uint8_t                         reserved:4;   /*!< reserved and set to 0  (bit:4-7)   */
    } bits;            /*!< represents the 8-bit control register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control register as `uint8_t`.   */
} i2c_ak8975_control_register_t;

/**
 * @brief AK8975 status 1 register structure.
 */
typedef union __attribute__((packed)) {
    struct STS1_REG_BITS_TAG {
        bool                data_ready:1;  /*!< data ready status     (bit:0)   */
        uint8_t             reserved:7;    /*!< reserved and set to 0 (bit:1-7)   */
    } bits;            /*!< represents the 8-bit status 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status 1 register as `uint8_t`.   */
} i2c_ak8975_status1_register_t;

/**
 * @brief AK8975 status 2 register structure.
 */
typedef union __attribute__((packed)) {
    struct STS2_REG_BITS_TAG {
        uint8_t             reserved1:2;       /*!< reserved and set to 0           (bit:0-1)   */
        bool                data_error:1;      /*!< data error status               (bit:2)   */
        bool                sensor_overflow:1; /*!< magnetic sensor overflow status (bit:3)   */
        uint8_t             reserved2:4;       /*!< reserved and set to 0           (bit:4-7)   */
    } bits;            /*!< represents the 8-bit status 2 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status 2 register as `uint8_t`.   */
} i2c_ak8975_status2_register_t;

/**
 * @brief AK8975 self-test control register structure.
 */
typedef union __attribute__((packed)) {
    struct STCTRL_REG_BITS_TAG {
        uint8_t             reserved1:6;   /*!< reserved and set to 0 (bit:0-5)   */
        bool                selftest_state:1; /*!< self-test status      (bit:6)   */
        uint8_t             reserved2:1;   /*!< reserved and set to 0 (bit:7)   */
    } bits;            /*!< represents the 8-bit self-test control register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit self-test control register as `uint8_t`.   */
} i2c_ak8975_selftest_control_register_t;

/**
 * @brief AK8975 processed compass axes data structure and axes have a range +/-1229 uT.
 * 
 * @note AK8975 data overflow of axes (x, y, z) is a sum of the absolute values for each axis
 *      and the sum of the axes should be smaller than 2400 uT (|X|+|Y|+|Z| < 2400 uT).
 * 
 */
typedef struct {
    float x_axis;
    float y_axis;
    float z_axis;
} i2c_ak8975_magnetic_axes_data_t;

/**
 * @brief AK8975 axes data registers (HX, HZ, HZ) structure.
 */
typedef struct {
    i2c_bytes_to_int16_t x_axis;
    i2c_bytes_to_int16_t y_axis;
    i2c_bytes_to_int16_t z_axis;
} i2c_ak8975_axes_data_t;

/**
 * @brief AK8975 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t             dev_config;       /*!< I2C configuration for ak8975 device */
} i2c_ak8975_config_t;

/**
 * @brief AK8975 I2C device structure.
 */
struct i2c_ak8975_t {
    i2c_master_dev_handle_t         i2c_dev_handle;  /*!< I2C master device handle */
    uint8_t                         device_id;    /*!< ak8975 device identifier */
    uint8_t                         device_info;  /*!< ak8975 device information */
    i2c_ak8975_control_register_t   control_reg;  /*!< ak8975 device control register */
    i2c_ak8975_selftest_control_register_t selftest_reg;  /*!< ak8975 device self-test control register */
    i2c_ak8975_status1_register_t   status1_reg;  /*!< ak8975 device status 1 register */
    i2c_ak8975_status2_register_t   status2_reg;  /*!< ak8975 device status 2 register */
    uint8_t                         asa_x_value;  /*!< ak8975 x-axis sensitivity adjustment value */
    uint8_t                         asa_y_value;  /*!< ak8975 y-axis sensitivity adjustment value */
    uint8_t                         asa_z_value;  /*!< ak8975 z-axis sensitivity adjustment value */
};

/**
 * @brief AK8975 I2C device structure definition.
 */
typedef struct i2c_ak8975_t i2c_ak8975_t;
/**
 * @brief AK8975 I2C device handle definition.
 */
typedef struct i2c_ak8975_t *i2c_ak8975_handle_t;


/**
 * @brief Reads control register from AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_control_register(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Writes control register to AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @param[in] control_reg AK8975 control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_set_control_register(i2c_ak8975_handle_t ak8975_handle, const i2c_ak8975_control_register_t control_reg);

/**
 * @brief Reads self-test control register from AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @param[in] control_reg AK8975 self-test control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_selftest_control_register(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Reads status 1 register from AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_status1_register(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Reads status 2 register from AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_status2_register(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Reads ASA (X, Y, Z) registers from AK8975.
 * 
 * @param[in] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_asa_registers(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Converts heading (0-359 degrees) from magnetic axes.
 * 
 * @return float heading in degrees (0-359).
 */
float i2c_ak8975_convert_to_heading(const i2c_ak8975_magnetic_axes_data_t magnetic_axes_data);

/**
 * @brief Initializes an AK8975 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ak8975_config configuration of AK8975 device.
 * @param[out] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_init(i2c_master_bus_handle_t bus_handle, const i2c_ak8975_config_t *ak8975_config, i2c_ak8975_handle_t *ak8975_handle);

/**
 * @brief Read magnetic measurement from AK8975.
 *
 * @param[in] ak8975_handle AK8975 device handle.
 * @param[out] magnetic_axes AK8975 magnetic axes data (X, Y, Z) with sensitivity adjustments applied.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_get_magnetic_axes(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_magnetic_axes_data_t *const magnetic_axes_data);

/**
 * @brief Self-test judgement of AK8975 to check if sensor is working normally.
 * 
 * @note Axes data should be witin the following ranges: x-axis (-100<=X<=+100), y-axis (-100<=Y<=+100), z-axis (-1000<=Z<=-300)
 *
 * @param[in] ak8975_handle AK8975 device handle.
 * @param[out] compass_axes AK8975 compass axes data (X, Y, Z) with sensitivity adjustments applied.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_selftest(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_magnetic_axes_data_t *const magnetic_axes_data);

esp_err_t i2c_ak8975_get_data_status(i2c_ak8975_handle_t ak8975_handle, bool *const ready);

esp_err_t i2c_ak8975_get_error_status(i2c_ak8975_handle_t ak8975_handle, bool *const error);

esp_err_t i2c_ak8975_get_overflow_status(i2c_ak8975_handle_t ak8975_handle, bool *const overflow);

esp_err_t i2c_ak8975_power_down(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Removes an AK8975 device from master I2C bus.
 *
 * @param[in] ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_remove(i2c_ak8975_handle_t ak8975_handle);

/**
 * @brief Removes an AK8975 device from master bus and frees handle.
 * 
 * @param ak8975_handle AK8975 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ak8975_delete(i2c_ak8975_handle_t ak8975_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AK8975_H__
