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
 * @file veml7700.h
 * @defgroup drivers veml7700
 * @{
 *
 * ESP-IDF driver for veml7700 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __VEML7700_H__
#define __VEML7700_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VEML7700 definitions
*/
#define I2C_VEML7700_SCL_SPEED_HZ           UINT32_C(100000)    //!< veml7700 I2C default clock frequency (100KHz)

#define I2C_VEML7700_DEV_ADDR               UINT8_C(0x10)       //!< veml7700 I2C address


/*
 * VEML7700 macro definitions
*/

/**
 * @brief VEML7700 device configuration initialization default.
 */
#define I2C_VEML7700_CONFIG_DEFAULT {                                               \
            .dev_config.device_address  = I2C_VEML7700_DEV_ADDR,                    \
            .dev_config.scl_speed_hz    = I2C_VEML7700_SCL_SPEED_HZ,                \
            .gain                       = I2C_VEML7700_GAIN_DIV_4,                  \
            .integration_time           = I2C_VEML7700_INTEGRATION_TIME_100MS,      \
            .persistence_protect        = I2C_VEML7700_PERSISTENCE_PROTECTION_4,    \
            .hi_threshold               = 0,                                        \
            .lo_threshold               = 0,                                        \
            .irq_enabled                = true,                                     \
            .shutdown                   = false,                                    \
            .power_saving_enabled       = false,                                    \
            .power_saving_mode          = I2C_VEML7700_POWER_SAVING_MODE_500MS }

/*
 * VEML7700 enumerator and sructure declerations
*/

/**
 * @brief VEML7700 gains enumerator.
 */
typedef enum {
    I2C_VEML7700_GAIN_1     = (0b00),
    I2C_VEML7700_GAIN_2     = (0b01),
    I2C_VEML7700_GAIN_DIV_8 = (0b10),
    I2C_VEML7700_GAIN_DIV_4 = (0b11),
} i2c_veml7700_gains_t;

/**
 * @brief VEML7700 integration times enumerator.
 */
typedef enum {
    I2C_VEML7700_INTEGRATION_TIME_25MS  = (0b1100),
    I2C_VEML7700_INTEGRATION_TIME_50MS  = (0b1000),
    I2C_VEML7700_INTEGRATION_TIME_100MS = (0b0000),
    I2C_VEML7700_INTEGRATION_TIME_200MS = (0b0001),
    I2C_VEML7700_INTEGRATION_TIME_400MS = (0b0010),
    I2C_VEML7700_INTEGRATION_TIME_800MS = (0b0011),
} i2c_veml7700_integration_times_t;

/**
 * @brief VEML7700 persistence protections enumerator.
 */
typedef enum {
    I2C_VEML7700_PERSISTENCE_PROTECTION_1 = (0b00),
    I2C_VEML7700_PERSISTENCE_PROTECTION_2 = (0b01),
    I2C_VEML7700_PERSISTENCE_PROTECTION_4 = (0b10),
    I2C_VEML7700_PERSISTENCE_PROTECTION_8 = (0b11),
} i2c_veml7700_persistence_protections_t;

/**
 * @brief VEML7700 power saving modes enumerator.
 */
typedef enum {
    I2C_VEML7700_POWER_SAVING_MODE_500MS  = (0b00),
    I2C_VEML7700_POWER_SAVING_MODE_1000MS = (0b01),
    I2C_VEML7700_POWER_SAVING_MODE_2000MS = (0b10),
    I2C_VEML7700_POWER_SAVING_MODE_4000MS = (0b11),
} i2c_veml7700_power_saving_modes_t;

/**
 * @brief VEML7700 configuration register structure.
 * 
 *
 * Table 1 - Configuration Register #0 (see datasheet pg. 7)
 * REGISTER NAME    BIT
 *  Reserved       15:13
 *  ALS_GAIN       12:11
 *  Reserved        10
 *  ALS_IT          9:6
 *  ALS_PERS        5:4
 *  Reserved        3:2
 *  ALS_INT_END     1
 *  ALS_SD          0
**/
typedef union __attribute__((packed)) {
    struct CFG_REG_BITS_TAG {
        bool                                    shutdown:1;             /*!< als shut-down when true                    (bit:0)     */
        bool                                    irq_enabled:1;          /*!< als interrupt enable when true             (bit:1)     */
        uint8_t                                 reserved1:2;            /*!< reserved and set to 0                      (bit:2-3)   */
        i2c_veml7700_persistence_protections_t  persistence_protect:2;  /*!< sample count before the interrupt triggers (bit:4-5)   */
        i2c_veml7700_integration_times_t        integration_time:4;     /*!< time to measure                            (bit:6-9)   */
        uint8_t                                 reserved2:1;            /*!< reserved and set to 0                      (bit:10)    */
        i2c_veml7700_gains_t                    gain:2;                 /*!< control the sensitivity                    (bit:11-12) */
        uint8_t                                 reserved3:3;            /*!< reserved and set to 0                      (bit:13-15) */
    } bits;
    uint16_t reg;
} i2c_veml7700_configuration_register_t;

/**
 * @brief VEML7700 power saving mode register structure.
 * 
 *
 * Table 4 - Power Saving Modes (see datasheet pg. 8)
 * REGISTER NAME    BIT
 *  Reserved       15:3
 *  PSM             2:1
 *  PSM_EN          0
**/
typedef union __attribute__((packed)) {
    struct PSM_REG_BITS_TAG {
        bool                                    power_saving_enabled:1; /*!< power saving enabeld when true     (bit:0)    */
        i2c_veml7700_power_saving_modes_t       power_saving_mode:2;    /*!< power saving mode                  (bit:1-2)  */
        uint16_t                                reserved:13;            /*!< reserved and set to 0              (bit:3-15) */
    } bits;
    uint16_t reg;
} i2c_veml7700_power_saving_mode_register_t;

/**
 * @brief VEML7700 interrupt status register structure.
 * 
 *
 * Table 7 - Interrupt Status (see datasheet pg. 9)
 * REGISTER NAME    BIT

**/
typedef union __attribute__((packed)) {
    struct ISTS_REG_BITS_TAG {
        uint16_t                                reserved:14;              /*!< reserved and set to 0                    (bit:0-13) */
        bool                                    hi_threshold_exceeded:1;  /*!< normal unless hi threshold is exceeded   (bit:14)   */
        bool                                    lo_threshold_exceeded:1;  /*!< normal unless lo threshold is exceeded   (bit:15)   */
    } bits;
    uint16_t reg;
} i2c_veml7700_interrupt_status_register_t;

/**
 * @brief VEML7700 identifier register structure.
 */
typedef union __attribute__((packed)) {
    struct ID_REG_BITS_TAG {
        uint8_t                                device_id_code:8;       /*!< device id code (fixed 0x81)  (bit:0-7)    */
        uint8_t                                slave_option_code:8;    /*!< slave address option code    (bit:8-15)   */
    } bits;
    uint16_t reg;
} i2c_veml7700_identifier_register_t;



/**
 * @brief VEML7700 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t                     dev_config;             /*!< I2C configuration for veml7700 device */
    i2c_veml7700_gains_t                    gain;                   /*!< gain sensitivity */
    i2c_veml7700_integration_times_t        integration_time;       /*!< integration time to measure */
    i2c_veml7700_persistence_protections_t  persistence_protect;    /*!< persistence protection */
    bool                                    irq_enabled;            /*!< interrupt enabled when true */
    bool                                    shutdown;               /*!< powered down (off) when true */
    uint16_t                                hi_threshold;           /*!< high threshold register for the interrupt */
    uint16_t                                lo_threshold;           /*!< low threshold register for the interrupt */
    bool                                    power_saving_enabled;   /*!< power saving enabled when true */
    i2c_veml7700_power_saving_modes_t       power_saving_mode;      /*!< power mode register */
} i2c_veml7700_config_t;

/**
 * @brief VEML7700 I2C device handle structure.
 */
struct i2c_veml7700_t {
    i2c_master_dev_handle_t                     i2c_dev_handle;  /*!< I2C device handle */
    bool                                        sleeping;               /*!< sleeping when true */
    i2c_veml7700_configuration_register_t       config_reg;             /*!< configuration register */
    uint16_t                                    hi_threshold_reg;       /*!< high threshold register for the interrupt */
    uint16_t                                    lo_threshold_reg;       /*!< low threshold register for the interrupt */
    i2c_veml7700_power_saving_mode_register_t   power_saving_mode_reg;  /*!< power saving mode register */
    i2c_veml7700_interrupt_status_register_t    interrupt_status_reg;   /*!< interrupt status register */
    i2c_veml7700_identifier_register_t          identifier_reg;         /*!< identifier register */
    float                                       resolution;			    /*!< Current resolution and multiplier */
    uint32_t                                    maximum_lux;		    /*!< Current maximum lux limit */
};

/**
 * @brief VEML7700 IC2 handle structure definitions.
 */
typedef struct i2c_veml7700_t i2c_veml7700_t;
typedef struct i2c_veml7700_t *i2c_veml7700_handle_t;

/**
 * @brief Reads configuration register from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_configuration_register(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Writes configuration register to VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[in] config_reg VEML7700 configuration register.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_set_configuration_register(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_configuration_register_t config_reg);

/**
 * @brief Reads high and low threshold registers from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_threshold_registers(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Writes high and low threshold registers to VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[in] hi_threshold VEML7700 high threshold register.
 * @param[in] lo_threshold VEML7700 lo threshold register.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_set_threshold_registers(i2c_veml7700_handle_t veml7700_handle, const uint16_t hi_threshold, const uint16_t lo_threshold);

/**
 * @brief Reads power saving mode register from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_power_saving_mode_register(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Writes power saving mode register to VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[in] power_saving_mode_reg VEML7700 power saving mode register.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_set_power_saving_mode_register(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_power_saving_mode_register_t power_saving_mode_reg);

/**
 * @brief Reads interrupt status register from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_interrupt_status_register(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Reads identifier register from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_identifier_register(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Initializes an VEML7700 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] veml7700_config configuration of VEML7700 device.
 * @param[out] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_init(i2c_master_bus_handle_t bus_handle, const i2c_veml7700_config_t *veml7700_config, i2c_veml7700_handle_t *veml7700_handle);

/**
 * @brief Reads ambient light from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[out] lux ambient light illumination in lux.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_ambient_light(i2c_veml7700_handle_t veml7700_handle, float *lux);

/**
 * @brief Reads optimal ambient light from VEML7700.
 * 
 * @note This follows an unofficial algorithm taken from other sources.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[out] lux ambient light illumination in lux.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_ambient_light_auto(i2c_veml7700_handle_t veml7700_handle, float *lux);

/**
 * @brief Reads white channel from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[out] lux white channel illumination in lux.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_white_channel(i2c_veml7700_handle_t veml7700_handle, float *lux);

/**
 * @brief Reads optimal white channel from VEML7700.
 * 
 * @note This follows an unofficial algorithm taken from other sources.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[out] lux white channel illumination in lux.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_get_white_channel_auto(i2c_veml7700_handle_t veml7700_handle, float *lux);

/**
 * @brief Reads interrupt status from VEML7700.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @param[out] hi_threshold_exceeded true when high threshold is exceeded.
 * @param[out] lo_threshold_exceeded true when lo threshold is exceeded.
 * @return ESP_OK on success.
 */
esp_err_t veml7700_get_interrupt_status(i2c_veml7700_handle_t veml7700_handle, bool *hi_threshold_exceeded, bool *lo_threshold_exceeded);

/**
 * @brief Shuts down VEML7700 until woken.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_shutdown(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Wakes up VEML7700 from shut-down.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_wakeup(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Removes an VEML7700 device from master bus.
 *
 * @param[in] veml7700_handle VEML7700 device handle.
 * @return ESP_OK on success.
 */
esp_err_t i2c_veml7700_rm(i2c_veml7700_handle_t veml7700_handle);

/**
 * @brief Removes an VEML7700 device from master I2C bus and delete the handle.
 * 
 * @param veml7700_handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_veml7700_del(i2c_veml7700_handle_t veml7700_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VEML7700_H__