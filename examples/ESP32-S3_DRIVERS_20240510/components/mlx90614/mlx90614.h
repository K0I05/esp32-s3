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
 * @file mlx90614.h
 * @defgroup drivers mlx90614
 * @{
 *
 * ESP-IDF driver for mlx90614 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MLX90614_H__
#define __MLX90614_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MLX90614 definitions
*/
#define I2C_MLX90614_CRC8_POLYNOM            UINT8_C(7)        //!< mlx90614 I2C CRC8 polynomial
#define I2C_MLX90614_DATA_RATE_HZ            (100000)          //!< mlx90614 I2C default clock frequency (100KHz)
#define I2C_MLX90614_ADDR                    UINT8_C(0x5A)     //!< mlx90614 I2C address when ADDR pin floating/low

#define I2C_MLX90614_CMD_RAM_READ_RAWIR1     UINT8_C(0x04)
#define I2C_MLX90614_CMD_RAM_READ_RAWIR2     UINT8_C(0x05)
#define I2C_MLX90614_CMD_RAM_READ_TA         UINT8_C(0x06)
#define I2C_MLX90614_CMD_RAM_READ_TOBJ1      UINT8_C(0x07)
#define I2C_MLX90614_CMD_RAM_READ_TOBJ2      UINT8_C(0x08)
#define I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX   UINT8_C(0x20)
#define I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN   UINT8_C(0x21)
#define I2C_MLX90614_CMD_EEPROM_RDWR_PWMCTRL UINT8_C(0x22)
#define I2C_MLX90614_CMD_EEPROM_RDWR_TAR     UINT8_C(0x23)
#define I2C_MLX90614_CMD_EEPROM_RDWR_EMISS   UINT8_C(0x24)
#define I2C_MLX90614_CMD_EEPROM_RDWR_CFGREG1 UINT8_C(0x25)
#define I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR UINT8_C(0x2E)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM1    UINT8_C(0x3C)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM2    UINT8_C(0x3D)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM3    UINT8_C(0x3E)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM4    UINT8_C(0x3F)        
#define I2C_MLX90614_CMD_READ_FLAGS_REG      UINT8_C(0xF0)
#define I2C_MLX90614_CMD_SLEEP               UINT8_C(0xFF)
#define I2C_MLX90614_CMD_EEPROM_CLR_CELL     UINT8_C(0x00)

/*
 * macro definitions
*/
#define I2C_MLX90614_CONFIG_DEFAULT {                               \
        .dev_config.device_address     = I2C_MLX90614_ADDR, }

/*
 * SHT4X enumerator and sructure declerations
*/

typedef enum {
    I2C_MLX90614_SENSOR_IIR_100 = (0b100),
    I2C_MLX90614_SENSOR_IIR_80 = (0b101),
    I2C_MLX90614_SENSOR_IIR_67 = (0b110),
    I2C_MLX90614_SENSOR_IIR_57 = (0b111),
    I2C_MLX90614_SENSOR_IIR_50 = (0b000),
    I2C_MLX90614_SENSOR_IIR_25 = (0b001),
    I2C_MLX90614_SENSOR_IIR_17 = (0b010),
    I2C_MLX90614_SENSOR_IIR_13 = (0b011)
} i2c_mlx90614_sensor_iirs_t;

typedef enum {
    I2C_MLX90614_SENSOR_TEST_REPEAT_OFF = 0,
    I2C_MLX90614_SENSOR_TEST_REPEAT_ON  = 1
} i2c_mlx90614_sensor_test_repeat_states_t;

typedef enum {
    I2C_MLX90614_TEMPERATURE_SENSOR_TA_TOBJ1    = (0b00),
    I2C_MLX90614_TEMPERATURE_SENSOR_TA_TOBJ2    = (0b01),
    I2C_MLX90614_TEMPERATURE_SENSOR_TOBJ2       = (0b10),
    I2C_MLX90614_TEMPERATURE_SENSOR_TOBJ1_TOBJ2 = (0b11)
} i2c_mlx90614_temperature_sensors_t;

typedef enum {
    I2C_MLX90614_SENSOR_IR_TYPE_SINGLE = 0,
    I2C_MLX90614_SENSOR_IR_TYPE_DUAL   = 1
} i2c_mlx90614_sensor_ir_types_t;

typedef enum {
    I2C_MLX90614_K_SIGN_POSITIVE = 0,
    I2C_MLX90614_K_SGIN_NEGATIVE = 1
} i2c_mlx90614_k_signs_t;

typedef enum {
    I2C_MLX90614_FIR_128    = (0b100),
    I2C_MLX90614_FIR_256    = (0b101),
    I2C_MLX90614_FIR_512    = (0b110),
    I2C_MLX90614_FIR_1024   = (0b111)
} i2c_mlx90614_fir_values_t;

typedef enum {
    I2C_MLX90614_GAIN_1     = (0b000),
    I2C_MLX90614_GAIN_3     = (0b001),
    I2C_MLX90614_GAIN_6     = (0b010),
    I2C_MLX90614_GAIN_12_5  = (0b011),
    I2C_MLX90614_GAIN_25    = (0b100),
    I2C_MLX90614_GAIN_50    = (0b101),
    I2C_MLX90614_GAIN_100A  = (0b110),
    I2C_MLX90614_GAIN_100B  = (0b111)
} i2c_ml90614_gains_t;

typedef enum {
    I2C_MLX90614_KT2_SIGN_POSITIVE = 0,
    I2C_MLX90614_KT2_SGIN_NEGATIVE = 1
} i2c_mlx90614_nk2_signs_t;

typedef enum {
    I2C_MLX90614_SENSOR_TEST_ENABLED  = 0,
    I2C_MLX90614_SENSOR_TEST_DISABLED = 1
} i2c_mlx90614_sensor_test_states_t;

/**
 * @brief MLX90614 configuration register structure.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_mlx90614_sensor_iirs_t                  iir:3;                  /*!< iir setting 						(bit:0-2) */
        i2c_mlx90614_sensor_test_repeat_states_t    test_repeat_state:1;    /*!< repeat sensor test state 			(bit:3) */
        i2c_mlx90614_temperature_sensors_t          t_sensors:2;            /*!< temperature sensor configuration 	(bit:4-5) */
        i2c_mlx90614_sensor_ir_types_t              ir_type:1;              /*!< ir sensor type 					(bit:6) */
        i2c_mlx90614_k_signs_t                      k_sign:1;               /*!< positie or negative signs of k 	(bit:7) */
        i2c_mlx90614_fir_values_t                   fir:3;                  /*!< fir setting 						(bit:8-10) */
        i2c_ml90614_gains_t                         gain:3;                 /*!< gain setting 						(bit:11-13) */
        i2c_mlx90614_nk2_signs_t                    kt2_sign:1;             /*!< positie or negative signs of kt2 	(bit:14) */
        i2c_mlx90614_sensor_test_states_t           test_state:1;           /*!< sensor test state 					(bit:15) */
    } bit;                          /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;                   /*!< represents the 16-bit control register as `uint16_t` */
} i2c_mlx90614_config_register_t;

typedef enum {
    I2C_MLX90614_PWM_MODE_EXTENDED = 0,
    I2C_MLX90614_PWM_MODE_SINGLE   = 1
} i2c_mlx90614_pwm_modes_t;

typedef enum {
    I2C_MLX90614_PWM_MODE_STATE_DISABLED = 0,
    I2C_MLX90614_PWM_MODE_STATE_ENABLED  = 1
} i2c_mlx90614_pwm_mode_states_t;

typedef enum {
    I2C_MLX90614_SDA_PIN_MODE_OPEN_DRAIN  = 0,
    I2C_MLX90614_SDA_PIN_MODE_PUSH_PULL  = 1
} i2c_mlx90614_sda_pin_modes_t;

typedef enum {
    I2C_MLX90614_THERMAL_MODE_PWM           = 0,
    I2C_MLX90614_THERMAL_MODE_THERMAL_RELAY = 1
} i2c_mlx90614_thermal_modes;

/**
 * @brief MLX90614 PWM control register structure.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_mlx90614_pwm_modes_t        pwm_mode:1;         /*!< PWM mode       (bit:0) */ 
        i2c_mlx90614_pwm_mode_states_t  pwm_mode_state:1;   /*!< PWM mode state (bit:1) */
        i2c_mlx90614_sda_pin_modes_t    sda_pin_mode:1;     /*!< SDA pin mode   (bit:2) */
        i2c_mlx90614_thermal_modes      thermal_mode:1;     /*!< thermal mode   (bit:3) */
        uint16_t                        pwm_repetition:5;   /*!< PWM repetition number 0...62 step of 2 (bit:4-8)*/
        uint16_t                        pwm_period_mult:7;  /*!< PWM period in ms is 1.024*bits (single PWM mode) or 2.048*bits (extended PWM mode), bits is the multiplier (bit:9-15) */
    } bit;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
    float    pwm_period;    /*!< PWM period in ms is calculated from `pwm_period_mult` and `pwm_mode`. */
} i2c_mlx90614_pwmctrl_register_t;

/**
 * @brief MLX90614 device parameters structure.
 */
typedef struct {
    uint8_t                  address;               /*!< I2C device address */
    uint32_t                 ident_number_hi;       /*!< I2C device identification number 32-bit hi */
    uint32_t                 ident_number_lo;       /*!< I2C device identification number 32-bit lo */
    float                    emissivity;            /*!< mlx90614 emissivity from 0.1 to 1.0. */
    float                    obj_max_temperature;   /*!< mlx90614 maximum object temperature in degrees Celsius. */
    float                    obj_min_temperature;   /*!< mlx90614 minimum object temperature in degrees Celsius. */
    i2c_mlx90614_config_register_t  config_reg;     /*!< mlx90614 `ConfigRegister1` consits of control bits for configuring the analog and digital parts of the device. */
    i2c_mlx90614_pwmctrl_register_t pwmctrl_reg;    /*!< mlx90614 `PWMCTRL` consists of control bits for configuring the PWM/SDA pin on the device. */
} i2c_mlx90614_params_t;

/**
 * @brief MLX90614 I2C device structure definition.
 */
typedef struct i2c_mlx90614_t i2c_mlx90614_t;
/**
 * @brief MLX90614 I2C device handle structure.
 */
typedef struct i2c_mlx90614_t *i2c_mlx90614_handle_t;

/**
 * @brief MLX90614 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t      dev_config;            /*!< configuration for mlx90614 device */
} i2c_mlx90614_config_t;

/**
 * @brief MLX90614 I2C device structure.
 */
struct i2c_mlx90614_t {
    i2c_master_dev_handle_t  i2c_dev_handle;        /*!< I2C device handle */
    i2c_mlx90614_params_t    *dev_params;           /*!< mlx90614 device params */
};

/**
 * @brief initializes an mlx90614 device onto the master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] mlx90614_config configuration of mlx90614 device
 * @param[out] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_init(i2c_master_bus_handle_t bus_handle, const i2c_mlx90614_config_t *mlx90614_config, i2c_mlx90614_handle_t *mlx90614_handle);

/**
 * @brief reads all three temperatures (ambient, object1 and object2) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] ambient_temperature ambient temperature in degrees celsius
 * @param[out] object1_temperature object1 temperature in degrees celsius
 * @param[out] object2_temperature object2 temperature in degrees celsius
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_temperatures(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature, float *object1_temperature, float *object2_temperature);

/**
 * @brief reads the ambient temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] ambient_temperature ambient temperature in degrees celsius
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_ambient_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature);

/**
 * @brief reads object1 temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] object1_temperature object1 temperature in degrees celsius
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_object1_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object1_temperature);

/**
 * @brief reads object2 temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] object2_temperature object2 temperature in degrees celsius
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_object2_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object2_temperature);

/**
 * @brief reads emissivity (0.1 to 1.0) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_emissivity(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined emissivity (0.1 to 1.0) to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_emissivity(i2c_mlx90614_handle_t mlx90614_handle, const float emissivity);

/**
 * @brief reads object temperature ranges (maximum and minimum) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_object_temperature_ranges(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined maximum object temperature range to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature);

/**
 * @brief writes user defined minimum object temperature range to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature);

/**
 * @brief reads the I2C address from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_address(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined I2C address to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_address(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t address);

/**
 * @brief puts the mlx90614 to sleep.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_sleep(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief wakes-up the mlx90614 from sleep.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_wake(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief reads configuration register from mlx90614.
 * 
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_config_register(i2c_mlx90614_handle_t mlx90614_handle);
/**
 * @brief writes configuration register to mlx90614.
 * 
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[in] config_reg mlx90614 configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_config_register(i2c_mlx90614_handle_t mlx90614_handle, i2c_mlx90614_config_register_t config_reg);
/**
 * @brief reads PWM control register from mlx90614.
 * 
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_get_pwmctrl_register(i2c_mlx90614_handle_t mlx90614_handle);
/**
 * @brief writes PWM control register to mlx90614.
 * 
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[in] pwmctrl_reg mlx90614 PWM control register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_set_pwmctrl_register(i2c_mlx90614_handle_t mlx90614_handle, i2c_mlx90614_pwmctrl_register_t pwmctrl_reg);

/**
 * @brief removes an mlx90614 device from master bus.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_mlx90614_rm(i2c_mlx90614_handle_t mlx90614_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MLX90614_H__
