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
typedef struct i2c_mlx90614_t i2c_mlx90614_t;
typedef struct i2c_mlx90614_t *i2c_mlx90614_handle_t;

typedef union __attribute__((packed)) {
    struct {
        uint16_t pwm_mode: 1;
        uint16_t pwm_mode_state: 1;
        uint16_t sda_pin: 1;
        uint16_t thermal_mode: 1;
        uint16_t pwm_repetition: 5;
        uint16_t reserved: 7;
    } bit;
    uint16_t reg;
} i2c_mlx90614_pwmctrl_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint16_t iir: 3;
        uint16_t repeat: 1;
        uint16_t t_objs: 2;
        uint16_t ir_type: 1;
        uint16_t ks_sign: 1;
        uint16_t fir: 3;
        uint16_t gain: 3;
        uint16_t kt2_sign: 1;
        uint16_t test: 1;
    } bit;
    uint16_t reg;
} i2c_mlx90614_config_register_t;


typedef struct {
    uint8_t                  address;               /*!< I2C device address */
    uint32_t                 ident_number_hi;       /*!< I2C device identification number 32-bit hi */
    uint32_t                 ident_number_lo;       /*!< I2C device identification number 32-bit lo */
    float                    emissivity;            /*!< mlx90614 emissivity from 0.1 to 1.0. */
    float                    obj_max_temperature;   /*!< mlx90614 maximum object temperature in degrees Celsius. */
    float                    obj_min_temperature;   /*!< mlx90614 minimum object temperature in degrees Celsius. */
} i2c_mlx90614_params_t;


typedef struct {
    i2c_device_config_t      dev_config;            /*!< configuration for mlx90614 device */
} i2c_mlx90614_config_t;


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
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_init(i2c_master_bus_handle_t bus_handle, const i2c_mlx90614_config_t *mlx90614_config, i2c_mlx90614_handle_t *mlx90614_handle);

/**
 * @brief reads all three temperatures (ambient, object1 and object2) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] ambient_temperature ambient temperature in degrees celsius
 * @param[out] object1_temperature object1 temperature in degrees celsius
 * @param[out] object2_temperature object2 temperature in degrees celsius
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_temperatures(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature, float *object1_temperature, float *object2_temperature);

/**
 * @brief reads the ambient temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] ambient_temperature ambient temperature in degrees celsius
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_ambient_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature);

/**
 * @brief reads object1 temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] object1_temperature object1 temperature in degrees celsius
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_object1_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object1_temperature);

/**
 * @brief reads object2 temperature from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @param[out] object2_temperature object2 temperature in degrees celsius
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_object2_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object2_temperature);

/**
 * @brief reads emissivity (0.1 to 1.0) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_emissivity(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined emissivity (0.1 to 1.0) to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_set_emissivity(i2c_mlx90614_handle_t mlx90614_handle, const float emissivity);

/**
 * @brief reads object temperature ranges (maximum and minimum) from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_object_temperature_ranges(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined maximum object temperature range to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_set_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature);

/**
 * @brief writes user defined minimum object temperature range to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_set_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature);

/**
 * @brief reads the I2C address from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_get_address(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief writes user defined I2C address to the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_set_address(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t address);

/**
 * @brief puts the mlx90614 to sleep.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_sleep(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief wakes up the mlx90614 from sleep.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_wake(i2c_mlx90614_handle_t mlx90614_handle);

/**
 * @brief removes an mlx90614 device from master bus.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
esp_err_t i2c_mlx90614_rm(i2c_mlx90614_handle_t mlx90614_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MLX90614_H__
