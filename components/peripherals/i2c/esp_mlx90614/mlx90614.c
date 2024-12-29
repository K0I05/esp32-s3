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
 * @file mlx90614.c
 *
 * ESP-IDF driver for MLX90614 IR sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "mlx90614.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * MLX90614 definitions
*/

#define I2C_MLX90614_CMD_RAM_READ_AMB        UINT8_C(0x03)
#define I2C_MLX90614_CMD_RAM_READ_RAWIR1     UINT8_C(0x04)
#define I2C_MLX90614_CMD_RAM_READ_RAWIR2     UINT8_C(0x05)
#define I2C_MLX90614_CMD_RAM_READ_TA         UINT8_C(0x06)
#define I2C_MLX90614_CMD_RAM_READ_TOBJ1      UINT8_C(0x07)
#define I2C_MLX90614_CMD_RAM_READ_TOBJ2      UINT8_C(0x08)

#define I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX   UINT8_C(0x20)
#define I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN   UINT8_C(0x21)
#define I2C_MLX90614_CMD_EEPROM_RDWR_PWMCTRL UINT8_C(0x22)
#define I2C_MLX90614_CMD_EEPROM_RDWR_TARANGE UINT8_C(0x23)
#define I2C_MLX90614_CMD_EEPROM_RDWR_EMISS   UINT8_C(0x24)
#define I2C_MLX90614_CMD_EEPROM_RDWR_CFGREG  UINT8_C(0x25)
#define I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR UINT8_C(0x2E)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM1    UINT8_C(0x3C)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM2    UINT8_C(0x3D)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM3    UINT8_C(0x3E)
#define I2C_MLX90614_CMD_EEPROM_RD_IDNUM4    UINT8_C(0x3F)

#define I2C_MLX90614_CMD_READ_FLAGS_REG      UINT8_C(0xF0)
#define I2C_MLX90614_CMD_SLEEP               UINT8_C(0xFF)

#define I2C_MLX90614_CMD_EEPROM_CLR_CELL     UINT8_C(0x00)

#define I2C_MLX90614_CRC8_POLYNOM            UINT8_C(7)        //!< mlx90614 I2C CRC8 polynomial

#define I2C_MLX90614_POWERUP_DELAY_MS        UINT16_C(10)
#define I2C_MLX90614_APPSTART_DELAY_MS       UINT16_C(10)
#define I2C_MLX90614_CMD_DELAY_MS            UINT16_C(5)
#define I2C_MLX90614_EEPROM_RDWR_DELAY_MS    UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "mlx90614";

/*
* functions and subrountines
*/

/**
 * @brief Calculates mlx90614 crc8 value using a x^8+x^2+x^1+1 poly.  See datasheet for details.
 *
 * @param[in] crc Crc recieved for data to validate
 * @param[in] data Data to perform crc8 validation
 * @return uint8_t Calculated crc8 value.
 */
static inline uint8_t i2c_mlx90614_crc8(const uint8_t crc, const uint8_t data) {
    uint8_t i;
	uint8_t data_crc = crc ^ data;

	for ( i = 0; i < 8; i++ ) {
		if (( data_crc & 0x80 ) != 0 ) {
			data_crc <<= 1;
			data_crc ^= I2C_MLX90614_CRC8_POLYNOM;
		} else {
			data_crc <<= 1;
		}
	}

	return data_crc;
}

/**
 * @brief Converts milliseconds to ticks.
 *
 * @param[in] ms Milliseconds to convert to ticks.
 * @return size_t Converted ms in ticks.
 */
static inline size_t i2c_mlx90614_get_tick_duration(const uint16_t ms) {
    size_t res = pdMS_TO_TICKS(ms);

    return res == 0 ? 1 : res;
}

/**
 * @brief Decodes raw `uint16_t` temperature to floating point temperature in degrees celsius.
 *
 * @param[in] raw_data Raw `uint16_t` temperature to decode.
 * @return float Decoded floating point temperature in degrees celsius.
 */
static inline float i2c_mlx90614_decode_temperature(const uint16_t encoded_temperature) {
    float decoded_temperature = (float)encoded_temperature * 0.02f;
    decoded_temperature -= 273.15f;  // kelvin to celcius
    return decoded_temperature;
}

/**
 * @brief Encodes floating point temperature in degrees celsius to raw `uint16_t` temperature.
 *
 * @param[in] temperature Decoded floating point temperature to encode.
 * @return uint16_t Encoded raw `uint16_t` temperature.
 */
static inline uint16_t i2c_mlx90614_encode_temperature(const float decoded_temperature) {
    float encoded_temperature = decoded_temperature + 273.15f;
	encoded_temperature *= 50.0f;  // then multiply by 0.02 degK / bit
	return (uint16_t)encoded_temperature;
}

/**
 * @brief Decodes raw `uint16_t` IR data to `int16_t` IR data.
 * 
 * @param raw_data Raw `uint16_t` IR data to decode.
 * @return int16_t Decoded IR data.
 */
static inline int16_t i2c_mlx90614_decode_ir(const uint16_t raw_data) {
    int16_t ir = raw_data;
    if(raw_data > 0x7FFF) {
        ir = 0x8000 - ir;
    }
    return ir;
}

/**
 * @brief Reads a word (2-bytes) with CRC validation from MLX90614.
 * 
 * @param mlx90614_handle MLX90614 device handle.
 * @param reg_addr MLX90614 read register (1-byte).
 * @param data MLX90614 register data (2-bytes).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_mlx90614_read_word(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, uint16_t *const data) {
    i2c_uint24_t buffer;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    /* attempt i2c read transaction */
    ESP_ERROR_CHECK( i2c_master_bus_read_byte24(mlx90614_handle->i2c_dev_handle, reg_addr, &buffer) );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_CMD_DELAY_MS));

    /* compute and validate crc */
    uint8_t crc = i2c_mlx90614_crc8(0, (mlx90614_handle->i2c_dev_address << 1));

    crc = i2c_mlx90614_crc8(crc, reg_addr);
	crc = i2c_mlx90614_crc8(crc, (mlx90614_handle->i2c_dev_address << 1) + 1);
	crc = i2c_mlx90614_crc8(crc, buffer[0]); // lsb
	crc = i2c_mlx90614_crc8(crc, buffer[1]); // msb

    // validate calculated crc vs pec received
    if (crc == buffer[2]) {
        *data = buffer[0] | (buffer[1] << 8);
	} else {
		return ESP_ERR_INVALID_CRC;
	}

    return ESP_OK;
}

/**
 * @brief Writes a word (2-bytes) with CRC to MLX90614.
 * 
 * @param mlx90614_handle MLX90614 device handle.
 * @param reg_addr MLX90614 write register (1-byte).
 * @param data MLX90614 data (2-bytes) to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_mlx90614_write_word(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, const uint16_t data) {
    uint8_t tx[I2C_UINT32_SIZE] = { 0, 0, 0, 0 };
    uint8_t crc; 

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    tx[0] = reg_addr;       // register
    tx[1] = data & 0x00FF;  // lsb
    tx[2] = data >> 8;      // msb

    crc = i2c_mlx90614_crc8(0, (mlx90614_handle->i2c_dev_address << 1));
	crc = i2c_mlx90614_crc8(crc, tx[0]); // register
	crc = i2c_mlx90614_crc8(crc, tx[1]); // lsb
	crc = i2c_mlx90614_crc8(crc, tx[2]); // msb

    tx[3] = crc;            // pec

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(mlx90614_handle->i2c_dev_handle, tx, I2C_UINT32_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_mlx90614_write_word failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes a word (2-bytes) to MLX90614 EEPROM.
 * 
 * @param mlx90614_handle MLX90614 device handle.
 * @param reg_addr MLX90614 write register (1-byte).
 * @param data MLX90614 data (2-bytes) to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_mlx90614_write_eeprom(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, const uint16_t data) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    // clear eeprom register
    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, reg_addr, I2C_MLX90614_CMD_EEPROM_CLR_CELL) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_EEPROM_RDWR_DELAY_MS));

    // write data to register
    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, reg_addr, data) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_EEPROM_RDWR_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads the identification number as two 32-bit values (hi and lo) from MLX90614.
 *
 * @param[in] mlx90614_handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_mlx90614_get_ident_numbers(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t id_num[I2C_UINT32_SIZE]; // 64-bit ident value

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM1, &id_num[0]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM2, &id_num[1]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM3, &id_num[2]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM4, &id_num[3]) );

    mlx90614_handle->ident_number_hi = id_num[2] | (id_num[3] << 16);
    mlx90614_handle->ident_number_lo = id_num[0] | (id_num[1] << 16);

    return ESP_OK;
}

static inline esp_err_t i2c_mlx90614_write_command(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t command) {
    uint8_t tx[I2C_UINT16_SIZE] = { 0, 0 };
    uint8_t crc;

    /* validate arguments I2C_MLX90614_CMD_SLEEP */
    ESP_ARG_CHECK( mlx90614_handle );

    //if(command != 0x60 && command != 0x61) {
    //if(command != 0x60 && command != 0x61) {
    //    return ESP_ERR_INVALID_ARG;
    //}

    tx[0] = command;       // command

    crc = i2c_mlx90614_crc8(0, (mlx90614_handle->i2c_dev_address << 1));
	crc = i2c_mlx90614_crc8(crc, tx[0]); // command

    tx[1] = crc;            // pec

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(mlx90614_handle->i2c_dev_handle, tx, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_mlx90614_write_command failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_config_register(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t config_reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_CFGREG, &config_reg) );

    mlx90614_handle->config_reg.reg = config_reg;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_config_register(i2c_mlx90614_handle_t mlx90614_handle, const i2c_mlx90614_config_register_t config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_CFGREG, config_reg.reg) );

    mlx90614_handle->config_reg = config_reg;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_pwmctrl_register(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t pwmctrl_reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_PWMCTRL, &pwmctrl_reg) );

    mlx90614_handle->pwmctrl_reg.reg = pwmctrl_reg;

    if(mlx90614_handle->pwmctrl_reg.bit.pwm_mode == I2C_MLX90614_PWM_MODE_SINGLE) {
        mlx90614_handle->pwmctrl_reg.pwm_period = (float)mlx90614_handle->pwmctrl_reg.bit.pwm_period_mult * 1.024;
    } else {
        mlx90614_handle->pwmctrl_reg.pwm_period = (float)mlx90614_handle->pwmctrl_reg.bit.pwm_period_mult * 2.048;
    }

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_pwmctrl_register(i2c_mlx90614_handle_t mlx90614_handle, const i2c_mlx90614_pwmctrl_register_t pwmctrl_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_PWMCTRL, pwmctrl_reg.reg) );

    mlx90614_handle->pwmctrl_reg = pwmctrl_reg;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_flags_register(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t flags_reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_READ_FLAGS_REG, &flags_reg) );

    mlx90614_handle->flags_reg.reg = flags_reg;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_init(i2c_master_bus_handle_t bus_handle, const i2c_mlx90614_config_t *mlx90614_config, i2c_mlx90614_handle_t *mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && mlx90614_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, mlx90614_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, mlx90614 device handle initialization failed", mlx90614_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_mlx90614_handle_t out_handle = (i2c_mlx90614_handle_t)calloc(1, sizeof(i2c_mlx90614_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mlx90614 device");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = mlx90614_config->dev_config.device_address,
        .scl_speed_hz       = mlx90614_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_CMD_DELAY_MS));

    /* copy configuration */
    out_handle->i2c_dev_address = mlx90614_config->dev_config.device_address;

    /* mlx90614 attempt to read configured identification numbers */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_ident_numbers(out_handle), err_handle, TAG, "i2c mlx90614 read identification numbers failed");

    /* mlx90614 attempt to read configuration register */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_config_register(out_handle), err_handle, TAG, "i2c mlx90614 read configuration register failed");

    /* mlx90614 attempt to read pwmctrl register */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_pwmctrl_register(out_handle), err_handle, TAG, "i2c mlx90614 read pwmctrl register failed");

    /* set device handle */
    *mlx90614_handle = out_handle;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MLX90614_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_mlx90614_get_temperatures(i2c_mlx90614_handle_t mlx90614_handle, float *const ambient_temperature, float *const object1_temperature, float *const object2_temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle && ambient_temperature && object1_temperature && object2_temperature );

    ESP_ERROR_CHECK( i2c_mlx90614_get_ambient_temperature(mlx90614_handle, ambient_temperature) );
    ESP_ERROR_CHECK( i2c_mlx90614_get_object1_temperature(mlx90614_handle, object1_temperature) );
    ESP_ERROR_CHECK( i2c_mlx90614_get_object2_temperature(mlx90614_handle, object2_temperature) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_ambient_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *const ambient_temperature) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TA, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *ambient_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object1_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *const object1_temperature) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TOBJ1, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object1_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object2_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *const object2_temperature) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TOBJ2, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object2_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_ir_channel1(i2c_mlx90614_handle_t mlx90614_handle, int16_t *const ir_channel1) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    uint16_t raw_data;
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_RAWIR1, &raw_data) );

    *ir_channel1 = i2c_mlx90614_decode_ir(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_ir_channel2(i2c_mlx90614_handle_t mlx90614_handle, int16_t *const ir_channel2) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    uint16_t raw_data;
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_RAWIR2, &raw_data) );

    *ir_channel2 = i2c_mlx90614_decode_ir(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_ambient_temperature_range(i2c_mlx90614_handle_t mlx90614_handle, float *const ambient_temperature_range) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TARANGE, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *ambient_temperature_range = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_emissivity(i2c_mlx90614_handle_t mlx90614_handle, float *const coefficient) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_EMISS, &raw_data) );

    // if we successfully read from the ke register
	// calculate the emissivity coefficient between 0.05 and 1.0:
    *coefficient = ((float)raw_data) / 0xffff;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_emissivity(i2c_mlx90614_handle_t mlx90614_handle, const float coefficient) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    // validate emissivity coefficient range is between 0.05 and 1.0
    ESP_RETURN_ON_FALSE((coefficient <= 1.0f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.05 and 1.0, set emissivity failed");
    ESP_RETURN_ON_FALSE((coefficient >= 0.05f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.05 and 1.0, set emissivity failed");

    uint16_t raw_data = (uint16_t)(0xffff * coefficient);

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_EMISS, raw_data) );

    return ESP_OK;
}


esp_err_t i2c_mlx90614_get_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *const temperature) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX, &raw_data) );

    *temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    uint16_t raw_data = i2c_mlx90614_encode_temperature(temperature);

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *const temperature) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN, &raw_data) );

    *temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle, const float temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    uint16_t raw_data = i2c_mlx90614_encode_temperature(temperature);

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_address(i2c_mlx90614_handle_t mlx90614_handle, uint8_t *const address) {
    uint16_t raw_data;

    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR, &raw_data) );

    *address = (uint8_t)raw_data;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_address(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t address) {
    uint16_t raw_data;
    
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    // make sure the address is within the proper range:
	if ((address >= 0x80) || (address == 0x00))
		return ESP_ERR_INVALID_ARG;

    // read the existing device address
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR, &raw_data) );

    raw_data &= 0xFF00;  // mask out the address (msb)
	raw_data |= address; // add the new address

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_sleep(i2c_mlx90614_handle_t mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t i2c_mlx90614_wakeup(i2c_mlx90614_handle_t mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t i2c_mlx90614_remove(i2c_mlx90614_handle_t mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    return i2c_master_bus_rm_device(mlx90614_handle->i2c_dev_handle);
}

esp_err_t i2c_mlx90614_delete(i2c_mlx90614_handle_t mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mlx90614_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_mlx90614_remove(mlx90614_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(mlx90614_handle->i2c_dev_handle) {
        free(mlx90614_handle->i2c_dev_handle);
        free(mlx90614_handle);
    }

    return ESP_OK;
}