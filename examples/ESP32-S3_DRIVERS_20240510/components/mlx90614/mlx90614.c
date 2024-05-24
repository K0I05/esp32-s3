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
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
 * @brief calculates mlx90614 crc8 value using a x^8+x^2+x^1+1 poly.  See datasheet for details.
 *
 * @param[in] crc crc recieved for data to validate
 * @param[in] data data to perform crc8 validation
 * @return calculated crc8 value.
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
 * @brief converts milliseconds to ticks.
 *
 * @param[in] ms milliseconds to convert to ticks.
 * @return converted ms in ticks.
 */
static inline size_t i2c_mlx90614_get_tick_duration(const uint16_t ms) {
    size_t res = pdMS_TO_TICKS(ms);

    return res == 0 ? 1 : res;
}

/**
 * @brief decodes raw `uint16_t` temperature to floating point temperature in degrees celsius.
 *
 * @param[in] raw_data raw `uint16_t` temperature to decode.
 * @return decoded floating point temperature in degrees celsius.
 */
static inline float i2c_mlx90614_decode_temperature(const uint16_t raw_data) {
    float temperature = (float)raw_data * 0.02;

    temperature -= 273.15;  // kelvin to celcius

    return temperature;
}

/**
 * @brief encodes floating point temperature in degrees celsius to raw `uint16_t` temperature.
 *
 * @param[in] temperature floating point temperature to encode.
 * @return encoded raw `uint16_t` temperature.
 */
static inline uint16_t i2c_mlx90614_encode_temperature(const float temperature) {
    float temp = temperature + 273.15;

	temp *= 50.0;  // then multiply by 0.02 degK / bit

	return (uint16_t)temp;
}

static inline esp_err_t i2c_mlx90614_read_word(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, uint16_t *data) {
    i2c_uint24_t buffer;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint24(mlx90614_handle->i2c_dev_handle, reg_addr, &buffer) );

    uint8_t crc = i2c_mlx90614_crc8(0, (mlx90614_handle->dev_params->address << 1));

    crc = i2c_mlx90614_crc8(crc, reg_addr);
	crc = i2c_mlx90614_crc8(crc, (mlx90614_handle->dev_params->address << 1) + 1);
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

static inline esp_err_t i2c_mlx90614_write_word(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, const uint16_t data) {
    uint8_t tx[I2C_UINT32_SIZE] = { 0, 0, 0, 0 };
    uint8_t crc; 

    ESP_ARG_CHECK( mlx90614_handle );

    tx[0] = reg_addr;       // register
    tx[1] = data & 0x00FF;  // lsb
    tx[2] = data >> 8;      // msb

    crc = i2c_mlx90614_crc8(0, (mlx90614_handle->dev_params->address << 1));
	crc = i2c_mlx90614_crc8(crc, tx[0]); // register
	crc = i2c_mlx90614_crc8(crc, tx[1]); // lsb
	crc = i2c_mlx90614_crc8(crc, tx[2]); // msb

    tx[3] = crc;            // pec

    ESP_ERROR_CHECK( i2c_master_transmit(mlx90614_handle->i2c_dev_handle, tx, I2C_UINT32_SIZE, -1) );

    return ESP_OK;
}

static inline esp_err_t i2c_mlx90614_write_eeprom(i2c_mlx90614_handle_t mlx90614_handle, const uint8_t reg_addr, const uint16_t data) {
    ESP_ARG_CHECK( mlx90614_handle );

    // clear eeprom register
    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, reg_addr, I2C_MLX90614_CMD_EEPROM_CLR_CELL) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay( i2c_mlx90614_get_tick_duration(10) );

    // write data to register
    ESP_ERROR_CHECK( i2c_mlx90614_write_word(mlx90614_handle, reg_addr, data) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay( i2c_mlx90614_get_tick_duration(10) );

    return ESP_OK;
}

/**
 * @brief reads the mlx90614 `uint64_t` identification number as two 32-bit values (hi and lo).
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_mlx90614_get_ident_numbers(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t id_num[I2C_UINT32_SIZE]; // 64-bit ident value

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM1, &id_num[0]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM2, &id_num[1]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM3, &id_num[2]) );
    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RD_IDNUM4, &id_num[3]) );

    mlx90614_handle->dev_params->ident_number_hi = id_num[2] | (id_num[3] << 16);
    mlx90614_handle->dev_params->ident_number_lo = id_num[0] | (id_num[1] << 16);

    return ESP_OK;
}

/**
 * @brief reads maximum object temperature range from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_mlx90614_get_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX, &raw_data) );

    mlx90614_handle->dev_params->obj_max_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

/**
 * @brief reads minimum object temperature range from the mlx90614.
 *
 * @param[in] mlx90614_handle mlx90614 device handle
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_mlx90614_get_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN, &raw_data) );

    mlx90614_handle->dev_params->obj_min_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_init(i2c_master_bus_handle_t bus_handle, const i2c_mlx90614_config_t *mlx90614_config, i2c_mlx90614_handle_t *mlx90614_handle) {
    esp_err_t             ret = ESP_OK;
    i2c_mlx90614_handle_t out_handle;

    ESP_ARG_CHECK( bus_handle && mlx90614_config );

    out_handle = (i2c_mlx90614_handle_t)calloc(1, sizeof(i2c_mlx90614_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mlx90614 device");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = mlx90614_config->dev_config.device_address,
        .scl_speed_hz       = I2C_MLX90614_DATA_RATE_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    out_handle->dev_params = (i2c_mlx90614_params_t*)calloc(1, sizeof(i2c_mlx90614_params_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mlx90614 device parameters");

    /* copy configuration */
    out_handle->dev_params->address = mlx90614_config->dev_config.device_address;

    /* mlx90614 attempt to read configured identification numbers */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_ident_numbers(out_handle), err, TAG, "i2c mlx90614 read identification numbers failed");

    /* mlx90614 attempt to read configured emissivity */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_emissivity(out_handle), err, TAG, "i2c mlx90614 read emissivity failed");

    /* mlx90614 attempt to read configured maximum and minimum object temperature */
    ESP_GOTO_ON_ERROR(i2c_mlx90614_get_object_temperature_ranges(out_handle), err, TAG, "i2c mlx90614 read object temperature ranges failed");

    /* set device handle */
    *mlx90614_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_mlx90614_get_ambient_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TA, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *ambient_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object1_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object1_temperature) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TOBJ1, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object1_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object2_temperature(i2c_mlx90614_handle_t mlx90614_handle, float *object2_temperature) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_RAM_READ_TOBJ2, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object2_temperature = i2c_mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_temperatures(i2c_mlx90614_handle_t mlx90614_handle, float *ambient_temperature, float *object1_temperature, float *object2_temperature) {
    ESP_ARG_CHECK( mlx90614_handle && ambient_temperature && object1_temperature && object2_temperature );

    ESP_ERROR_CHECK( i2c_mlx90614_get_ambient_temperature(mlx90614_handle, ambient_temperature) );
    ESP_ERROR_CHECK( i2c_mlx90614_get_object1_temperature(mlx90614_handle, object1_temperature) );
    ESP_ERROR_CHECK( i2c_mlx90614_get_object2_temperature(mlx90614_handle, object2_temperature) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_emissivity(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_EMISS, &raw_data) );

    // if we successfully read from the ke register
	// calculate the emissivity between 0.1 and 1.0:
    mlx90614_handle->dev_params->emissivity = ((float)raw_data) / 0xffff;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_emissivity(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    const float emissivity = mlx90614_handle->dev_params->emissivity;
    uint16_t raw_data = (uint16_t)(0xffff * emissivity);

    // make sure emissivity is between 0.1 and 1.0
	if ((emissivity > 1.0) || (emissivity < 0.1))
		return ESP_ERR_INVALID_ARG;

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_EMISS, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_object_temperature_ranges(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_get_object_maximum_temperature(mlx90614_handle) );
    ESP_ERROR_CHECK( i2c_mlx90614_get_object_minimum_temperature(mlx90614_handle) );

    return ESP_OK;    
}

esp_err_t i2c_mlx90614_set_object_maximum_temperature(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    const float maximum_temperature = mlx90614_handle->dev_params->obj_max_temperature;
    uint16_t raw_data = i2c_mlx90614_encode_temperature(maximum_temperature);

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMAX, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_object_minimum_temperature(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    const float minimum_temperature = mlx90614_handle->dev_params->obj_min_temperature;
    uint16_t raw_data = i2c_mlx90614_encode_temperature(minimum_temperature);

    ESP_ERROR_CHECK( i2c_mlx90614_write_eeprom(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_TOMIN, raw_data) );

    return ESP_OK;
}

esp_err_t i2c_mlx90614_get_address(i2c_mlx90614_handle_t mlx90614_handle) {
    uint16_t raw_data;

    ESP_ARG_CHECK( mlx90614_handle );

    ESP_ERROR_CHECK( i2c_mlx90614_read_word(mlx90614_handle, I2C_MLX90614_CMD_EEPROM_RDWR_SMBADDR, &raw_data) );

    mlx90614_handle->dev_params->address = (uint8_t)raw_data;

    return ESP_OK;
}

esp_err_t i2c_mlx90614_set_address(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    const uint8_t address = mlx90614_handle->dev_params->address;
    uint16_t raw_data;

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
    ESP_ARG_CHECK( mlx90614_handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t i2c_mlx90614_wake(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t i2c_mlx90614_rm(i2c_mlx90614_handle_t mlx90614_handle) {
    ESP_ARG_CHECK( mlx90614_handle );

    return i2c_master_bus_rm_device(mlx90614_handle->i2c_dev_handle);
}