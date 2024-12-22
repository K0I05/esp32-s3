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
 * @file veml7700.c
 *
 * ESP-IDF driver for VEML7700 illuminance sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "veml7700.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * VEML7700 definitions
*/
#define I2C_VEML7700_DEVICE_ID          UINT8_C(0x81)   //!< veml7700 device identifier (fixed)
#define I2C_VEML7700_POLY_COEF_A        (6.0135e-13)
#define I2C_VEML7700_POLY_COEF_B        (-9.3924e-9)
#define I2C_VEML7700_POLY_COEF_C        (8.1488e-5)
#define I2C_VEML7700_POLY_COEF_D        (1.0023)

#define I2C_VEML7700_CMD_ALS_CONF       UINT8_C(0x00)
#define I2C_VEML7700_CMD_ALS_WH         UINT8_C(0x01)
#define I2C_VEML7700_CMD_ALS_WL         UINT8_C(0x02)
#define I2C_VEML7700_CMD_POWER_SAVING   UINT8_C(0x03)
#define I2C_VEML7700_CMD_ALS            UINT8_C(0x04)
#define I2C_VEML7700_CMD_WHITE          UINT8_C(0x05)
#define I2C_VEML7700_CMD_ALS_INT        UINT8_C(0x06)
#define I2C_VEML7700_CMD_ID             UINT8_C(0x07)

#define I2C_VEML7700_POWERUP_DELAY_MS   UINT16_C(5)     /*!< veml7700 delay on power-up before attempting I2C transactions */
#define I2C_VEML7700_APPSTART_DELAY_MS  UINT16_C(10)    /*!< veml7700 delay after initialization before application start-up */
#define I2C_VEML7700_CMD_DELAY_MS       UINT16_C(5)     /*!< veml7700 delay before attempting I2C transactions after a command is issued */
#define I2C_VEML7700_RETRY_DELAY_MS     UINT16_C(2)     /*!< veml7700 delay between an I2C receive transaction retry */
#define I2C_VEML7700_TX_RX_DELAY_MS     UINT16_C(10)    /*!< veml7700 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_VEML7700_GAIN_OPTIONS_COUNT UINT8_C(4)	    /*!< Possible gain values count */
#define I2C_VEML7700_IT_OPTIONS_COUNT   UINT8_C(6)      /*!< Possible integration time values count */


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "veml7700";

/**
 * @brief List of all possible values for configuring sensor gain.
 * 
 */
static const uint8_t i2c_veml7700_gain_values[I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
    I2C_VEML7700_GAIN_2,
    I2C_VEML7700_GAIN_1,
    I2C_VEML7700_GAIN_DIV_8,
    I2C_VEML7700_GAIN_DIV_4
};

/**
 * @brief List of all possible values for configuring sensor integration time.
 * 
 */
static const uint8_t i2c_veml7700_integration_time_values[I2C_VEML7700_IT_OPTIONS_COUNT] = {
    I2C_VEML7700_INTEGRATION_TIME_800MS,
    I2C_VEML7700_INTEGRATION_TIME_400MS,
    I2C_VEML7700_INTEGRATION_TIME_200MS,
    I2C_VEML7700_INTEGRATION_TIME_100MS,
    I2C_VEML7700_INTEGRATION_TIME_50MS,
    I2C_VEML7700_INTEGRATION_TIME_25MS
};

/**
 * @brief Proper resolution multipliers mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 17-Jan-2024
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const float i2c_veml7700_resolution_map[I2C_VEML7700_IT_OPTIONS_COUNT][I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
    {0.0042, 0.0084, 0.0336, 0.0672},
    {0.0084, 0.0168, 0.0672, 0.1344},
    {0.0168, 0.0336, 0.1344, 0.2688},
    {0.0336, 0.0672, 0.2688, 0.5376},
    {0.0672, 0.1344, 0.5376, 1.0752},
    {9.1344, 0.2688, 1.0752, 2.1504}
};

/**
 * @brief Maximum luminocity mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 17-Jan-2024
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const uint32_t i2c_veml7700_maximums_map[I2C_VEML7700_IT_OPTIONS_COUNT][I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
    {275,   550,    2202,   4404},
    {550,   1101,   4404,   8808},
    {1101,  2202,   8808,   17616},
    {2202,  4404,   17616,  35232},
    {4404,  8808,   35232,  70463},
    {8808,  17616,  70463,  140926}
};

/**
 * @brief Find the index of a given element within an array.
 * 
 * This is a standard implementation of a commonly used function which can be
 * found online.
 * 
 * @param elm		Value of the element we are searching for
 * @param ar 		The array in which to search
 * @param len 		Length of the given array
 * 
 * @return uint8_t 
 * 		- n Index within the array
 * 		- -1 Element not found.
 */
static inline uint8_t i2c_veml7700_index_of(uint8_t elm, const uint8_t *ar, uint8_t len) {
    while (len--) { if (ar[len] == elm) { return len; } } return -1;
}

/**
 * @brief Get the index of the integration time value within the list of possible
 * integration time values.
 * 
 * @param integration_time The integration time value to search for.
 * 
 * @return int The index within the array of possible integration times.
 */
static inline int i2c_veml7700_get_it_index(uint8_t integration_time) {
	return i2c_veml7700_index_of(integration_time, i2c_veml7700_integration_time_values, I2C_VEML7700_IT_OPTIONS_COUNT);
}

/**
 * @brief Get the index of the gain value within the list of possible
 * gain values.
 * 
 * @param gain The gain value to search for.
 * 
 * @return int The index within the array of possible gains.
 */
static inline int i2c_veml7700_get_gain_index(uint8_t gain) {
	return i2c_veml7700_index_of(gain, i2c_veml7700_gain_values, I2C_VEML7700_GAIN_OPTIONS_COUNT);
}

/**
 * @brief The maximum possible lux value for any configuration on this
 * sensor.
 * 
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_maximum_lux() {
	return i2c_veml7700_maximums_map[I2C_VEML7700_IT_OPTIONS_COUNT - 1][I2C_VEML7700_GAIN_OPTIONS_COUNT - 1];
}

/**
 * @brief Get the smallest possible maximum lux value for this sensor.
 * 
 * @return uint32_t The smallest maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_lowest_maximum_lux() {
	return i2c_veml7700_maximums_map[0][0];
}

/**
 * @brief Get the next smallest maximum lux limit value.
 * 
 * Used to identify if a better range is possible for the current 
 * light levels.
 * 
 * @param veml7700_handle Handle for the device
 * @param lux The referent lux value, usually the last result.
 * 
 * @return uint32_t The next smallest maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_lower_maximum_lux(i2c_veml7700_handle_t veml7700_handle, float* lux) {
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	// find the next smallest 'maximum' value in the mapped maximum luminocities
	if ((gain_index > 0) && (it_index > 0)) {
		if (i2c_veml7700_maximums_map[it_index][gain_index - 1] >= i2c_veml7700_maximums_map[it_index - 1][gain_index]) {
			return i2c_veml7700_maximums_map[it_index][gain_index - 1];
		} else {
			return i2c_veml7700_maximums_map[it_index - 1][gain_index];
		}
	} else if ((gain_index > 0) && (it_index == 0)) {
		return i2c_veml7700_maximums_map[it_index][gain_index - 1];
	} else {
		return i2c_veml7700_maximums_map[it_index - 1][gain_index];
	}
}

/**
 * @brief Reads the maximum lux for the currentl configuration.
 * 
 * @param veml7700_handle Handle for the device
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_current_maximum_lux(i2c_veml7700_handle_t veml7700_handle) {
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	return i2c_veml7700_maximums_map[it_index][gain_index];
}

/**
 * @brief Decrease either gain and/or integration time in configuration.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @param veml7700_handle Handle for the device
 * @return void 
 */
static inline void i2c_veml7700_decrease_resolution(i2c_veml7700_handle_t veml7700_handle) {
	// identify the indexes of the currently configured values
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	ESP_LOGD(TAG, "Decreasing sensor resolution...\n");

	// if this is the last gain or integration time setting, increment the other property
	if (gain_index == 3) {
		veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_time_values[it_index + 1];
	} else if (it_index == 5) {
		veml7700_handle->config_reg.bits.gain = i2c_veml7700_gain_values[gain_index + 1];
	} else {
		// check which adjacent value is bigger than the current, but choose the smaller if both are
		if (i2c_veml7700_resolution_map[it_index + 1][gain_index] > veml7700_handle->resolution) {
			if (i2c_veml7700_resolution_map[it_index + 1][gain_index] <= i2c_veml7700_resolution_map[it_index][gain_index + 1]) {
				veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_time_values[it_index + 1];
			}
		} else if (i2c_veml7700_resolution_map[it_index][gain_index + 1] > veml7700_handle->resolution) {
			if (i2c_veml7700_resolution_map[it_index][gain_index + 1] <= i2c_veml7700_resolution_map[it_index + 1][gain_index]) {
				veml7700_handle->config_reg.bits.gain = i2c_veml7700_gain_values[gain_index + 1];
			}
		} else {
			ESP_LOGE(TAG, "Failed to decrease sensor resolution.");
		}
	}
}

/**
 * @brief Increase either gain and/or integration time in configuration.
 * 
 * @note Does not match official recommended algorithm.
 * 
 * @param veml7700_handle Handle for the device
 * @return void 
 */
static inline void i2c_veml7700_increase_resolution(i2c_veml7700_handle_t veml7700_handle) {
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	if ((gain_index > 0) && (it_index > 0)) {
		if (i2c_veml7700_maximums_map[it_index][gain_index - 1] >= i2c_veml7700_maximums_map[it_index - 1][gain_index]) {
			veml7700_handle->config_reg.bits.gain = i2c_veml7700_gain_values[gain_index - 1];
		} else {
			veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_time_values[it_index - 1];
		}
	} else if ((gain_index > 0) && (it_index == 0)) {
		veml7700_handle->config_reg.bits.gain = i2c_veml7700_gain_values[gain_index - 1];
	} else {
		veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_time_values[it_index - 1];
	}
}

/**
 * @brief Auto-resolution adjustment algorithm implementation for
 *      VEML7700 light sensor.
 * 
 * @note  Does not match official recommended algorithm.
 * 
 * @param veml7700_handle Handle for the device
 * @param lux Luminocity value for which we are optimizing.
 * 
 * @return esp_err_t 
 */
static inline esp_err_t i2c_veml7700_optimize_configuration(i2c_veml7700_handle_t veml7700_handle, float *lux) {
	if (ceil(*lux) >= i2c_veml7700_get_current_maximum_lux(veml7700_handle)) {	// Decrease resolution
		// ensure we haven't reached the upper luminocity limit
		if (veml7700_handle->maximum_lux == i2c_veml7700_get_maximum_lux(veml7700_handle)) {
			ESP_LOGD(TAG, "Already configured for maximum luminocity.");
			return ESP_FAIL;
		}
		i2c_veml7700_decrease_resolution(veml7700_handle);
	} else if (*lux < i2c_veml7700_get_lower_maximum_lux(veml7700_handle, lux)) {	// Increase resolution
		// ensure this isn't the smallest maximum
		if (veml7700_handle->maximum_lux == i2c_veml7700_get_lowest_maximum_lux(veml7700_handle)) {
			ESP_LOGD(TAG, "Already configured with maximum resolution.");
			return ESP_FAIL;
		}
		i2c_veml7700_increase_resolution(veml7700_handle);
	} else {
		ESP_LOGD(TAG, "Configuration already optimal.");
		return ESP_FAIL;
	}

	ESP_LOGD(TAG, "Configuration optimized.");
	return ESP_OK;
}

static inline float i2c_veml7700_get_resolution(i2c_veml7700_handle_t veml7700_handle)
{
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	return i2c_veml7700_resolution_map[it_index][gain_index];
}

esp_err_t i2c_veml7700_get_configuration_register(i2c_veml7700_handle_t veml7700_handle) {
    uint16_t reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_CONF, &reg), TAG, "read configuration register failed" );

    /* set handle register */
    veml7700_handle->config_reg.reg = reg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_configuration_register(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_configuration_register_t config_reg) {
    i2c_veml7700_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy register */
    config.reg = config_reg.reg;

    /* set reserved values */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;
    config.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_CONF, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_configuration_register(veml7700_handle), TAG, "read configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_threshold_registers(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_WH, &veml7700_handle->hi_threshold_reg), TAG, "read high threshold register failed" );
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_WL, &veml7700_handle->lo_threshold_reg), TAG, "read low threshold register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_threshold_registers(i2c_veml7700_handle_t veml7700_handle, const uint16_t hi_threshold, const uint16_t lo_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_WH, hi_threshold), TAG, "write high threshold register failed" );
    
    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_WL, lo_threshold), TAG, "write low threshold register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_threshold_registers(veml7700_handle), TAG, "read threshold registers failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_power_saving_mode_register(i2c_veml7700_handle_t veml7700_handle) {
    uint16_t reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_POWER_SAVING, &reg), TAG, "read power saving mode register failed" );

    /* set handle register */
    veml7700_handle->power_saving_mode_reg.reg = reg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_power_saving_mode_register(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_power_saving_mode_register_t power_saving_mode_reg) {
    i2c_veml7700_power_saving_mode_register_t power_saving_mode;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy register */
    power_saving_mode.reg = power_saving_mode_reg.reg;

    /* set reserved values */
    power_saving_mode.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_POWER_SAVING, power_saving_mode.reg), TAG, "write power saving mode register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_power_saving_mode_register(veml7700_handle), TAG, "read power saving mode register failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_interrupt_status_register(i2c_veml7700_handle_t veml7700_handle) {
    uint16_t reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS_INT, &reg), TAG, "read interrupt status register failed" );

    /* set handle register */
    veml7700_handle->interrupt_status_reg.reg = reg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_identifier_register(i2c_veml7700_handle_t veml7700_handle) {
    uint16_t reg = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ID, &reg), TAG, "read identifier register failed" );

    /* set handle register */
    veml7700_handle->identifier_reg.reg = reg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}


esp_err_t i2c_veml7700_init(i2c_master_bus_handle_t bus_handle, const i2c_veml7700_config_t *veml7700_config, i2c_veml7700_handle_t *veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && veml7700_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, veml7700_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, veml7700 device handle initialization failed", veml7700_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_veml7700_handle_t out_handle = (i2c_veml7700_handle_t)calloc(1, sizeof(i2c_veml7700_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for device, veml7700 device handle initialization failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = veml7700_config->dev_config.device_address,
        .scl_speed_hz       = veml7700_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c0 new bus failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));

    /* attempt to read initialization registers */
    
    /* attempt to read configuration register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_get_configuration_register(out_handle), err_handle, TAG, "read configuration register failed");

    /* attempt to read threshold registers */
    ESP_GOTO_ON_ERROR(i2c_veml7700_get_threshold_registers(out_handle), err_handle, TAG, "read threshold registers failed");

    /* attempt to read power saving mode register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_get_power_saving_mode_register(out_handle), err_handle, TAG, "read power saving mode register failed");

    /* attempt to read interrupt status register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_get_interrupt_status_register(out_handle), err_handle, TAG, "read interrupt status register failed");

    /* attempt to read identifier register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_get_identifier_register(out_handle), err_handle, TAG, "read identifier register failed");

    /* attempt to set initialization registers from configuration */
    i2c_veml7700_configuration_register_t       cfg_reg     = { .reg = out_handle->config_reg.reg };
    uint16_t                                    hi_thld_reg = out_handle->hi_threshold_reg;
    uint16_t                                    lo_thld_reg = out_handle->lo_threshold_reg;
    i2c_veml7700_power_saving_mode_register_t   psm_reg     = { .reg = out_handle->power_saving_mode_reg.reg };
    
    /* set configuration register */
    cfg_reg.bits.gain                   = veml7700_config->gain;
    cfg_reg.bits.integration_time       = veml7700_config->integration_time;
    cfg_reg.bits.persistence_protect    = veml7700_config->persistence_protect;
    cfg_reg.bits.irq_enabled            = veml7700_config->irq_enabled;
    cfg_reg.bits.shutdown               = veml7700_config->shutdown;

    // set the resolution on the configuration struct
	out_handle->resolution  = i2c_veml7700_get_resolution(out_handle);
	// set the current maximum value on the configuration struct
	out_handle->maximum_lux = i2c_veml7700_get_current_maximum_lux(out_handle);

    /* set power saving register */
    psm_reg.bits.power_saving_enabled   = veml7700_config->power_saving_enabled;
    psm_reg.bits.power_saving_mode      = veml7700_config->power_saving_mode;

    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_set_configuration_register(out_handle, cfg_reg), err_handle, TAG, "write configuration register failed");

    /* attempt to write threshold registers */
    ESP_GOTO_ON_ERROR(i2c_veml7700_set_threshold_registers(out_handle, hi_thld_reg, lo_thld_reg), err_handle, TAG, "write threshold registers failed");

    /* attempt to write power saving mode register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_set_power_saving_mode_register(out_handle, psm_reg), err_handle, TAG, "write power saving mode register failed");

    /* set device handle */
    *veml7700_handle = out_handle;

    /* application start delay  */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}


esp_err_t i2c_veml7700_get_ambient_light(i2c_veml7700_handle_t veml7700_handle, float *lux) {
    uint16_t raw_lux = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && lux);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS, &raw_lux), TAG, "read ambient light failed" );

    *lux = (float)(raw_lux) * i2c_veml7700_get_resolution(veml7700_handle);

    /* polynomial correction */
    *lux = (((I2C_VEML7700_POLY_COEF_A * *lux + I2C_VEML7700_POLY_COEF_B) * *lux + I2C_VEML7700_POLY_COEF_C) * *lux + I2C_VEML7700_POLY_COEF_D) * *lux;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_ambient_light_auto(i2c_veml7700_handle_t veml7700_handle, float *lux) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && lux);

    i2c_veml7700_get_ambient_light(veml7700_handle, lux);

	ESP_LOGD(TAG, "Configured maximum luminocity: %" PRIu32 "\n", veml7700_handle->maximum_lux);
	ESP_LOGD(TAG, "Configured resolution: %0.4f\n", veml7700_handle->resolution);
	
	// calculate and automatically reconfigure the optimal sensor configuration
	if (i2c_veml7700_optimize_configuration(veml7700_handle, lux) == ESP_OK) {
		// read again
		return i2c_veml7700_get_ambient_light(veml7700_handle, lux);
	}

	return ESP_OK;
}

esp_err_t i2c_veml7700_get_white_channel(i2c_veml7700_handle_t veml7700_handle, float *lux) {
    uint16_t raw_lux = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && lux);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_WHITE, &raw_lux), TAG, "read white channel failed" );

    *lux = (float)(raw_lux) * i2c_veml7700_get_resolution(veml7700_handle);

    /* polynomial correction */
    *lux = (((I2C_VEML7700_POLY_COEF_A * *lux + I2C_VEML7700_POLY_COEF_B) * *lux + I2C_VEML7700_POLY_COEF_C) * *lux + I2C_VEML7700_POLY_COEF_D) * *lux;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_white_channel_auto(i2c_veml7700_handle_t veml7700_handle, float *lux) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && lux);

    i2c_veml7700_get_white_channel(veml7700_handle, lux);

	ESP_LOGD(TAG, "Configured maximum luminocity: %" PRIu32 "\n", veml7700_handle->maximum_lux);
	ESP_LOGD(TAG, "Configured resolution: %0.4f\n", veml7700_handle->resolution);
	
	// calculate and automatically reconfigure the optimal sensor configuration
	if (i2c_veml7700_optimize_configuration(veml7700_handle, lux) == ESP_OK) {
		// read again
		return i2c_veml7700_get_white_channel(veml7700_handle, lux);
	}

	return ESP_OK;
}

esp_err_t veml7700_get_interrupt_status(i2c_veml7700_handle_t veml7700_handle, bool *hi_threshold_exceeded, bool *lo_threshold_exceeded) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    ESP_RETURN_ON_ERROR( i2c_veml7700_get_interrupt_status_register(veml7700_handle), TAG, "read interrupt status register for interrupt status failed" );

    *hi_threshold_exceeded = veml7700_handle->interrupt_status_reg.bits.hi_threshold_exceeded;
    *lo_threshold_exceeded = veml7700_handle->interrupt_status_reg.bits.lo_threshold_exceeded;

    return ESP_OK;
}

esp_err_t i2c_veml7700_shutdown(i2c_veml7700_handle_t veml7700_handle) {
    i2c_veml7700_configuration_register_t config_reg;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    config_reg.reg = veml7700_handle->config_reg.reg;

    /* shutdown device */
    config_reg.bits.shutdown = true;

    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for shutdown failed" );

    veml7700_handle->sleeping = true;

    return ESP_OK;
}

esp_err_t i2c_veml7700_wakeup(i2c_veml7700_handle_t veml7700_handle) {
    i2c_veml7700_configuration_register_t config_reg;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    config_reg.reg = veml7700_handle->config_reg.reg;

    /* wakeup device */
    config_reg.bits.shutdown = false;

    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for wake-up failed" );

    veml7700_handle->sleeping = false;

    return ESP_OK;
}

esp_err_t i2c_veml7700_rm(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    return i2c_master_bus_rm_device(veml7700_handle->i2c_dev_handle);
}

esp_err_t i2c_veml7700_del(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_veml7700_rm(veml7700_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(veml7700_handle->i2c_dev_handle) {
        free(veml7700_handle->i2c_dev_handle);
        free(veml7700_handle);
    }

    return ESP_OK;
}