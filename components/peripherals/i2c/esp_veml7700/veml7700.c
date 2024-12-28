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
#define I2C_VEML7700_IT_TIMES_COUNT     UINT8_C(6)      /*!< Possible integration time values count */
#define I2C_VEML7700_IT_OPTIONS_COUNT   UINT8_C(2)      /*!< Possible integration time values count */
#define I2C_VEML7700_PSM_TIMES_COUNT    UINT8_C(24) 
#define I2C_VEML7700_PSM_OPTIONS_COUNT  UINT8_C(4)

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
static const uint8_t i2c_veml7700_gains[I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
    I2C_VEML7700_GAIN_DIV_8,
    I2C_VEML7700_GAIN_DIV_4,
    I2C_VEML7700_GAIN_1,
    I2C_VEML7700_GAIN_2
};

/**
 * @brief List of all possible values for configuring sensor integration time.
 * 
 */
static const uint8_t i2c_veml7700_integration_times[I2C_VEML7700_IT_TIMES_COUNT] = {
    I2C_VEML7700_INTEGRATION_TIME_25MS,  
    I2C_VEML7700_INTEGRATION_TIME_50MS,  
    I2C_VEML7700_INTEGRATION_TIME_100MS, 
    I2C_VEML7700_INTEGRATION_TIME_200MS, 
    I2C_VEML7700_INTEGRATION_TIME_400MS, 
    I2C_VEML7700_INTEGRATION_TIME_800MS,
};


static const uint16_t i2c_veml7700_integration_time_map[I2C_VEML7700_IT_TIMES_COUNT][I2C_VEML7700_IT_OPTIONS_COUNT] = {
    {I2C_VEML7700_INTEGRATION_TIME_25MS,  24},
    {I2C_VEML7700_INTEGRATION_TIME_50MS,  50},
    {I2C_VEML7700_INTEGRATION_TIME_100MS, 100},
    {I2C_VEML7700_INTEGRATION_TIME_200MS, 200},
    {I2C_VEML7700_INTEGRATION_TIME_400MS, 400},
    {I2C_VEML7700_INTEGRATION_TIME_800MS, 800}
};

static const uint16_t i2c_veml7700_psm_refresh_time_map[I2C_VEML7700_PSM_TIMES_COUNT][I2C_VEML7700_PSM_OPTIONS_COUNT] = {
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_25MS, 25, 525},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_25MS, 25, 1025},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_25MS, 25, 2025},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_25MS, 25, 4025},
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_50MS, 50, 550},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_50MS, 50, 1050},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_50MS, 50, 2050},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_50MS, 50, 4050},
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_100MS, 100, 600},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_100MS, 100, 1100},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_100MS, 100, 2100},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_100MS, 100, 4100},
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_200MS, 200, 700},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_200MS, 200, 1200},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_200MS, 200, 2200},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_200MS, 200, 4200},
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_400MS, 400, 900},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_400MS, 400, 1400},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_400MS, 400, 2400},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_400MS, 400, 4400},
    {I2C_VEML7700_POWER_SAVING_MODE_1, I2C_VEML7700_INTEGRATION_TIME_800MS, 800, 1300},
    {I2C_VEML7700_POWER_SAVING_MODE_2, I2C_VEML7700_INTEGRATION_TIME_800MS, 800, 1800},
    {I2C_VEML7700_POWER_SAVING_MODE_3, I2C_VEML7700_INTEGRATION_TIME_800MS, 800, 2800},
    {I2C_VEML7700_POWER_SAVING_MODE_4, I2C_VEML7700_INTEGRATION_TIME_800MS, 800, 4800}
};


/**
 * @brief Proper resolution multipliers mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 17-Jan-2024
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const float i2c_veml7700_resolution_map[I2C_VEML7700_IT_TIMES_COUNT][I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
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
static const uint32_t i2c_veml7700_maximums_map[I2C_VEML7700_IT_TIMES_COUNT][I2C_VEML7700_GAIN_OPTIONS_COUNT] = {
    {275,   550,    2202,   4404},
    {550,   1101,   4404,   8808},
    {1101,  2202,   8808,   17616},
    {2202,  4404,   17616,  35232},
    {4404,  8808,   35232,  70463},
    {8808,  17616,  70463,  140926}
};

/**
 * @brief Finds the index of a given element within an array.
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
static inline uint8_t i2c_veml7700_index_of(const uint8_t elm, const uint8_t *ar, const uint8_t len) {
    uint8_t size = len;
    while (size--) { if(ar[size] == elm) { return size; } } return -1;
}

/**
 * @brief Gets the index of the integration time value within the list of possible
 * integration time values.
 * 
 * @param integration_time The integration time value to search for.
 * 
 * @return int The index within the array of possible integration times.
 */
static inline int i2c_veml7700_get_it_index(const i2c_veml7700_integration_times_t integration_time) {
	return i2c_veml7700_index_of(integration_time, i2c_veml7700_integration_times, I2C_VEML7700_IT_TIMES_COUNT);
}

/**
 * @brief Gets the index of the gain value within the list of possible
 * gain values.
 * 
 * @param gain The gain value to search for.
 * 
 * @return int The index within the array of possible gains.
 */
static inline int i2c_veml7700_get_gain_index(const i2c_veml7700_gains_t gain) {
	return i2c_veml7700_index_of(gain, i2c_veml7700_gains, I2C_VEML7700_GAIN_OPTIONS_COUNT);
}

/**
 * @brief Gets the index of the power saving mode and integration time value within the list of possible values.
 * 
 * @param psm_mode The power saving mode value to search for.
 * @param integration_time The integration time value to search for.
 * @return int The index within the array of possible values.
 */
static inline int i2c_veml7700_get_psm_mode_index(const i2c_veml7700_power_saving_modes_t psm_mode, const i2c_veml7700_integration_times_t integration_time) {
    uint8_t size = I2C_VEML7700_PSM_TIMES_COUNT;
    while (size--) { 
        if(i2c_veml7700_psm_refresh_time_map[size][0] == psm_mode && i2c_veml7700_psm_refresh_time_map[size][1] == integration_time) { 
            return size; 
        } 
    } 
    return -1;
}

/**
 * @brief The maximum possible lux value for any configuration on this
 * sensor.
 * 
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_maximum_lux(void) {
	return i2c_veml7700_maximums_map[I2C_VEML7700_IT_TIMES_COUNT - 1][I2C_VEML7700_GAIN_OPTIONS_COUNT - 1];
}

/**
 * @brief Gets the smallest possible maximum lux value for this sensor.
 * 
 * @return uint32_t The smallest maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_lowest_maximum_lux(void) {
	return i2c_veml7700_maximums_map[0][0];
}

/**
 * @brief Gets the next smallest maximum lux limit value.
 * 
 * Used to identify if a better range is possible for the current 
 * light levels.
 * 
 * @param veml7700_handle Handle for the device
 * 
 * @return uint32_t The next smallest maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_lower_maximum_lux(i2c_veml7700_handle_t veml7700_handle) {
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
 * @brief Reads the maximum lux for the current configuration.
 * 
 * @param veml7700_handle VEML7700 device handle.
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t i2c_veml7700_get_current_maximum_lux(i2c_veml7700_handle_t veml7700_handle) {
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	return i2c_veml7700_maximums_map[it_index][gain_index];
}

/**
 * @brief Reads gain and integration time to calculate resolution.
 * 
 * @param veml7700_handle VEML7700 device handle.
 * @return float Calculated resolution.
 */
static inline float i2c_veml7700_get_resolution(i2c_veml7700_handle_t veml7700_handle)
{
	int gain_index = i2c_veml7700_get_gain_index(veml7700_handle->config_reg.bits.gain);
	int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);

	return i2c_veml7700_resolution_map[it_index][gain_index];
}


esp_err_t i2c_veml7700_optimize_configuration___(i2c_veml7700_handle_t veml7700_handle) {
    bool optimized = false; uint16_t als_counts; 

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* enable power */
    ESP_RETURN_ON_ERROR( i2c_veml7700_enable_power(veml7700_handle), TAG, "enable power failed" );

    /* set baseline integration time and gain */
    int it_index                                      = i2c_veml7700_get_it_index(I2C_VEML7700_INTEGRATION_TIME_100MS); /* set baseline integration time to 100ms */
    int gain_index                                    = i2c_veml7700_get_gain_index(I2C_VEML7700_GAIN_DIV_8);           /* set baseline gain to 1/8 */
    veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_times[it_index];
    veml7700_handle->config_reg.bits.gain             = i2c_veml7700_gains[gain_index];

    /* set baseline configuration */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, veml7700_handle->config_reg), TAG, "write configuration register failed" );

    /* optimize sensor */
    do {
        /* read ambient light counts */
        ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light_counts(veml7700_handle, &als_counts), TAG, "read ambient light counts failed" );

        /* validate als counts <= 100 */
        if(als_counts <= 100) {
            /* disable power */
            ESP_RETURN_ON_ERROR( i2c_veml7700_disable_power(veml7700_handle), TAG, "disable power failed" );

            /* validate index is within range */
            if(gain_index < I2C_VEML7700_GAIN_OPTIONS_COUNT - 1) {
                /* if als counts ≤ 100 counts, set higher gain*/
                ESP_RETURN_ON_ERROR(i2c_veml7700_set_gain(veml7700_handle, i2c_veml7700_gains[++gain_index]), TAG, "write gain failed" );
            }

            /* increase integration time if gain is I2C_VEML7700_GAIN_2 */
            if(veml7700_handle->config_reg.bits.gain == I2C_VEML7700_GAIN_2) {
                /* validate index is within range */
                if(it_index < I2C_VEML7700_IT_TIMES_COUNT - 1) {
                    /* increase integration time */
                    ESP_RETURN_ON_ERROR(i2c_veml7700_set_integration_time(veml7700_handle, i2c_veml7700_integration_times[++it_index]), TAG, "write integration time failed" );
                }
            } else {
                /* enable power */
                ESP_RETURN_ON_ERROR( i2c_veml7700_enable_power(veml7700_handle), TAG, "enable power failed" );
            }

            /* sensor is optimize when integration time is I2C_VEML7700_INTEGRATION_TIME_800MS */
            if(veml7700_handle->config_reg.bits.integration_time == I2C_VEML7700_INTEGRATION_TIME_800MS) {
                ESP_LOGI(TAG, "IT     %d", veml7700_handle->config_reg.bits.integration_time);
                ESP_LOGI(TAG, "Gain   %d", veml7700_handle->config_reg.bits.gain);
                ESP_LOGI(TAG, "Counts %d", als_counts);

                /* sensor is optimized */
                optimized = true;
                break;
            } else {
                /* enable power */
                ESP_RETURN_ON_ERROR( i2c_veml7700_enable_power(veml7700_handle), TAG, "enable power failed" );
            }
        } else {
            /* validate als between 100 and 65535 count */
            if(als_counts > 10000) {
                /* validate index is within range */
                if(it_index > -1) {
                    /* decrease integration time */
                    ESP_RETURN_ON_ERROR(i2c_veml7700_set_integration_time(veml7700_handle, i2c_veml7700_integration_times[--it_index]), TAG, "write integration time failed" );
                }

                /* validate als counts if integration time is I2C_VEML7700_INTEGRATION_TIME_25MS */
                if(veml7700_handle->config_reg.bits.integration_time == I2C_VEML7700_INTEGRATION_TIME_25MS) {
                    /* validate als counts >= 200 */
                    if(als_counts >= 200) {
                        ESP_LOGI(TAG, "IT     %d", veml7700_handle->config_reg.bits.integration_time);
                        ESP_LOGI(TAG, "Gain   %d", veml7700_handle->config_reg.bits.gain);
                        ESP_LOGI(TAG, "Counts %d", als_counts);

                        /* sensor is optimized */
                        optimized = true;
                        break;
                    }
                }
            } else {
                /* validate als counts 100 to 10000 */
                if(als_counts >= 100 && als_counts <= 10000) {
                    ESP_LOGI(TAG, "IT     %d", veml7700_handle->config_reg.bits.integration_time);
                    ESP_LOGI(TAG, "Gain   %d", veml7700_handle->config_reg.bits.gain);
                    ESP_LOGI(TAG, "Counts %d", als_counts);

                    /* sensor is optimized */
                    optimized = true;
                    break;
                }
            }
        }
    } while (optimized == false);

    /* enable power */
    ESP_RETURN_ON_ERROR( i2c_veml7700_enable_power(veml7700_handle), TAG, "enable power failed" );

    return ESP_OK;
}


esp_err_t i2c_veml7700_optimize_configuration(i2c_veml7700_handle_t veml7700_handle) {
    uint16_t als_counts; 

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* enable power */
    ESP_RETURN_ON_ERROR( i2c_veml7700_enable_power(veml7700_handle), TAG, "enable power failed" );

    /* set baseline integration time and gain values */
    int it_index                                      = i2c_veml7700_get_it_index(I2C_VEML7700_INTEGRATION_TIME_100MS); /* set baseline integration time to 100ms */
    int gain_index                                    = i2c_veml7700_get_gain_index(I2C_VEML7700_GAIN_DIV_8);           /* set baseline gain to 1/8 */
    veml7700_handle->config_reg.bits.integration_time = i2c_veml7700_integration_times[it_index];
    veml7700_handle->config_reg.bits.gain             = i2c_veml7700_gains[gain_index];

    /* set baseline configuration */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, veml7700_handle->config_reg), TAG, "write configuration register failed" );

    /* read ambient light counts */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light_counts(veml7700_handle, &als_counts), TAG, "read ambient light counts failed" );

    if (als_counts <= 100) {
        // increase gain and then integration time as needed
        while ((als_counts <= 100) && !((gain_index == 3) && (it_index == 5))) {
            if (gain_index < 3) {
                // increase and set gain
                ESP_RETURN_ON_ERROR(i2c_veml7700_set_gain(veml7700_handle, i2c_veml7700_gains[++gain_index]), TAG, "write gain failed" );
            } else if (it_index < 5) {
                // increase and set integration time
                ESP_RETURN_ON_ERROR(i2c_veml7700_set_integration_time(veml7700_handle, i2c_veml7700_integration_times[++it_index]), TAG, "write integration time failed" );
            }
            /* read ambient light counts */
            ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light_counts(veml7700_handle, &als_counts), TAG, "read ambient light counts failed" );
        }

        ESP_LOGI(TAG, "IT     %d", veml7700_handle->config_reg.bits.integration_time);
        ESP_LOGI(TAG, "Gain   %d", veml7700_handle->config_reg.bits.gain);
    } else {
        // decrease integration time as needed
        while ((als_counts > 10000) && (it_index > 0)) {
            // decrease and set integration time
            ESP_RETURN_ON_ERROR(i2c_veml7700_set_integration_time(veml7700_handle, i2c_veml7700_integration_times[--it_index]), TAG, "write integration time failed" );

            /* read ambient light counts */
            ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light_counts(veml7700_handle, &als_counts), TAG, "read ambient light counts failed" );
        }

        ESP_LOGI(TAG, "IT     %d", veml7700_handle->config_reg.bits.integration_time);
        ESP_LOGI(TAG, "Gain   %d", veml7700_handle->config_reg.bits.gain);
    }

    return ESP_OK;
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
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy register */
    i2c_veml7700_configuration_register_t config = { .reg = config_reg.reg };

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
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy register */
    i2c_veml7700_power_saving_mode_register_t power_saving_mode = { .reg = power_saving_mode_reg.reg };

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

    // set the resolution on the configuration struct
	//out_handle->resolution  = i2c_veml7700_get_resolution(out_handle);
	// set the current maximum value on the configuration struct
	//out_handle->maximum_lux = i2c_veml7700_get_current_maximum_lux(out_handle);

    /* attempt to set initialization registers from configuration */
    i2c_veml7700_configuration_register_t       cfg_reg     = { .reg = out_handle->config_reg.reg };
    i2c_veml7700_power_saving_mode_register_t   psm_reg     = { .reg = out_handle->power_saving_mode_reg.reg };
    
    /* set configuration register */
    cfg_reg.bits.gain                   = veml7700_config->gain;
    cfg_reg.bits.integration_time       = veml7700_config->integration_time;
    cfg_reg.bits.persistence_protect    = veml7700_config->persistence_protect;
    cfg_reg.bits.irq_enabled            = veml7700_config->irq_enabled;
    cfg_reg.bits.shutdown               = !veml7700_config->power_enabled;

    /* set power saving register */
    psm_reg.bits.power_saving_enabled   = veml7700_config->power_saving_enabled;
    psm_reg.bits.power_saving_mode      = veml7700_config->power_saving_mode;

    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_veml7700_set_configuration_register(out_handle, cfg_reg), err_handle, TAG, "write configuration register failed");

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

esp_err_t i2c_veml7700_get_ambient_light_counts(i2c_veml7700_handle_t veml7700_handle, uint16_t *const counts) {
    uint16_t als_counts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && counts);

    /* handle measurement refresh time */
    if(veml7700_handle->power_saving_mode_reg.bits.power_saving_enabled) {
        int psm_index = i2c_veml7700_get_psm_mode_index(veml7700_handle->power_saving_mode_reg.bits.power_saving_mode, veml7700_handle->config_reg.bits.integration_time);
        vTaskDelay(pdMS_TO_TICKS(i2c_veml7700_psm_refresh_time_map[psm_index][3]));
    } else {
        int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);
        vTaskDelay(pdMS_TO_TICKS(i2c_veml7700_integration_time_map[it_index][1]));
    }
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_ALS, &als_counts), TAG, "read ambient light counts failed" );

    /* set output parameter */
    *counts = als_counts;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_ambient_light(i2c_veml7700_handle_t veml7700_handle, float *const ambient_light) {
    uint16_t als_counts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && ambient_light);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light_counts(veml7700_handle, &als_counts), TAG, "read ambient light counts failed" );

    /* apply resolution correction */
    float comp_lux = (float)(als_counts) * i2c_veml7700_get_resolution(veml7700_handle);

    /* apply correction formula for illumination > 1000 lux */
    if(comp_lux > 1000) {
        /* polynomial correction and set output parameter */
        *ambient_light = (I2C_VEML7700_POLY_COEF_A * powf(comp_lux, 4)) + (I2C_VEML7700_POLY_COEF_B * powf(comp_lux, 3)) + (I2C_VEML7700_POLY_COEF_C * powf(comp_lux, 2)) + (I2C_VEML7700_POLY_COEF_D * comp_lux);
    } else {
        /* set output parameter */
        *ambient_light = comp_lux;
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_ambient_light_auto(i2c_veml7700_handle_t veml7700_handle, float *const ambient_light) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && ambient_light);

    // calculate and automatically reconfigure the optimal sensor configuration
    ESP_RETURN_ON_ERROR( i2c_veml7700_optimize_configuration(veml7700_handle), TAG, "optimize configuration for read ambient light auto failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_ambient_light(veml7700_handle, ambient_light), TAG, "read ambient light auto failed" );

	//ESP_LOGD(TAG, "Configured maximum luminocity: %" PRIu32 "", veml7700_handle->maximum_lux);
	//ESP_LOGD(TAG, "Configured resolution: %0.4f", veml7700_handle->resolution);

	return ESP_OK;
}

esp_err_t i2c_veml7700_get_white_channel_counts(i2c_veml7700_handle_t veml7700_handle, uint16_t *const counts) {
    uint16_t als_counts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && counts);

    /* handle measurement refresh time */
    if(veml7700_handle->power_saving_mode_reg.bits.power_saving_enabled) {
        int psm_index = i2c_veml7700_get_psm_mode_index(veml7700_handle->power_saving_mode_reg.bits.power_saving_mode, veml7700_handle->config_reg.bits.integration_time);
        vTaskDelay(pdMS_TO_TICKS(i2c_veml7700_psm_refresh_time_map[psm_index][3]));
    } else {
        int it_index = i2c_veml7700_get_it_index(veml7700_handle->config_reg.bits.integration_time);
        vTaskDelay(pdMS_TO_TICKS(i2c_veml7700_integration_time_map[it_index][1]));
    }
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(veml7700_handle->i2c_dev_handle, I2C_VEML7700_CMD_WHITE, &als_counts), TAG, "read white channel counts failed" );

    /* set output parameter */
    *counts = als_counts;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_white_channel(i2c_veml7700_handle_t veml7700_handle, float *const white_light) {
    uint16_t als_counts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && white_light);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_white_channel_counts(veml7700_handle, &als_counts), TAG, "read white channel failed" );

    /* apply resolution correction */
    float comp_lux = (float)(als_counts) * i2c_veml7700_get_resolution(veml7700_handle);

    /* apply correction formula for illumination > 1000 lux */
    if(comp_lux > 1000) {
        /* polynomial correction and set output parameter */
        *white_light = (I2C_VEML7700_POLY_COEF_A * powf(comp_lux, 4)) + (I2C_VEML7700_POLY_COEF_B * powf(comp_lux, 3)) + (I2C_VEML7700_POLY_COEF_C * powf(comp_lux, 2)) + (I2C_VEML7700_POLY_COEF_D * comp_lux);
    } else {
        /* set output parameter */
        *white_light = comp_lux;
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_veml7700_get_white_channel_auto(i2c_veml7700_handle_t veml7700_handle, float *const white_light) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle && white_light);

    // calculate and automatically reconfigure the optimal sensor configuration
    ESP_RETURN_ON_ERROR( i2c_veml7700_optimize_configuration(veml7700_handle), TAG, "optimize configuration for read white channel auto failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_white_channel(veml7700_handle, white_light), TAG, "read white channel auto failed" );

	//ESP_LOGD(TAG, "Configured maximum luminocity: %" PRIu32 "\n", veml7700_handle->maximum_lux);
	//ESP_LOGD(TAG, "Configured resolution: %0.4f\n", veml7700_handle->resolution);

	return ESP_OK;
}

esp_err_t i2c_veml7700_get_thresholds(i2c_veml7700_handle_t veml7700_handle, uint16_t *const hi_threshold, uint16_t *const lo_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_threshold_registers(veml7700_handle), TAG, "read threshold registers for get thresholds failed" );

    /* set output parameters */
    *hi_threshold = veml7700_handle->hi_threshold_reg;
    *lo_threshold = veml7700_handle->lo_threshold_reg;

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_thresholds(i2c_veml7700_handle_t veml7700_handle, const uint16_t hi_threshold, const uint16_t lo_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_threshold_registers(veml7700_handle, hi_threshold, lo_threshold), TAG, "write threshold registers for set thresholds failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_gain(i2c_veml7700_handle_t veml7700_handle, i2c_veml7700_gains_t *const gain) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_configuration_register(veml7700_handle), TAG, "read configuration register for get gain failed" );

    /* set output parameters */
    *gain = veml7700_handle->config_reg.bits.gain;

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_gain(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_gains_t gain) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* set parameters */
    config_reg.bits.gain = gain;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for set gain failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_integration_time(i2c_veml7700_handle_t veml7700_handle, i2c_veml7700_integration_times_t *const integration_time) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_configuration_register(veml7700_handle), TAG, "read configuration register for get gain failed" );

    /* set output parameters */
    *integration_time = veml7700_handle->config_reg.bits.integration_time;

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_integration_time(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_integration_times_t integration_time) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* set parameters */
    config_reg.bits.integration_time = integration_time;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for set integration time failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_persistence_protection(i2c_veml7700_handle_t veml7700_handle, i2c_veml7700_persistence_protections_t *const persistence_protection) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_configuration_register(veml7700_handle), TAG, "read configuration register for get gain failed" );

    /* set output parameters */
    *persistence_protection = veml7700_handle->config_reg.bits.persistence_protect;

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_persistence_protection(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_persistence_protections_t persistence_protection) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* set gain */
    config_reg.bits.persistence_protect = persistence_protection;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for set persistence protection failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_power_saving_mode(i2c_veml7700_handle_t veml7700_handle, i2c_veml7700_power_saving_modes_t *const power_saving_mode, bool *const power_saving_enabled) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_power_saving_mode_register(veml7700_handle), TAG, "read power saving mode register for get power saving mode failed" );

    /* set output parameters */
    *power_saving_mode    = veml7700_handle->power_saving_mode_reg.bits.power_saving_mode;
    *power_saving_enabled = veml7700_handle->power_saving_mode_reg.bits.power_saving_enabled;

    return ESP_OK;
}

esp_err_t i2c_veml7700_set_power_saving_mode(i2c_veml7700_handle_t veml7700_handle, const i2c_veml7700_power_saving_modes_t power_saving_mode, const bool power_saving_enabled) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_power_saving_mode_register_t pwr_mode_reg = { .reg = veml7700_handle->power_saving_mode_reg.reg };

    /* set parameters */
    pwr_mode_reg.bits.power_saving_mode    = power_saving_mode;
    pwr_mode_reg.bits.power_saving_enabled = power_saving_enabled;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_power_saving_mode_register(veml7700_handle, pwr_mode_reg), TAG, "write power saving mode register for set power saving mode failed" );

    return ESP_OK;
}

esp_err_t veml7700_get_interrupt_status(i2c_veml7700_handle_t veml7700_handle, bool *const hi_threshold_exceeded, bool *const lo_threshold_exceeded) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_get_interrupt_status_register(veml7700_handle), TAG, "read interrupt status register for interrupt status failed" );

    /* set output parameters */
    *hi_threshold_exceeded = veml7700_handle->interrupt_status_reg.bits.hi_threshold_exceeded;
    *lo_threshold_exceeded = veml7700_handle->interrupt_status_reg.bits.lo_threshold_exceeded;

    return ESP_OK;
}

esp_err_t i2c_veml7700_enable_irq(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* set parameters */
    config_reg.bits.irq_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for enable irq failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_disable_irq(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* set parameters */
    config_reg.bits.irq_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for disable irq failed" );

    return ESP_OK;
}

esp_err_t i2c_veml7700_disable_power(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* shutdown device */
    config_reg.bits.shutdown = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for shutdown failed" );

    veml7700_handle->sleeping = true;

    return ESP_OK;
}

esp_err_t i2c_veml7700_enable_power(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* copy configuration register */
    i2c_veml7700_configuration_register_t config_reg = { .reg = veml7700_handle->config_reg.reg };

    /* wakeup device */
    config_reg.bits.shutdown = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_veml7700_set_configuration_register(veml7700_handle, config_reg), TAG, "write configuration register for wake-up failed" );

    veml7700_handle->sleeping = false;

    return ESP_OK;
}

esp_err_t i2c_veml7700_remove(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    return i2c_master_bus_rm_device(veml7700_handle->i2c_dev_handle);
}

esp_err_t i2c_veml7700_delete(i2c_veml7700_handle_t veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( veml7700_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_veml7700_remove(veml7700_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(veml7700_handle->i2c_dev_handle) {
        free(veml7700_handle->i2c_dev_handle);
        free(veml7700_handle);
    }

    return ESP_OK;
}