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
 * @file sht4x.c
 *
 * ESP-IDF driver for SHT4x air temperature and relative humidity sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "sht4x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
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
static const char *TAG = "sht4x";

/*
* functions and subrountines
*/

/**
 * @brief Calculates sht4x crc8 value.  See datasheet for details.
 *
 * @param[in] data[] data buffer to perform crc8 check against.
 * @param[in] len length of `data` buffer.
 * @return crc8 value.
 */
static inline uint8_t i2c_sht4x_crc8(const uint8_t data[], const size_t len) {
    uint8_t crc = 0xff;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ I2C_SHT4X_CRC8_G_POLYNOM : crc << 1;
    }
    return crc;
}

/**
 * @brief Gets sht4x millisecond duration from device handle.  See datasheet for details.
 *
 * @param[in] sht4x_handle sht4x device handle.
 * @return duration in milliseconds.
 */
static inline size_t i2c_sht4x_get_ms_duration(i2c_sht4x_handle_t sht4x_handle) {
    switch (sht4x_handle->dev_params->heater) {
        case I2C_SHT4X_HEATER_HIGH_LONG:
        case I2C_SHT4X_HEATER_MEDIUM_LONG:
        case I2C_SHT4X_HEATER_LOW_LONG:
            return 1100;
        case I2C_SHT4X_HEATER_HIGH_SHORT:
        case I2C_SHT4X_HEATER_MEDIUM_SHORT:
        case I2C_SHT4X_HEATER_LOW_SHORT:
            return 110;
        default:
            switch (sht4x_handle->dev_params->repeatability) {
                case I2C_SHT4X_REPEAT_HIGH:
                    return 10;
                case I2C_SHT4X_REPEAT_MEDIUM:
                    return 5;
                default:
                    return 2;
            }
    }
}

/**
 * @brief Gets sht4x tick duration from device handle.
 *
 * @param[in] sht4x_handle sht4x device handle.
 * @return duration in ticks.
 */
static inline size_t i2c_sht4x_get_tick_duration(i2c_sht4x_handle_t sht4x_handle) {
    if (!sht4x_handle) return 0;
    size_t res = pdMS_TO_TICKS(i2c_sht4x_get_ms_duration(sht4x_handle));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets sht4x measurement command from device handle.  See datasheet for details.
 *
 * @param[in] sht4x_handle sht4x device handle.
 * @return command value.
 */
static inline uint8_t i2c_sht4x_get_command(i2c_sht4x_handle_t sht4x_handle) {
    switch (sht4x_handle->dev_params->heater) {
        case I2C_SHT4X_HEATER_HIGH_LONG:
            return I2C_SHT4X_CMD_MEAS_H_HIGH_LONG;
        case I2C_SHT4X_HEATER_HIGH_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_HIGH_SHORT;
        case I2C_SHT4X_HEATER_MEDIUM_LONG:
            return I2C_SHT4X_CMD_MEAS_H_MED_LONG;
        case I2C_SHT4X_HEATER_MEDIUM_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_MED_SHORT;
        case I2C_SHT4X_HEATER_LOW_LONG:
            return I2C_SHT4X_CMD_MEAS_H_LOW_LONG;
        case I2C_SHT4X_HEATER_LOW_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_LOW_SHORT;
        default:
            switch (sht4x_handle->dev_params->repeatability) {
                case I2C_SHT4X_REPEAT_HIGH:
                    return I2C_SHT4X_CMD_MEAS_HIGH;
                case I2C_SHT4X_REPEAT_MEDIUM:
                    return I2C_SHT4X_CMD_MEAS_MED;
                default:
                    return I2C_SHT4X_CMD_MEAS_LOW;
            }
    }
}

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humiity in percent.
 * @param[out] dewpoint calculated dewpoint temperature in degrees Celsius.
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_sht4x_calculate_dewpoint(const float temperature, const float humidity, float *dewpoint) {
    ESP_ARG_CHECK(temperature && humidity && dewpoint);

    // validate parameters
    if(temperature > 80 || temperature < -40) return ESP_ERR_INVALID_ARG;
    if(humidity > 100 || humidity < 0) return ESP_ERR_INVALID_ARG;
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

/**
 * @brief read serial number from sensor
 *
 * @param[in] sht4x_config configuration of sht4x device
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_sht4x_get_serial_number(i2c_sht4x_handle_t sht4x_handle) {
    ESP_ARG_CHECK( sht4x_handle );

    return i2c_master_bus_read_uint32(sht4x_handle->i2c_dev_handle, I2C_SHT4X_CMD_SERIAL, &sht4x_handle->dev_params->serial_number);
}

esp_err_t i2c_sht4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sht4x_config_t *sht4x_config, i2c_sht4x_handle_t *sht4x_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_sht4x_handle_t  out_handle;

    ESP_ARG_CHECK( bus_handle && sht4x_config );

    out_handle = (i2c_sht4x_handle_t)calloc(1, sizeof(i2c_sht4x_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c sht4x device");

    out_handle->dev_params = (i2c_sht4x_params_t*)calloc(1, sizeof(i2c_sht4x_params_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c sht4x device configuration parameters");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = sht4x_config->dev_config.device_address,
        .scl_speed_hz       = I2C_SHT4X_DATA_RATE_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    /* copy configuration */
    out_handle->dev_params->heater = sht4x_config->heater;
    out_handle->dev_params->repeatability = sht4x_config->repeatability;

    /* sht4x attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_sht4x_reset(out_handle), err, TAG, "i2c sht4x soft-reset device failed");

    /* sht4x attempt to read device serial number */
    ESP_GOTO_ON_ERROR(i2c_sht4x_get_serial_number(out_handle), err, TAG, "i2c sht4x read serial number from device failed");

    /* set device handle */
    *sht4x_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_sht4x_reset(i2c_sht4x_handle_t sht4x_handle) {
    ESP_ARG_CHECK( sht4x_handle );

    return i2c_master_bus_write_cmd(sht4x_handle->i2c_dev_handle, I2C_SHT4X_CMD_RESET);
}

esp_err_t i2c_sht4x_get_measurement(i2c_sht4x_handle_t sht4x_handle, float *temperature, float *humidity) {
    esp_err_t ret;
    size_t delay_ticks = 0;
    i2c_uint8_t i2c_tx_buffer = { 0 };
    i2c_uint48_t i2c_rx_buffer = { 0, 0, 0, 0, 0, 0 };

    ESP_ARG_CHECK( sht4x_handle && temperature && humidity );
    
    i2c_tx_buffer[0] = i2c_sht4x_get_command(sht4x_handle);
    delay_ticks      = i2c_sht4x_get_tick_duration(sht4x_handle);

    do {
        ret = i2c_master_transmit(sht4x_handle->i2c_dev_handle, i2c_tx_buffer, I2C_UINT8_SIZE, -1);
    } while (ret == ESP_ERR_TIMEOUT);
    if (ret == ESP_OK) {
        if(delay_ticks) vTaskDelay(delay_ticks);
        do {
            // sht4x read response
            ret = i2c_master_receive(sht4x_handle->i2c_dev_handle, i2c_rx_buffer, I2C_UINT48_SIZE, -1); 
        } while (ret == ESP_ERR_TIMEOUT);
        if (ret == ESP_OK) {
            // convert sht4x results to engineering units of measure (C and %)
            if (i2c_rx_buffer[2] != i2c_sht4x_crc8(i2c_rx_buffer, 2) || i2c_rx_buffer[5] != i2c_sht4x_crc8(i2c_rx_buffer + 3, 2)) {
                return ESP_ERR_INVALID_CRC;
            } else {
                *temperature = ((uint16_t)i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1]) * 175.0 / 65535.0 - 45.0;
                *humidity    = ((uint16_t)i2c_rx_buffer[3] << 8 | i2c_rx_buffer[4]) * 125.0 / 65535.0 - 6.0;
            }
        } else {
            return ret;
        }
    } else {
        return ret;
    }
    return ESP_OK;
}

esp_err_t i2c_sht4x_get_measurements(i2c_sht4x_handle_t sht4x_handle, float *temperature, float *humidity, float *dewpoint) {
    ESP_ARG_CHECK( sht4x_handle && temperature && humidity && dewpoint );

    ESP_ERROR_CHECK( i2c_sht4x_get_measurement(sht4x_handle, temperature, humidity) );

    ESP_ERROR_CHECK( i2c_sht4x_calculate_dewpoint(*temperature, *humidity, dewpoint) );

    return ESP_OK;
}

esp_err_t i2c_sht4x_rm(i2c_sht4x_handle_t sht4x_handle) {
    ESP_ARG_CHECK( sht4x_handle );

    return i2c_master_bus_rm_device(sht4x_handle->i2c_dev_handle);
}