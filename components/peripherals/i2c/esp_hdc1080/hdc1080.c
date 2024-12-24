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
 * @file hdc1080.c
 *
 * ESP-IDF driver for HDC1080 temperature and humdity sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "hdc1080.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * HDC1080 definitions
*/

#define I2C_HDC1080_REG_TEMPERATURE         UINT8_C(0x00)   //!< hdc1080 I2C temperature measurement output 
#define I2C_HDC1080_REG_HUMIDITY            UINT8_C(0x01)   //!< hdc1080 I2C relative humidity measurement ouput  
#define I2C_HDC1080_REG_CONFIGURATION       UINT8_C(0x02)   //!< hdc1080 I2C configuration and status
#define I2C_HDC1080_REG_SERIAL_ID_FBP       UINT8_C(0xFB)   //!< hdc1080 I2C first 2 bytes of the serial ID of the part
#define I2C_HDC1080_REG_SERIAL_ID_MBP       UINT8_C(0xFC)   //!< hdc1080 I2C mid 2 bytes of the serial ID of the part
#define I2C_HDC1080_REG_SERIAL_ID_LBP       UINT8_C(0xFD)   //!< hdc1080 I2C last byte bit of the serial ID of the part 
#define I2C_HDC1080_REG_MANUFACTURER_ID     UINT8_C(0xFE)   //!< hdc1080 I2C ID of Texas Instruments
#define I2C_HDC1080_REG_DEVICE_ID           UINT8_C(0xFF)   //!< hdc1080 I2C ID of the device

#define I2C_HDC1080_MANUFACTURER_ID         UINT16_C(0x5449)
#define I2C_HDC1080_DEVICE_ID               UINT16_C(0x1050)

#define I2C_HDC1080_TEMPERATURE_MAX         (float)(125.0)  //!< hdc1080 maximum temperature range
#define I2C_HDC1080_TEMPERATURE_MIN         (float)(-40.0)  //!< hdc1080 minimum temperature range
#define I2C_HDC1080_HUMIDITY_MAX            (float)(100.0)  //!< hdc1080 maximum humidity range
#define I2C_HDC1080_HUMIDITY_MIN            (float)(0.0)    //!< hdc1080 minimum humidity range

#define I2C_HDC1080_POWERUP_DELAY_MS        UINT16_C(30)    //!< hdc1080 I2C power-up delay in milliseconds
#define I2C_HDC1080_APPSTART_DELAY_MS       UINT16_C(15)    //!< hdc1080 I2C application start delay in milliseconds
#define I2C_HDC1080_RESET_DELAY_MS          UINT16_C(20)
#define I2C_HDC1080_CMD_DELAY_MS            UINT16_C(5)
#define I2C_HDC1080_RETRY_DELAY_MS          UINT16_C(2)

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "hdc1080";

/*
* functions and subrountines
*/

/**
 * @brief Gets HDC1080 millisecond duration from humidity measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 humidity measurement resolution.
 * @return size_t Measurement duration in milliseconds.
 */
static inline size_t i2c_hdc1080_get_humidity_duration_ms(const i2c_hdc1080_humidity_resolutions_t resolution) {
    switch (resolution) {
        case I2C_HDC1080_HUMIDITY_RESOLUTION_14BIT:
            return 7;
        case I2C_HDC1080_HUMIDITY_RESOLUTION_11BIT:
            return 4;
        case I2C_HDC1080_HUMIDITY_RESOLUTION_8BIT:
            return 3;
        default:
            return 7;
    }
}

/**
 * @brief Gets HDC1080 tick duration from humidity measurement resolution.
 *
 * @param[in] resolution HDC1080 humidity measurement resolution.
 * @return size_t Measurement duration in ticks.
 */
static inline size_t i2c_hdc1080_get_humidity_tick_duration(const i2c_hdc1080_humidity_resolutions_t resolution) {
    size_t res = pdMS_TO_TICKS(i2c_hdc1080_get_humidity_duration_ms(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets HDC1080 millisecond duration from temperature measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 temperature measurement resolution.
 * @return size_t Measurement duration in milliseconds.
 */
static inline size_t i2c_hdc1080_get_temperature_duration_ms(const i2c_hdc1080_temperature_resolutions_t resolution) {
    switch (resolution) {
        case I2C_HDC1080_TEMPERATURE_RESOLUTION_14BIT:
            return 7;
        case I2C_HDC1080_TEMPERATURE_RESOLUTION_11BIT:
            return 4;
        default:
            return 7;
    }
}

/**
 * @brief Gets HDC1080 tick duration from temperature measurement resolution.
 *
 * @param[in] resolution HDC1080 temperature measurement resolution.
 * @return size_t Measurement duration in ticks.
 */
static inline size_t i2c_hdc1080_get_temperature_tick_duration(const i2c_hdc1080_temperature_resolutions_t resolution) {
    size_t res = pdMS_TO_TICKS(i2c_hdc1080_get_temperature_duration_ms(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature Air temperature in degrees Celsius.
 * @param[in] humidity Relative humiity in percentage.
 * @param[out] dewpoint Calculated dewpoint temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_hdc1080_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK(temperature && humidity && dewpoint);

    /* validate temperature argument */
    if(temperature > I2C_HDC1080_TEMPERATURE_MAX || temperature < I2C_HDC1080_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dewpoint failed");
    }

    /* validate humidity argument */
    if(humidity > I2C_HDC1080_HUMIDITY_MAX || humidity < I2C_HDC1080_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dewpoint failed");
    }
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

/**
 * @brief Reads unique serial number register from HDC1080.
 * 
 * @param[in] hdc1080_handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_hdc1080_get_serial_number_register(i2c_hdc1080_handle_t hdc1080_handle) {
    //uint16_t serial_id_fbp, serial_id_mbp, serial_id_lbp; // serial identifier parts
    //i2c_hdc1080_serial_number_register_t sn_reg; // this structure may not be needed
    // hdc1080_handle->dev_params->serial_number

    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_CMD_DELAY_MS) );

    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Reads manufacturer identifier register from HDC1080.
 * 
 * @param[in] hdc1080_handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_hdc1080_get_manufacturer_id_register(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* attempt to read manufacturer identifier */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_MANUFACTURER_ID, &hdc1080_handle->manufacturer_id), TAG, "read manufacturer identifier failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_CMD_DELAY_MS) );

    return ESP_OK;
}

/**
 * @brief Reads device identifier register from HDC1080.
 * 
 * @param[in] hdc1080_handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_hdc1080_get_device_id_register(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* attempt to read device identifier */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_DEVICE_ID, &hdc1080_handle->device_id), TAG, "read device identifier failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_CMD_DELAY_MS) );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_get_configuration_register(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* attempt to read configuration register */
    uint16_t config;
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_CONFIGURATION, &config), TAG, "read configuration register failed" );

    /* set handle */
    hdc1080_handle->config_reg.reg = config;

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_CMD_DELAY_MS) );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_set_configuration_register(i2c_hdc1080_handle_t hdc1080_handle, const i2c_hdc1080_configuration_register_t config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register */
    i2c_hdc1080_configuration_register_t config = { .reg = config_reg.reg };

    /* set configuration reserved fields to 0 */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_CONFIGURATION, config.reg), TAG, "write configuration register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_CMD_DELAY_MS) );

    /* attempt to set device handle configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_get_configuration_register(hdc1080_handle), TAG, "Read configuration register for write configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_init(i2c_master_bus_handle_t bus_handle, const i2c_hdc1080_config_t *hdc1080_config, i2c_hdc1080_handle_t *hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && hdc1080_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_HDC1080_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, hdc1080_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, hdc1080 device handle initialization failed", hdc1080_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_hdc1080_handle_t out_handle = (i2c_hdc1080_handle_t)calloc(1, sizeof(i2c_hdc1080_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hdc1080 device");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = hdc1080_config->dev_config.device_address,
        .scl_speed_hz       = hdc1080_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_hdc1080_reset(out_handle), err_handle, TAG, "i2c hdc1080 soft-reset device failed");

    /* attempt to read configuration register */
    ESP_GOTO_ON_ERROR(i2c_hdc1080_get_configuration_register(out_handle), err_handle, TAG, "i2c hdc1080 read configuration register failed");

    /* attempt to read manufacturer identifier */
    ESP_GOTO_ON_ERROR(i2c_hdc1080_get_manufacturer_id_register(out_handle), err_handle, TAG, "i2c hdc1080 read manufacturer identifer failed");

    /* attempt to read device identifier */
    ESP_GOTO_ON_ERROR(i2c_hdc1080_get_device_id_register(out_handle), err_handle, TAG, "i2c hdc1080 read device identifer failed");

    /* attempt to read device serial number */

    /* configure device */
    i2c_hdc1080_configuration_register_t config = { .reg = out_handle->config_reg.reg };
    config.bits.acquisition_mode       = I2C_HDC1080_ACQUISITION_SEQUENCED;
    config.bits.temperature_resolution = hdc1080_config->temperature_resolution;
    config.bits.humidity_resolution    = hdc1080_config->humidity_resolution;

    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_hdc1080_set_configuration_register(out_handle, config), err_handle, TAG, "i2c hdc1080 write configuration register failed");

    /* set device handle */
    *hdc1080_handle = out_handle;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_HDC1080_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_hdc1080_get_measurement(i2c_hdc1080_handle_t hdc1080_handle, float *const temperature, float *const humidity) {
    const uint8_t rx_retry_max   = 5;
    esp_err_t     ret            = ESP_OK;
    uint8_t       rx_retry_count = 0;
    i2c_uint16_t  rx             = {};

    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_cmd(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_TEMPERATURE), TAG, "unable to write to i2c device handle, write to trigger temperature measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(i2c_hdc1080_get_temperature_tick_duration(hdc1080_handle->config_reg.bits.temperature_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(hdc1080_handle->i2c_dev_handle, rx, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read temperature failed" );

    uint16_t t_raw = ((uint16_t)rx[0] << 8) | rx[1];
    *temperature = t_raw / 65536.0f * 165.0f - 40;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_cmd(hdc1080_handle->i2c_dev_handle, I2C_HDC1080_REG_HUMIDITY), TAG, "unable to write to i2c device handle, write to trigger humidity measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(i2c_hdc1080_get_humidity_tick_duration(hdc1080_handle->config_reg.bits.humidity_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    rx_retry_count = 0;
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(hdc1080_handle->i2c_dev_handle, rx, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read humidity failed" );

    uint16_t h_raw = ((uint16_t)rx[0] << 8) | rx[1];
    *humidity = h_raw / 65536.0f * 100.0f;

    return ESP_OK;
}

esp_err_t i2c_hdc1080_get_measurements(i2c_hdc1080_handle_t hdc1080_handle, float *const temperature, float *const humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_get_measurement(hdc1080_handle, temperature, humidity), TAG, "unable to read to i2c device handle, read measurements failed" );

    /* calculate dewpoint */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "unable to calculate dewpoint, read measurements failed");

    return ESP_OK;
}

esp_err_t i2c_hdc1080_enable_heater(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register from handle */
    i2c_hdc1080_configuration_register_t config_reg = { .reg = hdc1080_handle->config_reg.reg };

    /* set heater state */
    config_reg.bits.heater_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_set_configuration_register(hdc1080_handle, config_reg), TAG, "unable to write to i2c handle, set configuration for enable heater failed" );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_disable_heater(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register from handle */
    i2c_hdc1080_configuration_register_t config_reg = { .reg = hdc1080_handle->config_reg.reg };

    /* set heater state */
    config_reg.bits.heater_enabled = false;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_set_configuration_register(hdc1080_handle, config_reg), TAG, "unable to write to i2c handle, set configuration for disable heater failed" );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_set_temperature_resolution(i2c_hdc1080_handle_t hdc1080_handle, const i2c_hdc1080_temperature_resolutions_t resolution) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register from handle */
    i2c_hdc1080_configuration_register_t config_reg = { .reg = hdc1080_handle->config_reg.reg };

    /* set temperature resolution */
    config_reg.bits.temperature_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_set_configuration_register(hdc1080_handle, config_reg), TAG, "unable to write to i2c handle, set configuration for set temperature resolution failed" );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_set_humidity_resolution(i2c_hdc1080_handle_t hdc1080_handle, const i2c_hdc1080_humidity_resolutions_t resolution) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register from handle */
    i2c_hdc1080_configuration_register_t config_reg = { .reg = hdc1080_handle->config_reg.reg };

    /* set humidity resolution */
    config_reg.bits.humidity_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_set_configuration_register(hdc1080_handle, config_reg), TAG, "unable to write to i2c handle, set configuration for set humidity resolution failed" );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_reset(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* copy register from handle */
    i2c_hdc1080_configuration_register_t config_reg = { .reg = hdc1080_handle->config_reg.reg };

    /* set soft-reset bit */
    config_reg.bits.reset_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_set_configuration_register(hdc1080_handle, config_reg), TAG, "unable to write to i2c handle, set configuration for reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(I2C_HDC1080_RESET_DELAY_MS) );

    return ESP_OK;
}

esp_err_t i2c_hdc1080_remove(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    return i2c_master_bus_rm_device(hdc1080_handle->i2c_dev_handle);
}

esp_err_t i2c_hdc1080_delete(i2c_hdc1080_handle_t hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hdc1080_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_hdc1080_remove(hdc1080_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(hdc1080_handle->i2c_dev_handle) {
        free(hdc1080_handle->i2c_dev_handle);
        free(hdc1080_handle);
    }

    return ESP_OK;
}