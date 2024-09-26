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
 * @file ahtxx.c
 *
 * ESP-IDF driver for AHTXX temperature and humidity sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ahtxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "ahtxx";

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humiity in percent.
 * @param[out] dewpoint calculated dewpoint temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ahtxx_calculate_dewpoint(const float temperature, const float humidity, float *dewpoint) {
    ESP_ARG_CHECK(temperature && humidity && dewpoint);

    // validate parameters
    if(temperature > 80 || temperature < -40) return ESP_ERR_INVALID_ARG;
    if(humidity > 100 || humidity < 0) return ESP_ERR_INVALID_ARG;
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

esp_err_t i2c_ahtxx_setup(i2c_ahtxx_handle_t ahtxx_handle) {
    i2c_uint24_t tx = { 0, I2C_AHTXX_CTRL_CALI, I2C_AHTXX_CTRL_NOP };;

    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    if(ahtxx_handle->dev_params->aht_type == I2C_AHTXX_AHT2X) {
        tx[0] = I2C_AHTXX_CMD_AHT2X_INIT;
    } else {
        tx[0] = I2C_AHTXX_CMD_AHT10_INIT;
    }

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ahtxx_handle->i2c_dev_handle, tx, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write initializaion register 0xbe failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_SETUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_status_register(i2c_ahtxx_handle_t ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ahtxx_handle->i2c_dev_handle, I2C_AHTXX_CMD_STATUS, &ahtxx_handle->dev_params->status_reg.reg), TAG, "read status register failed" );

    return ESP_OK;
}

esp_err_t i2c_ahtxx_init(i2c_master_bus_handle_t bus_handle, const i2c_ahtxx_config_t *ahtxx_config, i2c_ahtxx_handle_t *ahtxx_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_ahtxx_handle_t  out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ahtxx_config );

    /* validate memory availability for handle */
    out_handle = (i2c_ahtxx_handle_t)calloc(1, sizeof(i2c_ahtxx_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ahtxx device, init failed");

    /* validate memory availability for handle parameters */
    out_handle->dev_params = (i2c_ahtxx_params_t*)calloc(1, sizeof(i2c_ahtxx_params_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ahtxx device configuration parameters, init failed");

    /* set i2c device configuration */
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ahtxx_config->dev_config.device_address,
        .scl_speed_hz       = I2C_AHTXX_DATA_RATE_HZ,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus for init failed");
    }

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_POWERUP_DELAY_MS));

    /* attempt soft-reset */
    ESP_GOTO_ON_ERROR(i2c_ahtxx_reset(out_handle), err, TAG, "soft-reset for init failed");

    /* set device handle */
    *ahtxx_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_ahtxx_get_measurement(i2c_ahtxx_handle_t ahtxx_handle, float *temperature, float *humidity) {
    esp_err_t    ret        = ESP_OK;
    uint64_t     start_time = 0;
    bool         is_busy    = true;
    uint32_t     temp       = 0;
    i2c_uint24_t tx         = { I2C_AHTXX_CMD_TRIGGER_MEAS, I2C_AHTXX_CTRL_MEAS, I2C_AHTXX_CTRL_NOP };
    i2c_uint48_t rx         = { 0, 0, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ahtxx_handle->i2c_dev_handle, tx, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write measurement trigger command for get measurement failed" );

    /* attempt to poll status until data is available or timeout occurs */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_ahtxx_get_busy_status(ahtxx_handle, &is_busy), err, TAG, "is busy read for get measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (I2C_AHTXX_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (is_busy == true);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(ahtxx_handle->i2c_dev_handle, rx, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "read measurement data for get measurement failed" );

    /* compute and set humidity */
    temp = rx[1];
    temp <<= 8;
    temp |= rx[2];
    temp <<= 4;
    temp = temp | (rx[3] >> 4);
    *humidity = (float)(temp * 100) / (float)0x100000;

    /* compute and set temperature */
    temp = rx[3] & 0x0F;
    temp <<= 8;
    temp |= rx[4];
    temp <<= 8;
    temp |= rx[5];
    *temperature = (float)(temp * 200) / (float)0x100000 - 50;
    
    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ahtxx_get_measurements(i2c_ahtxx_handle_t ahtxx_handle, float *temperature, float *humidity, float *dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle && temperature && humidity && dewpoint );

    /* attempt to get temperature and humidity measurements */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_measurement(ahtxx_handle, temperature, humidity), TAG, "read measurement for get measurements failed" );

    /* compute dewpoint from temperature and humidity */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "calculate dew-point for get measurements failed" );

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_busy_status(i2c_ahtxx_handle_t ahtxx_handle, bool *busy) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for busy status failed" );

    /* set status */
    *busy = ahtxx_handle->dev_params->status_reg.bits.busy;

    //ESP_LOGD(TAG, "aht2x busy state    %s", busy ? "true" : "false");

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_calibration_status(i2c_ahtxx_handle_t ahtxx_handle, bool *calibrated) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for calibration status failed" );

    /* set status */
    *calibrated = ahtxx_handle->dev_params->status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_status(i2c_ahtxx_handle_t ahtxx_handle, bool *busy, bool *calibrated) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for status failed" );

    /* set status */
    *busy       = ahtxx_handle->dev_params->status_reg.bits.busy;
    *calibrated = ahtxx_handle->dev_params->status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t i2c_ahtxx_reset(i2c_ahtxx_handle_t ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_cmd(ahtxx_handle->i2c_dev_handle, I2C_AHTXX_CMD_RESET), TAG, "write reset command failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_RESET_DELAY_MS));

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR(i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for reset failed");

    /* validate status register */
    if(ahtxx_handle->dev_params->status_reg.reg != I2C_AHTXX_STATUS_WORD) {
        /* attempt to write init command */
        ESP_RETURN_ON_ERROR(i2c_ahtxx_setup(ahtxx_handle), TAG, "setup failed");

        /* attempt to read device status register */
        ESP_RETURN_ON_ERROR(i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for reset failed");
    }
    
    return ESP_OK;
}

esp_err_t i2c_ahtxx_rm(i2c_ahtxx_handle_t ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(ahtxx_handle->i2c_dev_handle);
}