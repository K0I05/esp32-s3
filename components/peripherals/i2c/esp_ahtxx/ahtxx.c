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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * AHTXX definitions
 */
#define I2C_AHTXX_STATUS_WORD           UINT8_C(0x18) //!< ahtxx initialization status word (default)

#define I2C_AHTXX_CTRL_CALI             UINT8_C(0x08)
#define I2C_AHTXX_CTRL_MEAS             UINT8_C(0x33)
#define I2C_AHTXX_CTRL_NOP              UINT8_C(0x00)

#define I2C_AHTXX_DATA_POLL_TIMEOUT_MS  UINT16_C(100)
#define I2C_AHTXX_DATA_READY_DELAY_MS   UINT16_C(2)
#define I2C_AHTXX_POWERUP_DELAY_MS      UINT16_C(120)
#define I2C_AHTXX_RESET_DELAY_MS        UINT16_C(25)
#define I2C_AHTXX_SETUP_DELAY_MS        UINT16_C(15)
#define I2C_AHTXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< ahtxx delay after initialization before application start-up */
#define I2C_AHTXX_CMD_DELAY_MS          UINT16_C(5)     /*!< ahtxx delay before attempting I2C transactions after a command is issued */
#define I2C_AHTXX_TX_RX_DELAY_MS        UINT16_C(10)    /*!< ahtxx delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


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
static inline esp_err_t i2c_ahtxx_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
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
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    i2c_uint24_t tx = { 0, I2C_AHTXX_CTRL_CALI, I2C_AHTXX_CTRL_NOP };

    if(ahtxx_handle->aht_type == I2C_AHTXX_AHT2X) {
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

    i2c_uint8_t tx = { I2C_AHTXX_CMD_STATUS };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ahtxx_handle->i2c_dev_handle, tx, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, read status register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(ahtxx_handle->i2c_dev_handle, &ahtxx_handle->status_reg.reg, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ahtxx_init(i2c_master_bus_handle_t bus_handle, const i2c_ahtxx_config_t *ahtxx_config, i2c_ahtxx_handle_t *ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ahtxx_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, ahtxx_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ahtxx device handle initialization failed", ahtxx_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_ahtxx_handle_t out_handle = (i2c_ahtxx_handle_t)calloc(1, sizeof(i2c_ahtxx_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ahtxx device, init failed");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ahtxx_config->dev_config.device_address,
        .scl_speed_hz       = ahtxx_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_CMD_DELAY_MS));

    /* attempt soft-reset */
    ESP_GOTO_ON_ERROR(i2c_ahtxx_reset(out_handle), err_handle, TAG, "soft-reset for init failed");

    /* set device handle */
    *ahtxx_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_ahtxx_get_measurement(i2c_ahtxx_handle_t ahtxx_handle, float *const temperature, float *const humidity) {
    //esp_err_t    ret        = ESP_OK;
    //uint64_t     start_time = 0;
    //bool         is_busy    = true;
    uint32_t     raw       = 0;
    i2c_uint24_t tx         = { I2C_AHTXX_CMD_TRIGGER_MEAS, I2C_AHTXX_CTRL_MEAS, I2C_AHTXX_CTRL_NOP };
    i2c_uint48_t rx         = { };

    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* set start time (us) for timeout monitoring */
    //start_time = esp_timer_get_time(); 

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ahtxx_handle->i2c_dev_handle, tx, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write measurement trigger command for get measurement failed" );

    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_DATA_POLL_TIMEOUT_MS));

    /* delay before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_CMD_DELAY_MS));

    /* attempt to poll status until data is available or timeout occurs  */
    /*
        NOTE this causes problems since ESP-IDF v5.3.1, unexpected NACK but it may be hardware related
        some breakout SHT20 boards tested work with the do..while loop where others don't and generate an unexpected NACK
        the SHT10 board tested generates an unexpected NACK
    */
    //do {
        /* attempt to check if data is ready */
    //    ESP_GOTO_ON_ERROR( i2c_ahtxx_get_busy_status(ahtxx_handle, &is_busy), err, TAG, "is busy read for get measurement failed." );

        /* delay task before next i2c transaction */
    //    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_DATA_READY_DELAY_MS));

        /* validate timeout condition */
    //    if (ESP_TIMEOUT_CHECK(start_time, (I2C_AHTXX_DATA_POLL_TIMEOUT_MS * 1000)))
    //        return ESP_ERR_TIMEOUT;
    //} while (is_busy == true);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(ahtxx_handle->i2c_dev_handle, rx, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "read measurement data for get measurement failed" );

    /* compute and set humidity */
    raw = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | (rx[3] >> 4);
    *humidity = (float)(raw * 100) / (float)0x100000;

    /* compute and set temperature */
    raw = ((uint32_t)(rx[3] & 0x0f) << 16) | ((uint32_t)rx[4] << 8) | rx[5];
    *temperature = (float)(raw * 200) / (float)0x100000 - 50;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_CMD_DELAY_MS));
    
    return ESP_OK;

    //err:
    //    return ret;
}

esp_err_t i2c_ahtxx_get_measurements(i2c_ahtxx_handle_t ahtxx_handle, float *const temperature, float *const humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle && temperature && humidity && dewpoint );

    /* attempt to get temperature and humidity measurements */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_measurement(ahtxx_handle, temperature, humidity), TAG, "read measurement for get measurements failed" );

    /* compute dewpoint from temperature and humidity */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "calculate dew-point for get measurements failed" );

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_busy_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const busy) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for busy status failed" );

    /* set status */
    *busy = ahtxx_handle->status_reg.bits.busy;

    //ESP_LOGD(TAG, "aht2x busy state    %s", busy ? "true" : "false");

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_calibration_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const calibrated) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for calibration status failed" );

    /* set status */
    *calibrated = ahtxx_handle->status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t i2c_ahtxx_get_status(i2c_ahtxx_handle_t ahtxx_handle, bool *const busy, bool *const calibrated) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for status failed" );

    /* set status */
    *busy       = ahtxx_handle->status_reg.bits.busy;
    *calibrated = ahtxx_handle->status_reg.bits.calibrated;

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
    if(ahtxx_handle->status_reg.reg != I2C_AHTXX_STATUS_WORD) {
        /* attempt to write init command */
        ESP_RETURN_ON_ERROR(i2c_ahtxx_setup(ahtxx_handle), TAG, "setup failed");

        /* attempt to read device status register */
        ESP_RETURN_ON_ERROR(i2c_ahtxx_get_status_register(ahtxx_handle), TAG, "read status register for reset failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AHTXX_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t i2c_ahtxx_remove(i2c_ahtxx_handle_t ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(ahtxx_handle->i2c_dev_handle);
}

esp_err_t i2c_ahtxx_delete(i2c_ahtxx_handle_t ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ahtxx_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_ahtxx_remove(ahtxx_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(ahtxx_handle->i2c_dev_handle) {
        free(ahtxx_handle->i2c_dev_handle);
        free(ahtxx_handle);
    }

    return ESP_OK;
}