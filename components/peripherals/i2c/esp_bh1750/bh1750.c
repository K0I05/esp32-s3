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
 * @file bh1750.c
 *
 * ESP-IDF driver for BH1750 illuminance sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bh1750.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * BH1750 definitions
*/
#define I2C_BH1750_OPCODE_MT_HI         UINT8_C(0x40)   /*!< measurement time hi-bit */
#define I2C_BH1750_OPCODE_MT_LO         UINT8_C(0x60)   /*!< measurement time lo-bit */

#define I2C_BH1750_CMD_POWER_DOWN       UINT8_C(0b00000000)
#define I2C_BH1750_CMD_POWER_UP         UINT8_C(0b00000001)
#define I2C_BH1750_CMD_RESET            UINT8_C(0b00000111)

#define I2C_BH1750_POWERUP_DELAY_MS     UINT16_C(10)
#define I2C_BH1750_APPSTART_DELAY_MS    UINT16_C(10)
#define I2C_BH1750_RESET_DELAY_MS       UINT16_C(25)
#define I2C_BH1750_CMD_DELAY_MS         UINT16_C(5)
#define I2C_BH1750_RETRY_DELAY_MS       UINT16_C(2)     /*!< bh1750 delay between an I2C receive transaction retry */
#define I2C_BH1750_TX_RX_DELAY_MS       UINT16_C(10)    /*!< bh1750 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declerations
*/
static const char *TAG = "bh1750";

/*
* functions and subrountines
*/

/**
 * @brief Gets bh1750 duration in milli-seconds from device handle.  See datasheet for details.
 *
 * @param[in] bh1750_handle bh1750 device handle 
 * @return duration in milliseconds.
 */
static inline size_t i2c_bh1750_get_duration_ms(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    if (!bh1750_handle) return 180;
    /* todo - duration when measurement time is modified */
    switch (bh1750_handle->mode) {
        case I2C_BH1750_MODE_OM_HI_RESOLUTION:
            return 180;
        case I2C_BH1750_MODE_OM_HI2_RESOLUTION:
            return 180;
        case I2C_BH1750_MODE_OM_LO_RESOLUTION:
            return 25;
        case I2C_BH1750_MODE_CM_HI_RESOLUTION:
            return 180;
        case I2C_BH1750_MODE_CM_HI2_RESOLUTION:
            return 180;
        case I2C_BH1750_MODE_CM_LO_RESOLUTION:
            return 25;
        default:
            return 180;
    }
}

/**
 * @brief Gets bh1750 tick duration from device handle.
 *
 * @param[in] bh1750_handle bh1750 device handle 
 * @return duration in ticks.
 */
static inline size_t i2c_bh1750_get_tick_duration(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    if (!bh1750_handle) return 0;
    size_t res = pdMS_TO_TICKS(i2c_bh1750_get_duration_ms(bh1750_handle));
    return res == 0 ? 1 : res;
}

esp_err_t i2c_bh1750_init(i2c_master_bus_handle_t bus_handle, const i2c_bh1750_config_t *bh1750_config, i2c_bh1750_handle_t *bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && bh1750_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, bh1750_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bh1750 device handle initialization failed", bh1750_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_bh1750_handle_t out_handle = (i2c_bh1750_handle_t)calloc(1, sizeof(i2c_bh1750_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for device, bh1750 device handle initialization failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bh1750_config->dev_config.device_address,
        .scl_speed_hz       = bh1750_config->dev_config.scl_speed_hz
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "unable to add device to master bus, bh1750 device handle initialization failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_bh1750_reset(out_handle), err_handle, TAG, "unabled to soft-reset device, bh1750 device handle initialization failed");

    /* validate power status */
    if(bh1750_config->power_enabled == true) {
        /* attempt to power up device */
        ESP_GOTO_ON_ERROR(i2c_bh1750_enable_power(out_handle), err_handle, TAG, "unable to power-up device, bh1750 device handle initialization failed");
    }

    /* validate measurement time */
    if(bh1750_config->set_timespan == true) {
        if(bh1750_config->power_enabled == false) {
            /* attempt to power up device */
            ESP_GOTO_ON_ERROR(i2c_bh1750_enable_power(out_handle), err_handle, TAG, "unable to power-up device, bh1750 device handle initialization failed");
        }

        /* attempt to write measurement time */
        ESP_GOTO_ON_ERROR(i2c_bh1750_set_measurement_time(out_handle, bh1750_config->timespan), err_handle, TAG, "unable to write measurement time to device, bh1750 device handle initialization failed");
    }

    /* attempt to write measurement mode */
    ESP_GOTO_ON_ERROR(i2c_bh1750_set_measurement_mode(out_handle, bh1750_config->mode), err_handle, TAG, "unable to write measurement mode to device, bh1750 device handle initialization failed");

    /* set device handle */
    *bh1750_handle = out_handle;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_bh1750_get_ambient_light(i2c_bh1750_handle_t bh1750_handle, float *const ambient_light) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    size_t delay_ticks          = 0;
    esp_err_t ret               = ESP_OK;
    i2c_uint8_t i2c_tx_buffer   = { 0 };
    i2c_uint16_t i2c_rx_buffer  = { 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );
    
    /* set command and delay */
    i2c_tx_buffer[0] = bh1750_handle->mode;
    delay_ticks = i2c_bh1750_get_tick_duration(bh1750_handle);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_transmit(bh1750_handle->i2c_dev_handle, i2c_tx_buffer, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write measurement mode command to device, get measurement failed");

    /* delay task - allow time for the sensor to process measurement request */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(bh1750_handle->i2c_dev_handle, i2c_rx_buffer, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, get measurement failed" );

    /* convert bh1750 results to engineering units of measure (lux) */
    uint16_t val = i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1];
    *ambient_light = (float)(val * 10) / 12;

    /* set handle power status */
     if(bh1750_handle->mode == I2C_BH1750_MODE_OM_HI_RESOLUTION ||
        bh1750_handle->mode == I2C_BH1750_MODE_OM_HI2_RESOLUTION ||
        bh1750_handle->mode == I2C_BH1750_MODE_OM_LO_RESOLUTION) ESP_RETURN_ON_ERROR(i2c_bh1750_disable_power(bh1750_handle), TAG, "disable power failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_set_measurement_mode(i2c_bh1750_handle_t bh1750_handle, const i2c_bh1750_measurement_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, mode), TAG, "write measurement mode command failed");

    /* set handle measurement mode parameter */
    bh1750_handle->mode = mode;

    ESP_LOGD(TAG, "i2c_bh1750_set_measurement_mode (VAL = 0x%02x)", mode);

    /* set handle power status */
     if(bh1750_handle->mode == I2C_BH1750_MODE_OM_HI_RESOLUTION ||
        bh1750_handle->mode == I2C_BH1750_MODE_OM_HI2_RESOLUTION ||
        bh1750_handle->mode == I2C_BH1750_MODE_OM_LO_RESOLUTION) ESP_RETURN_ON_ERROR(i2c_bh1750_disable_power(bh1750_handle), TAG, "disable power failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_set_measurement_time(i2c_bh1750_handle_t bh1750_handle, const uint8_t timespan) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* validate timespan */
    if(timespan < 31 || timespan > 254) return ESP_ERR_INVALID_ARG;

    /* attempt to write measurement hi and lo timespan */
    ESP_ERROR_CHECK( i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_OPCODE_MT_HI | (timespan >> 5)) );
    ESP_ERROR_CHECK( i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_OPCODE_MT_LO | (timespan >> 0x1f)) );

    /* set handle measurement timespan parameter */
    bh1750_handle->timespan = timespan;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_enable_power(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_POWER_UP), TAG, "write power-up command failed");

    /* delay before next command - power cycle */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_POWERUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_disable_power(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_POWER_DOWN), TAG, "write power-down command failed");

    /* delay before next command - power cycle */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_POWERUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_reset(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_RESET), TAG, "write soft-reset command failed");

    /* delay before next command - power cycle */
    vTaskDelay(pdMS_TO_TICKS(I2C_BH1750_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bh1750_remove(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    return i2c_master_bus_rm_device(bh1750_handle->i2c_dev_handle);
}

esp_err_t i2c_bh1750_delete(i2c_bh1750_handle_t bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bh1750_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_bh1750_remove(bh1750_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(bh1750_handle->i2c_dev_handle) {
        free(bh1750_handle->i2c_dev_handle);
        free(bh1750_handle);
    }

    return ESP_OK;
}