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
static const char *TAG = "bh1750";

/*
* functions and subrountines
*/

/**
 * @brief Gets bh1750 millisecond duration from device handle.  See datasheet for details.
 *
 * @param[in] bh1750_handle bh1750 device handle 
 * @return duration in milliseconds.
 */
static inline size_t i2c_bh1750_get_ms_duration(i2c_bh1750_handle_t bh1750_handle) {
    switch (bh1750_handle->dev_params->resolution) {
        case I2C_BH1750_RES_LOW:
            return 16;
        case I2C_BH1750_RES_HIGH:
            return 120;
        case I2C_BH1750_RES_HIGH2:
            return 180;
        default:
            return 120;
    }
}

/**
 * @brief Gets bh1750 tick duration from device handle.
 *
 * @param[in] bh1750_handle bh1750 device handle 
 * @return duration in ticks.
 */
static inline size_t i2c_bh1750_get_tick_duration(i2c_bh1750_handle_t bh1750_handle) {
    if (!bh1750_handle) return 0;
    size_t res = pdMS_TO_TICKS(i2c_bh1750_get_ms_duration(bh1750_handle));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets bh1750 measurement command (opcode) from device handle.  See datasheet for details.
 *
 * @param[in] i2c_bh1750_handle_t bh1750 device handle 
 * @return command value.
 */
static inline uint8_t i2c_bh1750_get_command(i2c_bh1750_handle_t bh1750_handle) {
    if(bh1750_handle->dev_params->mode == I2C_BH1750_MODE_CONTINUOUS) {
        switch (bh1750_handle->dev_params->resolution) {
            case I2C_BH1750_RES_LOW:
                return I2C_BH1750_CMD_MEAS_CM_LOW;
            case I2C_BH1750_RES_HIGH:
                return I2C_BH1750_CMD_MEAS_CM_HIGH;
            case I2C_BH1750_RES_HIGH2:
                return I2C_BH1750_CMD_MEAS_CM2_HIGH;
            default:
                return I2C_BH1750_CMD_MEAS_CM_LOW;
        }
    } else {
        switch (bh1750_handle->dev_params->resolution) {
            case I2C_BH1750_RES_LOW:
                return I2C_BH1750_CMD_MEAS_OM_LOW;
            case I2C_BH1750_RES_HIGH:
                return I2C_BH1750_CMD_MEAS_OM_HIGH;
            case I2C_BH1750_RES_HIGH2:
                return I2C_BH1750_CMD_MEAS_OM2_HIGH;
            default:
                return I2C_BH1750_CMD_MEAS_OM_LOW;
        }
    }
}

/**
 * @brief configures bh1750 sensor measurement mode and resolution
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_bh1750_setup(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    const i2c_bh1750_modes_t mode = bh1750_handle->dev_params->mode;
    const i2c_bh1750_resolutions_t resolution = bh1750_handle->dev_params->resolution;

    uint8_t opcode = mode == I2C_BH1750_MODE_CONTINUOUS ? I2C_BH1750_OPCODE_CONT : I2C_BH1750_OPCODE_OT;

    switch (resolution) {
        case I2C_BH1750_RES_LOW:    opcode |= I2C_BH1750_OPCODE_LOW;   break;
        case I2C_BH1750_RES_HIGH:   opcode |= I2C_BH1750_OPCODE_HIGH;  break;
        default:                    opcode |= I2C_BH1750_OPCODE_HIGH2; break;
    }

    ESP_ERROR_CHECK( i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, opcode) );

    ESP_LOGD(TAG, "i2c_bh1750_setup(VAL = 0x%02x)", opcode);

    return ESP_OK;
}

/**
 * @brief sets bh1750 sensor measurement time. see datasheet for details.
 *
 * @param[in] bh1750_handle bh1750 device handle
 * @return ESP_OK: init success.
 */
/*
static inline esp_err_t i2c_bh1750_write_measurement_time(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    const uint8_t time = bh1750_handle->dev_params->measurement_time;

    ESP_ERROR_CHECK( i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_OPCODE_MT_HI | (time >> 5)) );
    ESP_ERROR_CHECK( i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_OPCODE_MT_LO | (time >> 0x1f)) );

    return ESP_OK;
}
*/

esp_err_t i2c_bh1750_init(i2c_master_bus_handle_t bus_handle, const i2c_bh1750_config_t *bh1750_config, i2c_bh1750_handle_t *bh1750_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_bh1750_handle_t out_handle;

    ESP_ARG_CHECK( bus_handle && bh1750_config );

    out_handle = (i2c_bh1750_handle_t)calloc(1, sizeof(i2c_bh1750_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c bh1750 device");

    out_handle->dev_params = (i2c_bh1750_params_t*)calloc(1, sizeof(i2c_bh1750_params_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c bh1750 device configuration parameters");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bh1750_config->dev_config.device_address,
        .scl_speed_hz       = I2C_BH1750_DATA_RATE_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    /* copy configuration */
    out_handle->dev_params->mode = bh1750_config->mode;
    out_handle->dev_params->resolution = bh1750_config->resolution;

    /* bh1750 attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_bh1750_reset(out_handle), err, TAG, "i2c bh1750 soft-reset device failed");

    /* bh1750 attempt to configure the device */
    ESP_GOTO_ON_ERROR(i2c_bh1750_setup(out_handle), err, TAG, "i2c bh1750 device configuration failed");

    /* set device handle */
    *bh1750_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_bh1750_reset(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    esp_err_t ret = i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_RESET);

    /* delay before next command - power cycle */
    if(ret == ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));

    return ret;
}

esp_err_t i2c_bh1750_power_up(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    esp_err_t ret =  i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_POWER_UP);

    /* delay before next command - power cycle */
    if(ret == ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));

    return ret;
}

esp_err_t i2c_bh1750_power_down(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    esp_err_t ret = i2c_master_bus_write_cmd(bh1750_handle->i2c_dev_handle, I2C_BH1750_CMD_POWER_DOWN);

    /* delay before next command - power cycle */
    if(ret == ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));

    return ret;
}

esp_err_t i2c_bh1750_get_measurement(i2c_bh1750_handle_t bh1750_handle, float *lux) {
    esp_err_t ret               = ESP_OK;
    size_t delay_ticks          = 0;
    i2c_uint8_t i2c_tx_buffer   = { 0 };
    i2c_uint16_t i2c_rx_buffer  = { 0, 0 };

    ESP_ARG_CHECK( bh1750_handle );
    
    i2c_tx_buffer[0] = i2c_bh1750_get_command(bh1750_handle);
    delay_ticks = i2c_bh1750_get_tick_duration(bh1750_handle);

    /* attempt i2c write transaction */
    ret = i2c_master_transmit(bh1750_handle->i2c_dev_handle, i2c_tx_buffer, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS);
  
    /* validate i2c transaction results */
    if (ret != ESP_OK) return ret;

    /* delay task */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* attempt i2c read transaction */
    ret = i2c_master_receive(bh1750_handle->i2c_dev_handle, i2c_rx_buffer, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS); 

    /* validate i2c transaction results */
    if (ret != ESP_OK) return ret;

    /* convert bh1750 results to engineering units of measure (lux) */
    *lux = i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1];
    *lux = (float)(*lux * 10) / 12;

    return ESP_OK;
}

esp_err_t i2c_bh1750_rm(i2c_bh1750_handle_t bh1750_handle) {
    ESP_ARG_CHECK( bh1750_handle );

    return i2c_master_bus_rm_device(bh1750_handle->i2c_dev_handle);
}