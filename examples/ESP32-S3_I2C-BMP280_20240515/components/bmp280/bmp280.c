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
 * @file bmp280.c
 *
 * ESP-IDF driver for BMP280 temperature and pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bmp280.h"
#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "bmp280";

/*
* functions and subrountines
*/

static inline void vTaskDelayMs(const uint ms) {
    const TickType_t xDelay = (ms / portTICK_PERIOD_MS);
    vTaskDelay( xDelay );
}

esp_err_t i2c_bmp280_read_cal_factors(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_cal_factors_t *cal_factors) {
    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x88, &cal_factors->dig_T1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8a, (uint16_t *)&cal_factors->dig_T2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8c, (uint16_t *)&cal_factors->dig_T3) );
    /* bmp280 attempt to request P1-P9 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8e, &cal_factors->dig_P1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x90, (uint16_t *)&cal_factors->dig_P2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x92, (uint16_t *)&cal_factors->dig_P3) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x94, (uint16_t *)&cal_factors->dig_P4) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x96, (uint16_t *)&cal_factors->dig_P5) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x98, (uint16_t *)&cal_factors->dig_P6) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9a, (uint16_t *)&cal_factors->dig_P7) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9c, (uint16_t *)&cal_factors->dig_P8) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9e, (uint16_t *)&cal_factors->dig_P9) );

    /*
    ESP_LOGI(CONFIG_APP_TAG, "Calibration data received:");
    ESP_LOGI(CONFIG_APP_TAG, "dig_T1=%u", bmp280_dev_hdl->dev_cal_factors.dig_T1);
    ESP_LOGI(CONFIG_APP_TAG, "dig_T2=%u", bmp280_dev_hdl->dev_cal_factors.dig_T2);
    ESP_LOGI(CONFIG_APP_TAG, "dig_T3=%u", bmp280_dev_hdl->dev_cal_factors.dig_T3);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P1=%u", bmp280_dev_hdl->dev_cal_factors.dig_P1);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P2=%u", bmp280_dev_hdl->dev_cal_factors.dig_P2);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P3=%u", bmp280_dev_hdl->dev_cal_factors.dig_P3);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P4=%u", bmp280_dev_hdl->dev_cal_factors.dig_P4);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P5=%u", bmp280_dev_hdl->dev_cal_factors.dig_P5);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P6=%u", bmp280_dev_hdl->dev_cal_factors.dig_P6);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P7=%u", bmp280_dev_hdl->dev_cal_factors.dig_P7);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P8=%u", bmp280_dev_hdl->dev_cal_factors.dig_P8);
    ESP_LOGI(CONFIG_APP_TAG, "dig_P9=%u", bmp280_dev_hdl->dev_cal_factors.dig_P9);
    */

    return ESP_OK;
}

esp_err_t i2c_bmp280_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp280_config_t *bmp280_config, i2c_bmp280_handle_t *bmp280_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_bmp280_cal_factors_t cal_factors;
    i2c_bmp280_handle_t out_handle;

    CHECK_ARG(bus_handle && bmp280_config);

    out_handle = (i2c_bmp280_handle_t)calloc(1, sizeof(i2c_bmp280_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bmp280_config->dev_config.device_address,
        .scl_speed_hz       = I2C_BMP280_FREQ_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c0 new bus failed");
    }

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(i2c_master_bus_read_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_ID, &out_handle->dev_type), err, TAG, "i2c0 read device type failed");
    if(out_handle->dev_type != I2C_BMP280_TYPE_BMP280) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err, TAG, "i2c0 detected an invalid device type, got: %02x", out_handle->dev_type);
    }

    /* bmp280 attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_bmp280_reset(out_handle), err, TAG, "i2c0 bmp280 soft-reset failed");

    /* wait until finished copying NVP data */
    while(1) {
        uint8_t status;
        if (!i2c_master_bus_read_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_STATUS, &status) && (status & 1) == 0)
            break;
    }

    /* bmp280 attempt to read calibration data from device */
    //ESP_GOTO_ON_ERROR(i2c_bmp280_read_cal_factors(out_handle, &cal_factors), err, TAG, "i2c0 bmp280 read calibration factors failed");
    
    /* bmp280 attempt to write configuration params to device */
    uint8_t config = (bmp280_config->dev_params.standby << 5) | (bmp280_config->dev_params.filter << 2);
    ESP_LOGD(TAG, "i2c0 bmp280 writing config reg=%02x", config);

    ESP_GOTO_ON_ERROR(i2c_master_bus_write_register8(out_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, config), err, TAG, "i2c0 bmp280 write configuration params failed");

    /* bmp280 attempt to write control params to device */
    uint8_t ctrl;
    if (bmp280_config->dev_params.mode == I2C_BMP280_MODE_FORCED) {
        // initial mode for forced is sleep
        ctrl = (bmp280_config->dev_params.oversampling_temperature << 5) | (bmp280_config->dev_params.oversampling_pressure << 2) | (I2C_BMP280_MODE_SLEEP);
    } else {
        ctrl = (bmp280_config->dev_params.oversampling_temperature << 5) | (bmp280_config->dev_params.oversampling_pressure << 2) | (bmp280_config->dev_params.mode);
    }
    ESP_LOGD(TAG, "i2c0 bmp280 writing control reg=%02x", ctrl);
    ESP_GOTO_ON_ERROR(i2c_master_bus_write_register8(out_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, ctrl), err, TAG, "i2c0 bmp280 write control params failed");


    /* copy configuration */
    //out_handle->dev_cal_factors = cal_factors;
    *bmp280_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}


esp_err_t i2c_bmp280_reset(i2c_bmp280_handle_t bmp280_handle) {
    CHECK_ARG(bmp280_handle);

    return i2c_master_bus_write_register8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_RESET, I2C_BMP280_RESET_VALUE);
}
