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
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "bmp280";

/*
* functions and subrountines
*/

/**
 * @brief converts milliseconds to ticks.
 *
 * @param[in] ms milliseconds to convert to ticks.
 * @return converted ms in ticks.
 */
static inline size_t i2c_bmp280_get_tick_duration(const uint16_t ms) {
    size_t res = pdMS_TO_TICKS(ms);

    return res == 0 ? 1 : res;
}

/**
 * @brief temperature compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[in] adc_temperature raw adc temperature.
 * @param[out] fine_temperature fine temperature.
 * @return fine temperature in degrees Celsius.
 */
static inline int32_t i2c_bmp280_compensate_temperature(i2c_bmp280_handle_t bmp280_handle, const int32_t adc_temperature, int32_t *fine_temperature) {
    int32_t var1, var2;

    var1 = ((((adc_temperature >> 3) - ((int32_t)bmp280_handle->dev_cal_factors->dig_T1 << 1))) * (int32_t)bmp280_handle->dev_cal_factors->dig_T2) >> 11;
    var2 = (((((adc_temperature >> 4) - (int32_t)bmp280_handle->dev_cal_factors->dig_T1) * ((adc_temperature >> 4) - (int32_t)bmp280_handle->dev_cal_factors->dig_T1)) >> 12) * (int32_t)bmp280_handle->dev_cal_factors->dig_T3) >> 14;
 
    *fine_temperature = var1 + var2;

    return (*fine_temperature * 5 + 128) >> 8;
}

/**
 * @brief pressure compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[in] adc_pressure raw adc pressure.
 * @param[in] fine_temperature fine temperature in degrees Celsius.
 * @return Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t i2c_bmp280_compensate_pressure(i2c_bmp280_handle_t bmp280_handle, const int32_t adc_pressure, int32_t fine_temperature) {
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temperature - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_handle->dev_cal_factors->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_handle->dev_cal_factors->dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_handle->dev_cal_factors->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_handle->dev_cal_factors->dig_P3) >> 8) + ((var1 * (int64_t)bmp280_handle->dev_cal_factors->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)bmp280_handle->dev_cal_factors->dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)bmp280_handle->dev_cal_factors->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)bmp280_handle->dev_cal_factors->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)bmp280_handle->dev_cal_factors->dig_P7 << 4);

    return p;
}

/**
 * @brief reads calibration factors onboard the bmp280.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_bmp280_get_cal_factors(i2c_bmp280_handle_t bmp280_handle) {
    ESP_ARG_CHECK( bmp280_handle );

    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x88, &bmp280_handle->dev_cal_factors->dig_T1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8a, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_T2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8c, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_T3) );
    /* bmp280 attempt to request P1-P9 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x8e, &bmp280_handle->dev_cal_factors->dig_P1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x90, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x92, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P3) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x94, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P4) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x96, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P5) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x98, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P6) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9a, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P7) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9c, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P8) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp280_handle->i2c_dev_handle, 0x9e, (uint16_t *)&bmp280_handle->dev_cal_factors->dig_P9) );

    ESP_LOGD(TAG, "Calibration data received:");
    ESP_LOGD(TAG, "dig_T1=%u", bmp280_handle->dev_cal_factors->dig_T1);
    ESP_LOGD(TAG, "dig_T2=%d", bmp280_handle->dev_cal_factors->dig_T2);
    ESP_LOGD(TAG, "dig_T3=%d", bmp280_handle->dev_cal_factors->dig_T3);
    ESP_LOGD(TAG, "dig_P1=%u", bmp280_handle->dev_cal_factors->dig_P1);
    ESP_LOGD(TAG, "dig_P2=%d", bmp280_handle->dev_cal_factors->dig_P2);
    ESP_LOGD(TAG, "dig_P3=%d", bmp280_handle->dev_cal_factors->dig_P3);
    ESP_LOGD(TAG, "dig_P4=%d", bmp280_handle->dev_cal_factors->dig_P4);
    ESP_LOGD(TAG, "dig_P5=%d", bmp280_handle->dev_cal_factors->dig_P5);
    ESP_LOGD(TAG, "dig_P6=%d", bmp280_handle->dev_cal_factors->dig_P6);
    ESP_LOGD(TAG, "dig_P7=%d", bmp280_handle->dev_cal_factors->dig_P7);
    ESP_LOGD(TAG, "dig_P8=%d", bmp280_handle->dev_cal_factors->dig_P8);
    ESP_LOGD(TAG, "dig_P9=%d", bmp280_handle->dev_cal_factors->dig_P9);
    
    return ESP_OK;
}

/**
 * @brief reads fixed measurements from the bmp280.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[out] temperature fixed temperature.
 * @param[out] pressure fixed temperature.
 * @return ESP_OK: init success.
 */
static inline esp_err_t i2c_bmp280_get_fixed_measurement(i2c_bmp280_handle_t bmp280_handle, int32_t *temperature, uint32_t *pressure) {
    int32_t         adc_press;
    int32_t         adc_temp;
    i2c_uint48_t    data;

    ESP_ARG_CHECK( bmp280_handle && temperature && pressure );

    // need to read in one sequence to ensure they match.
    ESP_ERROR_CHECK( i2c_master_bus_read_byte48(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_PRESSURE, &data) );

    adc_press = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp  = data[3] << 12 | data[4] << 4 | data[5] >> 4;
    ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);
    ESP_LOGD(TAG, "ADC pressure: %" PRIi32, adc_press);

    int32_t fine_temp;
    *temperature = i2c_bmp280_compensate_temperature(bmp280_handle, adc_temp, &fine_temp);
    *pressure = i2c_bmp280_compensate_pressure(bmp280_handle, adc_press, fine_temp);

    return ESP_OK;
}

esp_err_t i2c_bmp280_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp280_config_t *bmp280_config, i2c_bmp280_handle_t *bmp280_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_bmp280_handle_t out_handle;

    ESP_ARG_CHECK( bus_handle && bmp280_config );

    out_handle = (i2c_bmp280_handle_t)calloc(1, sizeof(i2c_bmp280_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bmp280_config->dev_config.device_address,
        .scl_speed_hz       = I2C_BMP280_DATA_RATE_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c0 new bus failed");
    }

    out_handle->dev_cal_factors = (i2c_bmp280_cal_factors_t*)calloc(1, sizeof(i2c_bmp280_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_cal_factors, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c bmp280 device calibration factors");

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(i2c_master_bus_read_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_ID, &out_handle->dev_type), err, TAG, "i2c0 read device type failed");
    if(out_handle->dev_type != I2C_BMP280_TYPE_BMP280) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err, TAG, "i2c0 detected an invalid device type, got: %02x", out_handle->dev_type);
    }

    /* bmp280 attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_bmp280_reset(out_handle), err, TAG, "i2c0 bmp280 soft-reset failed");

    /* wait until finished copying NVP data */
    //while(1) {
    //    uint8_t status;
    //    if (!i2c_master_bus_read_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_STATUS, &status) && (status & 1) == 0)
    //        break;
    //}
    // forced delay before next transaction - see datasheet for details
    vTaskDelay( i2c_bmp280_get_tick_duration(30) );

    /* bmp280 attempt to read calibration factors from device */
    ESP_GOTO_ON_ERROR(i2c_bmp280_get_cal_factors(out_handle), err, TAG, "i2c0 bmp280 read calibration factors failed");
    
    /* bmp280 attempt to write configuration params to device */
    uint8_t config = (bmp280_config->dev_params.standby << 5) | (bmp280_config->dev_params.filter << 2);
    ESP_LOGD(TAG, "i2c0 bmp280 writing config reg=%02x", config);

    ESP_GOTO_ON_ERROR(i2c_master_bus_write_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, config), err, TAG, "i2c0 bmp280 write configuration params failed");

    /* bmp280 attempt to write control params to device */
    uint8_t ctrl;
    if (bmp280_config->dev_params.mode == I2C_BMP280_MODE_FORCED) {
        // initial mode for forced is sleep
        ctrl = (bmp280_config->dev_params.oversampling_temperature << 5) | (bmp280_config->dev_params.oversampling_pressure << 2) | (I2C_BMP280_MODE_SLEEP);
    } else {
        ctrl = (bmp280_config->dev_params.oversampling_temperature << 5) | (bmp280_config->dev_params.oversampling_pressure << 2) | (bmp280_config->dev_params.mode);
    }
    ESP_LOGD(TAG, "i2c0 bmp280 writing control reg=%02x", ctrl);
    ESP_GOTO_ON_ERROR(i2c_master_bus_write_uint8(out_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, ctrl), err, TAG, "i2c0 bmp280 write control params failed");

    /* copy configuration */
    *bmp280_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_bmp280_get_measurement(i2c_bmp280_handle_t bmp280_handle, float *temperature, float *pressure) {
    int32_t fixed_temperature;
    uint32_t fixed_pressure;

    ESP_ARG_CHECK( bmp280_handle && temperature && pressure );

    ESP_ERROR_CHECK( i2c_bmp280_get_fixed_measurement(bmp280_handle, &fixed_temperature, &fixed_pressure) );

    *temperature = (float)fixed_temperature / 100;
    *pressure = (float)fixed_pressure / 256;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_forced_measurement(i2c_bmp280_handle_t bmp280_handle) {
    esp_err_t ret = ESP_OK;
    uint8_t   ctrl;

    ESP_ARG_CHECK( bmp280_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, &ctrl) );

    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= I2C_BMP280_MODE_FORCED;
    ESP_LOGD(TAG, "writing ctrl reg=%x", ctrl);

    ret = i2c_master_bus_write_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, ctrl);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "write start in forced mode failed");
        return ret;
    }

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_is_measuring(i2c_bmp280_handle_t bmp280_handle, bool *busy) {
    esp_err_t           ret = ESP_OK;
    const i2c_uint16_t  regs = { I2C_BMP280_REG_STATUS, I2C_BMP280_REG_CTRL };
    i2c_uint16_t        status;

    ESP_ARG_CHECK( bmp280_handle && busy );

    ret = i2c_master_transmit_receive(bmp280_handle->i2c_dev_handle, regs, I2C_UINT16_SIZE, status, I2C_UINT16_SIZE, -1);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "read status registers failed");
        return ret;
    }

    // check mode - FORCED means BM280 is busy (it switches to SLEEP mode when finished)
    // additionally, check 'measuring' bit in status register
    *busy = ((status[1] & 0b11) == I2C_BMP280_MODE_FORCED) || (status[0] & (1 << 3));

    return ESP_OK;
}

esp_err_t i2c_bmp280_reset(i2c_bmp280_handle_t bmp280_handle) {
    ESP_ARG_CHECK( bmp280_handle );

    return i2c_master_bus_write_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_RESET, I2C_BMP280_RESET_VALUE);
}

esp_err_t i2c_bmp280_rm(i2c_bmp280_handle_t bmp280_handle) {
    ESP_ARG_CHECK( bmp280_handle );

    return i2c_master_bus_rm_device(bmp280_handle->i2c_dev_handle);
}
