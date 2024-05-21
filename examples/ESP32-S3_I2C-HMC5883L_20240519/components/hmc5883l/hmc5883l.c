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
 * @file hmc5883l.c
 *
 * ESP-IDF driver for HMC5883L digital compass sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "hmc5883l.h"
#include <string.h>
#include <stdio.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_HMC5883L_BIT_MA  5
#define I2C_HMC5883L_BIT_DO  2
#define I2C_HMC5883L_BIT_GN  5

#define I2C_HMC5883L_MASK_MD 0x03
#define I2C_HMC5883L_MASK_MA 0x60
#define I2C_HMC5883L_MASK_DO 0x1c
#define I2C_HMC5883L_MASK_MS 0x03
#define I2C_HMC5883L_MASK_DR 0x01
#define I2C_HMC5883L_MASK_DL 0x02

/*
 * macro definitions
*/
#define timeout_expired(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "hmc5883l";
static const float i2c_hmc5883l_gain_values [] = {
    [I2C_HMC5883L_GAIN_1370] = 0.73,
    [I2C_HMC5883L_GAIN_1090] = 0.92,
    [I2C_HMC5883L_GAIN_820]  = 1.22,
    [I2C_HMC5883L_GAIN_660]  = 1.52,
    [I2C_HMC5883L_GAIN_440]  = 2.27,
    [I2C_HMC5883L_GAIN_390]  = 2.56,
    [I2C_HMC5883L_GAIN_330]  = 3.03,
    [I2C_HMC5883L_GAIN_230]  = 4.35
};

/*
* functions and subrountines
*/

static inline esp_err_t i2c_hmc5883l_write_register(i2c_hmc5883l_handle_t hmc5883l_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t tx[I2C_UINT16_SIZE] = { reg_addr, data };

    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_master_transmit(hmc5883l_handle->i2c_dev_handle, tx, I2C_UINT16_SIZE, -1) );

    return ESP_OK;
}

static inline esp_err_t i2c_hmc5883l_read_register(i2c_hmc5883l_handle_t hmc5883l_handle, uint8_t reg_addr, uint8_t *data) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, reg_addr, data) );

    return ESP_OK;
}

static inline esp_err_t i2c_hmc5883l_update_register(i2c_hmc5883l_handle_t hmc5883l_handle, uint8_t reg_addr, uint8_t mask, uint8_t data) {
    uint8_t old;
    esp_err_t ret = i2c_hmc5883l_read_register(hmc5883l_handle, reg_addr, &old);

    if (ret != ESP_OK)
        return ret;

    return i2c_hmc5883l_write_register(hmc5883l_handle, reg_addr, (old & mask) | data);
}

esp_err_t i2c_hmc5883l_init(i2c_master_bus_handle_t bus_handle, const i2c_hmc5883l_config_t *hmc5883l_config, i2c_hmc5883l_handle_t *hmc5883l_handle) {
    esp_err_t             ret = ESP_OK;
    i2c_hmc5883l_handle_t out_handle;

    ESP_ARG_CHECK( bus_handle && hmc5883l_config );

    out_handle = (i2c_hmc5883l_handle_t)calloc(1, sizeof(i2c_hmc5883l_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hmc5883l device");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = hmc5883l_config->dev_config.device_address,
        .scl_speed_hz       = I2C_HMC5883L_FREQ_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    /* hmc5883l attempt to read and validate device identification */
    uint8_t ident_a; uint8_t ident_b; uint8_t ident_c;
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_read_register(out_handle, I2C_HMC5883L_REG_IDENT_A, &ident_a), err, TAG, "i2c read register IDENT_A failed");
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_read_register(out_handle, I2C_HMC5883L_REG_IDENT_B, &ident_b), err, TAG, "i2c read register IDENT_B failed");
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_read_register(out_handle, I2C_HMC5883L_REG_IDENT_C, &ident_c), err, TAG, "i2c read register IDENT_C failed");
    /* construct device identification and validate */
    uint32_t id = ident_a | (ident_b << 8) | (ident_c << 16);
    if (id != I2C_HMC5883L_DEV_ID) {
        ESP_LOGE(TAG, "Unknown ID: %lu (device) != %lu (reference)", id, I2C_HMC5883L_DEV_ID);
        ESP_LOGE(TAG, "Unknown ID: a(%02x), b(%02x), c(%02x)", ident_a, ident_b, ident_c);
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_FOUND, err, TAG, "i2c hmc5883l device identifier validation failed");
    }

    /* copy configuration and set device handle */
    out_handle->mode = hmc5883l_config->mode;
    out_handle->sample = hmc5883l_config->sample;
    out_handle->rate = hmc5883l_config->rate;
    out_handle->gain = hmc5883l_config->gain;
    out_handle->bias = hmc5883l_config->bias;
    

    /* hmc5883l attempt to write configurations */
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_mode(out_handle), err, TAG, "i2c write operation mode configuration failed");
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_mode(out_handle, hmc5883l_config->mode), err, TAG, "i2c write operation mode configuration failed");
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_samples_averaged(out_handle, hmc5883l_config->sample), err, TAG, "i2c write samples averaged configuration failed");
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_data_rate(out_handle, hmc5883l_config->rate), err, TAG, "i2c write data rate configuration failed");
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_bias(out_handle, hmc5883l_config->bias), err, TAG, "i2c write measurement mode (bias) configuration failed");
    //ESP_GOTO_ON_ERROR(i2c_hmc5883l_write_gain(out_handle, hmc5883l_config->gain), err, TAG, "i2c write measurement gain configuration failed");

    *hmc5883l_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

/*
esp_err_t i2c_hmc5883l_read_mode(i2c_hmc5883l_handle_t hmc5883l_handle) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_MODE, (uint8_t *)&hmc5883l_handle->mode) );

    hmc5883l_handle->mode = (hmc5883l_handle->mode & I2C_HMC5883L_MASK_MD) == 0 ? I2C_HMC5883L_MODE_CONTINUOUS : I2C_HMC5883L_MODE_SINGLE;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_mode(i2c_hmc5883l_handle_t hmc5883l_handle) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_write_register(hmc5883l_handle, I2C_HMC5883L_REG_MODE, hmc5883l_handle->mode) );

    return ESP_OK;
}
*/

esp_err_t i2c_hmc5883l_read_mode(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_modes_t *mode) {
    ESP_ARG_CHECK( hmc5883l_handle && mode );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_MODE, (uint8_t *)&mode) );

    *mode = (*mode & I2C_HMC5883L_MASK_MD) == 0 ? I2C_HMC5883L_MODE_CONTINUOUS : I2C_HMC5883L_MODE_SINGLE;

    hmc5883l_handle->mode = *mode;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_mode(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_modes_t mode) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_write_register(hmc5883l_handle, I2C_HMC5883L_REG_MODE, mode) );

    //hmc5883l_handle->mode = mode;

    return ESP_OK;
}


esp_err_t i2c_hmc5883l_read_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_sample_averages_t *sample) {
    ESP_ARG_CHECK( hmc5883l_handle && sample );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, (uint8_t *)&sample) );

    *sample = (*sample & I2C_HMC5883L_MASK_MA) >> I2C_HMC5883L_BIT_MA;

    hmc5883l_handle->sample = *sample;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_sample_averages_t sample) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_update_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, I2C_HMC5883L_MASK_MA, sample << I2C_HMC5883L_BIT_MA) );

    hmc5883l_handle->sample = sample;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_read_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_data_rates_t *rate) {
    ESP_ARG_CHECK( hmc5883l_handle && rate );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, (uint8_t *)&rate) );

    *rate = (*rate & I2C_HMC5883L_MASK_DO) >> I2C_HMC5883L_BIT_DO;

    hmc5883l_handle->rate = *rate;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_data_rates_t rate) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_update_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, I2C_HMC5883L_MASK_DO, rate << I2C_HMC5883L_BIT_DO) );

    hmc5883l_handle->rate = rate;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_read_bias(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_biases_t *bias) {
    ESP_ARG_CHECK( hmc5883l_handle && bias );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, (uint8_t *)&bias) );

    *bias &= I2C_HMC5883L_MASK_MS;

    hmc5883l_handle->bias = *bias;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_bias(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_biases_t bias) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_update_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_A, I2C_HMC5883L_MASK_MS, bias) );

    hmc5883l_handle->bias = bias;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_read_gain(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_gains_t *gain) {
    ESP_ARG_CHECK( hmc5883l_handle && gain );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_B, (uint8_t *)&gain) );

    *gain >>= I2C_HMC5883L_BIT_GN;

    hmc5883l_handle->gain = *gain;
    hmc5883l_handle->gain_value = i2c_hmc5883l_gain_values[*gain];

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_write_gain(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_gains_t gain) {
    ESP_ARG_CHECK( hmc5883l_handle );

    ESP_ERROR_CHECK( i2c_hmc5883l_write_register(hmc5883l_handle, I2C_HMC5883L_REG_CONFIG_B, gain << I2C_HMC5883L_BIT_GN) );

    hmc5883l_handle->gain = gain;
    hmc5883l_handle->gain_value = i2c_hmc5883l_gain_values[gain];

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_data_is_locked(i2c_hmc5883l_handle_t hmc5883l_handle, bool *locked) {
    ESP_ARG_CHECK( hmc5883l_handle && locked );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_STATUS, (uint8_t *)&locked) );

    *locked &= I2C_HMC5883L_MASK_DL;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_data_is_ready(i2c_hmc5883l_handle_t hmc5883l_handle, bool *ready) {
    ESP_ARG_CHECK( hmc5883l_handle && ready );

    ESP_ERROR_CHECK( i2c_hmc5883l_read_register(hmc5883l_handle, I2C_HMC5883L_REG_STATUS, (uint8_t *)&ready) );

    *ready &= I2C_HMC5883L_MASK_DR;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_read_raw_data(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_raw_data_t *raw_data) {
    ESP_ARG_CHECK( hmc5883l_handle && raw_data );

    if (hmc5883l_handle->mode == I2C_HMC5883L_MODE_SINGLE) {
        // overwrite mode register for measurement
        //ESP_ERROR_CHECK( i2c_hmc5883l_write_mode(hmc5883l_handle, hmc5883l_handle->mode) );
        // wait for data
        uint64_t start = esp_timer_get_time();
        bool dready = false;
        do
        {
            ESP_ERROR_CHECK( i2c_hmc5883l_data_is_ready(hmc5883l_handle, &dready) );
            if (timeout_expired(start, I2C_HMC5883L_MEAS_TIMEOUT))
                return ESP_ERR_TIMEOUT;
        } while (!dready);
    }
    uint8_t buf[I2C_UINT48_SIZE];
    uint8_t reg[I2C_UINT8_SIZE] = { I2C_HMC5883L_REG_DATA_OUT_X_MSB };

    ESP_ERROR_CHECK( i2c_master_transmit_receive(hmc5883l_handle->i2c_dev_handle, reg, I2C_UINT8_SIZE, buf, I2C_UINT48_SIZE, -1) );

    raw_data->x = ((int16_t)buf[I2C_HMC5883L_REG_DATA_OUT_X_MSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB] << 8) | buf[I2C_HMC5883L_REG_DATA_OUT_X_LSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB];
    raw_data->y = ((int16_t)buf[I2C_HMC5883L_REG_DATA_OUT_Y_MSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB] << 8) | buf[I2C_HMC5883L_REG_DATA_OUT_Y_LSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB];
    raw_data->z = ((int16_t)buf[I2C_HMC5883L_REG_DATA_OUT_Z_MSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB] << 8) | buf[I2C_HMC5883L_REG_DATA_OUT_Z_LSB - I2C_HMC5883L_REG_DATA_OUT_X_MSB];

    return ESP_OK;
}