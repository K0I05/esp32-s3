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
#include <stdlib.h>
#include <math.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * HMC5883L definitions
*/

#define I2C_HMC5883L_REG_CONFIG_A               UINT8_C(0x00)
#define I2C_HMC5883L_REG_CONFIG_B               UINT8_C(0x01)
#define I2C_HMC5883L_REG_MODE                   UINT8_C(0x02)
#define I2C_HMC5883L_REG_DATA_OUT_X_MSB         UINT8_C(0x03)
#define I2C_HMC5883L_REG_DATA_OUT_X_LSB         UINT8_C(0x04)
#define I2C_HMC5883L_REG_DATA_OUT_Z_MSB         UINT8_C(0x05)
#define I2C_HMC5883L_REG_DATA_OUT_Z_LSB         UINT8_C(0x06)
#define I2C_HMC5883L_REG_DATA_OUT_Y_MSB         UINT8_C(0x07)
#define I2C_HMC5883L_REG_DATA_OUT_Y_LSB         UINT8_C(0x08)
#define I2C_HMC5883L_REG_STATUS                 UINT8_C(0x09)
#define I2C_HMC5883L_REG_IDENT_A                UINT8_C(0x0a)
#define I2C_HMC5883L_REG_IDENT_B                UINT8_C(0x0b)
#define I2C_HMC5883L_REG_IDENT_C                UINT8_C(0x0c)

#define I2C_HMC5883L_DEV_ID                     UINT32_C(0x00333448)    //!< Chip ID, "H43"

#define I2C_HMC5883L_XY_EXCITATION              (1160)  // The magnetic field excitation in X and Y direction during Self Test (Calibration)
#define I2C_HMC5883L_Z_EXCITATION               (1080)  // The magnetic field excitation in Z direction during Self Test (Calibration)

#define I2C_HMC5883L_DATA_READY_DELAY_MS        UINT16_C(1)
#define I2C_HMC5883L_DATA_POLL_TIMEOUT_MS       UINT16_C(50)
#define I2C_HMC5883L_POWERUP_DELAY_MS           UINT16_C(100)
#define I2C_HMC5883L_APPSTART_DELAY_MS          UINT16_C(20)
#define I2C_HMC5883L_RESET_DELAY_MS             UINT16_C(50)
#define I2C_HMC5883L_CMD_DELAY_MS               UINT16_C(5)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "hmc5883l";

/* Gain sensitivity values for HMC5883L */
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

/**
 * @brief Reads all registers from HMC5883L
 * 
 * @param[in] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_hmc5883l_get_registers(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* hmc5883l attempt to read device identification */
    uint8_t ident_a; uint8_t ident_b; uint8_t ident_c;
    ESP_RETURN_ON_ERROR(i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_IDENT_A, &ident_a), TAG, "read register IDENT_A failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_IDENT_B, &ident_b), TAG, "read register IDENT_B failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_IDENT_C, &ident_c), TAG, "read register IDENT_C failed");
    /* construct device identification and validate */
    hmc5883l_handle->dev_id = ident_a | (ident_b << 8) | (ident_c << 16);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_configuration1_register(hmc5883l_handle), TAG, "read configuration 1 register failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_configuration2_register(hmc5883l_handle), TAG, "read configuration 2 register failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_mode_register(hmc5883l_handle), TAG, "read mode register failed" );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_configuration1_register(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_CONFIG_A, &hmc5883l_handle->config1_reg.reg), TAG, "read configuration 1 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_configuration1_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_configuration1_register_t config1_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register */
    i2c_hmc5883l_configuration1_register_t config1 = { .reg = config1_reg.reg };

    /* set register reserved settings */
    config1.bits.reserved = 0;

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_CONFIG_A, config1.reg), TAG, "write configuration 1 register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_configuration1_register(hmc5883l_handle), TAG, "read configuration 1 register failed" );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_configuration2_register(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_CONFIG_B, &hmc5883l_handle->config2_reg.reg), TAG, "read configuration 2 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_configuration2_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_configuration2_register_t config2_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register */
    i2c_hmc5883l_configuration2_register_t config2 = { .reg = config2_reg.reg };

    /* set register reserved settings */
    config2.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_CONFIG_B, config2.reg), TAG, "write configuration 2 register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_configuration2_register(hmc5883l_handle), TAG, "read configuration 2 register failed" );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_mode_register(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_MODE, &hmc5883l_handle->mode_reg.reg), TAG, "read mode register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_mode_register(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_mode_register_t mode_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register */
    i2c_hmc5883l_mode_register_t mode = { .reg = mode_reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(i2c_master_bus_write_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_MODE, mode.reg), TAG, "write mode register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_get_mode_register(hmc5883l_handle), TAG, "read mode register failed" );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_status_register(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_STATUS, &hmc5883l_handle->status_reg.reg), TAG, "read status register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_init(i2c_master_bus_handle_t bus_handle, const i2c_hmc5883l_config_t *hmc5883l_config, i2c_hmc5883l_handle_t *hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && hmc5883l_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, hmc5883l_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, hmc5883l device handle initialization failed", hmc5883l_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_hmc5883l_handle_t out_handle = (i2c_hmc5883l_handle_t)calloc(1, sizeof(i2c_hmc5883l_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hmc5883l device");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = hmc5883l_config->dev_config.device_address,
        .scl_speed_hz       = hmc5883l_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_CMD_DELAY_MS));

    /* attempt to read registers */
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_get_registers(out_handle), err_handle, TAG, "read registers failed");
    
    /* validate device identifier */
    if (out_handle->dev_id != I2C_HMC5883L_DEV_ID) {
        ESP_LOGE(TAG, "Unknown ID: %lu (device) != %lu (reference)", out_handle->dev_id, I2C_HMC5883L_DEV_ID);
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_FOUND, err_handle, TAG, "i2c hmc5883l device identifier validation failed");
    }

    /* copy registers from device handle */
    i2c_hmc5883l_configuration1_register_t config1_reg = { .reg = out_handle->config1_reg.reg };
    i2c_hmc5883l_configuration2_register_t config2_reg = { .reg = out_handle->config2_reg.reg };
    i2c_hmc5883l_mode_register_t           mode_reg    = { .reg = out_handle->mode_reg.reg };

    /* attempt to write configuration 1 register */
    config1_reg.bits.bias       = hmc5883l_config->bias;
    config1_reg.bits.data_rate  = hmc5883l_config->rate;
    config1_reg.bits.sample_avg = hmc5883l_config->sample;
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_set_configuration1_register(out_handle, config1_reg), err_handle, TAG, "write configuration 1 register failed");

    /* attempt to write configuration 2 register */
    config2_reg.bits.gain       = hmc5883l_config->gain;
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_set_configuration2_register(out_handle, config2_reg), err_handle, TAG, "write configuration 2 register failed");

    /* attempt to write mode register */
    mode_reg.bits.mode          = hmc5883l_config->mode;
    ESP_GOTO_ON_ERROR(i2c_hmc5883l_set_mode_register(out_handle, mode_reg), err_handle, TAG, "write mode register failed");

    /* copy configuration */
    out_handle->declination     = hmc5883l_config->declination;

    /* set device handle */
    *hmc5883l_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_hmc5883l_get_fixed_magnetic_axes(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_axes_data_t *const axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle && axes_data );

    /* initialize local variables */
    esp_err_t    ret       = ESP_OK;
    uint64_t     start     = esp_timer_get_time();
    bool         is_ready  = false;
    bool         is_locked = false;
    i2c_uint48_t rx        = {};

    /* poll data status until data is ready or timeout condition is asserted */
    do {
        /* read data status */
        ESP_GOTO_ON_ERROR( i2c_hmc5883l_get_data_status(hmc5883l_handle, &is_ready, &is_locked), err, TAG, "data ready ready for get fixed measurement failed" );

        /* delay task before i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_HMC5883L_DATA_READY_DELAY_MS));

        if (ESP_TIMEOUT_CHECK(start, I2C_HMC5883L_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (!is_ready);
    
    /* attempt i2c read transaction */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte48(hmc5883l_handle->i2c_dev_handle, I2C_HMC5883L_REG_DATA_OUT_X_MSB, &rx), err, TAG, "read uncompensated compass data for get fixed measurement failed" );

    /* convert 2-byte data to int16 data type - 2s complement */
    axes_data->x_axis = (int16_t)(rx[0] << 8) | rx[1];
    axes_data->z_axis = (int16_t)(rx[4] << 8) | rx[5];
    axes_data->y_axis = (int16_t)(rx[2] << 8) | rx[3];

    //ESP_LOGW(TAG, "Raw X-Axis: %d", data->x_axis);
    //ESP_LOGW(TAG, "Raw Y-Axis: %d", data->y_axis);
    //ESP_LOGW(TAG, "Raw Z-Axis: %d", data->z_axis);

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_hmc5883l_get_magnetic_axes(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_magnetic_axes_data_t *const magnetic_axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle && magnetic_axes_data );

    /* set gain sensitivity */
    const float gain_sensitivity = i2c_hmc5883l_gain_values[hmc5883l_handle->config2_reg.bits.gain];

    /* attempt to read uncompensated magnetic measurements */
    i2c_hmc5883l_axes_data_t raw;
    ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw) );

    /* handle calibration corrections and compensation factors */
    if(hmc5883l_handle->gain_calibrated == true && hmc5883l_handle->offset_calibrated == true) {
        magnetic_axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.x_axis + hmc5883l_handle->offset_axes.x_axis;
        magnetic_axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.y_axis + hmc5883l_handle->offset_axes.y_axis;
        magnetic_axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.z_axis + hmc5883l_handle->offset_axes.z_axis;
    } else if(hmc5883l_handle->gain_calibrated == true && hmc5883l_handle->offset_calibrated == false) {
        magnetic_axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.x_axis;
        magnetic_axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.y_axis;
        magnetic_axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity * hmc5883l_handle->gain_error_axes.z_axis;
    } else if(hmc5883l_handle->gain_calibrated == false && hmc5883l_handle->offset_calibrated == true) {
        magnetic_axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity + hmc5883l_handle->offset_axes.x_axis;
        magnetic_axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity + hmc5883l_handle->offset_axes.y_axis;
        magnetic_axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity + hmc5883l_handle->offset_axes.z_axis;
    } else {
        magnetic_axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity;
        magnetic_axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity;
        magnetic_axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity;
    }

    magnetic_axes_data->heading = atan2f(0.0f - magnetic_axes_data->y_axis, magnetic_axes_data->x_axis) * 180.0f / M_PI;
    //compass_axes_data->heading = atan2(compass_axes_data->y_axis, compass_axes_data->x_axis);
    //compass_axes_data->heading += (hmc5883l_handle->declination * 180/M_PI);
     // Correct for when signs are reversed.
    //if(compass_axes_data->heading < 0) compass_axes_data->heading += 2*M_PI;
    // Check for wrap due to addition of declination.
    //if(compass_axes_data->heading > 2*M_PI) compass_axes_data->heading -= 2*M_PI;
    // Convert radians to degrees for readability.
    //compass_axes_data->heading = compass_axes_data->heading * 180/M_PI;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_calibrated_offsets(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_calibration_options_t option) {
    i2c_hmc5883l_configuration1_register_t config1_reg;
    i2c_hmc5883l_axes_data_t            raw_axes;
    i2c_hmc5883l_magnetic_axes_data_t   scalled_axes;
    i2c_hmc5883l_gain_error_axes_data_t gain_error_axes;
    i2c_hmc5883l_offset_axes_data_t     offset_axes;
    i2c_hmc5883l_offset_axes_data_t     max_offset_axes;
    i2c_hmc5883l_offset_axes_data_t     min_offset_axes;
    uint16_t x_count = 0, y_count = 0, z_count = 0;
    bool x_zero = false, y_zero = false, z_zero = false;

    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy user configured settings */
    i2c_hmc5883l_sample_averages_t  sample  = hmc5883l_handle->config1_reg.bits.sample_avg;             
    i2c_hmc5883l_data_rates_t       rate    = hmc5883l_handle->config1_reg.bits.data_rate;         
    i2c_hmc5883l_biases_t           bias    = hmc5883l_handle->config1_reg.bits.bias;                         
    float                           gain_sensitivity = i2c_hmc5883l_gain_values[hmc5883l_handle->config2_reg.bits.gain];

    /* handle calibration option */
    if(option == I2C_HMC5883L_CAL_GAIN_DIFF || option == I2C_HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Gain");

        // configuring the control register for positive bias mode
        config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
        config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
        config1_reg.bits.bias       = I2C_HMC5883L_BIAS_POSITIVE;
        ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );

        // reading the positive biased data
        //while(raw_axes.x_axis<200 || raw_axes.y_axis<200 || raw_axes.z_axis<200){   // making sure the data is with positive baised
        while(raw_axes.x_axis<50 || raw_axes.y_axis<50 || raw_axes.z_axis<50){
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
        }

        scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // offset = 1160 - data positive
        gain_error_axes.x_axis = (float)I2C_HMC5883L_XY_EXCITATION/scalled_axes.x_axis;
        gain_error_axes.y_axis = (float)I2C_HMC5883L_XY_EXCITATION/scalled_axes.y_axis;
        gain_error_axes.z_axis = (float)I2C_HMC5883L_Z_EXCITATION/scalled_axes.z_axis;

        // configuring the control register for negative bias mode
        config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
        config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
        config1_reg.bits.bias       = I2C_HMC5883L_BIAS_NEGATIVE;
        ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );

        // reading the negative biased data
        //while(raw_axes.x_axis>-200 || raw_axes.y_axis>-200 || raw_axes.z_axis>-200){   // making sure the data is with negative baised
        while(raw_axes.x_axis>-50 || raw_axes.y_axis>-50 || raw_axes.z_axis>-50){
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
        }

        scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // taking the average of the offsets
        gain_error_axes.x_axis = (float)((I2C_HMC5883L_XY_EXCITATION/fabs(scalled_axes.x_axis))+gain_error_axes.x_axis)/2;
        gain_error_axes.y_axis = (float)((I2C_HMC5883L_XY_EXCITATION/fabs(scalled_axes.y_axis))+gain_error_axes.y_axis)/2;
        gain_error_axes.z_axis = (float)((I2C_HMC5883L_Z_EXCITATION/fabs(scalled_axes.z_axis))+gain_error_axes.z_axis)/2;

        hmc5883l_handle->gain_calibrated = true;
        hmc5883l_handle->gain_error_axes = gain_error_axes;

        ESP_LOGW(TAG, "Gain Offset X-Axis: %f", gain_error_axes.x_axis);
        ESP_LOGW(TAG, "Gain Offset Y-Axis: %f", gain_error_axes.y_axis);
        ESP_LOGW(TAG, "Gain Offset Z-Axis: %f", gain_error_axes.z_axis);
    }

    // configuring the control register for normal mode
    config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
    config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
    config1_reg.bits.bias       = I2C_HMC5883L_BIAS_NORMAL;
    ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

    if(option == I2C_HMC5883L_CAL_AXES_MEAN || option == I2C_HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Offset");
        ESP_LOGW(TAG, "Please rotate the magnetometer 2 or 3 times in complete circules with in one minute .............");

        while (x_count < 3 || y_count < 3 || z_count < 3) {
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
            scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
            scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
            scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

            //if ((fabs(scalled_axes.x_axis) > 600) || (fabs(scalled_axes.y_axis) > 600) || (fabs(scalled_axes.z_axis) > 600)) {
            if ((fabs(scalled_axes.x_axis) > 100) || (fabs(scalled_axes.y_axis) > 100) || (fabs(scalled_axes.z_axis) > 100)) {
                continue;
            }

            if (min_offset_axes.x_axis > scalled_axes.x_axis) {
                min_offset_axes.x_axis = scalled_axes.x_axis;
            } else if (max_offset_axes.x_axis < scalled_axes.x_axis) {
                max_offset_axes.x_axis = scalled_axes.x_axis;
            }

            if (min_offset_axes.y_axis > scalled_axes.y_axis) {
                min_offset_axes.y_axis = scalled_axes.y_axis;
            } else if (max_offset_axes.y_axis < scalled_axes.y_axis) {
                max_offset_axes.y_axis = scalled_axes.y_axis;
            }

            if (min_offset_axes.z_axis > scalled_axes.z_axis) {
                min_offset_axes.z_axis = scalled_axes.z_axis;
            } else if (max_offset_axes.z_axis < scalled_axes.z_axis) {
                max_offset_axes.z_axis = scalled_axes.z_axis;
            }

            if (x_zero) {
                if (fabs(scalled_axes.x_axis) > 50) {
                    x_zero = false;
                    x_count++;
                }
            } else {
                if (fabs(scalled_axes.x_axis) < 40) {
                    x_zero = true;
                }
            }

            if (y_zero) {
                if (fabs(scalled_axes.y_axis) > 50) {
                    y_zero = false;
                    y_count++;
                }
            } else {
                if (fabs(scalled_axes.y_axis) < 40) {
                    y_zero = true;
                }
            }

            if (z_zero) {
                if (fabs(scalled_axes.z_axis) > 50) {
                    z_zero = false;
                    z_count++;
                }
            } else {
                if (fabs(scalled_axes.z_axis) < 40) {
                    z_zero = true;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(30));
        }

        offset_axes.x_axis = (max_offset_axes.x_axis + min_offset_axes.x_axis) / 2;
        offset_axes.y_axis = (max_offset_axes.y_axis + min_offset_axes.y_axis) / 2;
        offset_axes.z_axis = (max_offset_axes.z_axis + min_offset_axes.z_axis) / 2;

        hmc5883l_handle->offset_calibrated = true;
        hmc5883l_handle->offset_axes = offset_axes;

        ESP_LOGW(TAG, "Offset X-Axis: %f", offset_axes.x_axis);
        ESP_LOGW(TAG, "Offset Y-Axis: %f", offset_axes.y_axis);
        ESP_LOGW(TAG, "Offset Z-Axis: %f", offset_axes.z_axis);
    }

    // configuring the control register to user defined settings
    config1_reg.bits.bias       = bias;
    config1_reg.bits.data_rate  = rate;
    config1_reg.bits.sample_avg = sample;
    ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

    return ESP_OK;
}

// https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/blob/master/Core/Compass_header_example_ver_0_2/compass.cpp
esp_err_t i2c_hmc5883l_get_calibrated_offsets___(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_calibration_options_t option) {
    i2c_hmc5883l_configuration1_register_t config1_reg;
    i2c_hmc5883l_axes_data_t raw_axes;
    i2c_hmc5883l_magnetic_axes_data_t   scalled_axes;
    i2c_hmc5883l_gain_error_axes_data_t gain_error_axes;
    i2c_hmc5883l_offset_axes_data_t offset_axes;

    /* copy user configured settings */
    i2c_hmc5883l_sample_averages_t  sample  = hmc5883l_handle->config1_reg.bits.sample_avg;             
    i2c_hmc5883l_data_rates_t       rate    = hmc5883l_handle->config1_reg.bits.data_rate;         
    i2c_hmc5883l_biases_t           bias    = hmc5883l_handle->config1_reg.bits.bias;                        
    float                           gain_sensitivity = i2c_hmc5883l_gain_values[hmc5883l_handle->config2_reg.bits.gain];

    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* handle calibration option */
    if(option == I2C_HMC5883L_CAL_GAIN_DIFF || option == I2C_HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Gain");

        // configuring the control register for positive bias mode
        config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
        config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
        config1_reg.bits.bias       = I2C_HMC5883L_BIAS_POSITIVE;
        ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );

        // reading the positive biased data
        //while(raw_axes.x_axis<200 || raw_axes.y_axis<200 || raw_axes.z_axis<200){   // making sure the data is with positive baised
        while(raw_axes.x_axis<50 || raw_axes.y_axis<50 || raw_axes.z_axis<50){
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
        }

        scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // offset = 1160 - data positive
        gain_error_axes.x_axis = (float)I2C_HMC5883L_XY_EXCITATION/scalled_axes.x_axis;
        gain_error_axes.y_axis = (float)I2C_HMC5883L_XY_EXCITATION/scalled_axes.y_axis;
        gain_error_axes.z_axis = (float)I2C_HMC5883L_Z_EXCITATION/scalled_axes.z_axis;

        // configuring the control register for negative bias mode
        config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
        config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
        config1_reg.bits.bias       = I2C_HMC5883L_BIAS_NEGATIVE;
        ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );

        // reading the negative biased data
        //while(raw_axes.x_axis>-200 || raw_axes.y_axis>-200 || raw_axes.z_axis>-200){   // making sure the data is with negative baised
        while(raw_axes.x_axis>-50 || raw_axes.y_axis>-50 || raw_axes.z_axis>-50){
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
        }

        scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // taking the average of the offsets
        gain_error_axes.x_axis = (float)((I2C_HMC5883L_XY_EXCITATION/fabs(scalled_axes.x_axis))+gain_error_axes.x_axis)/2;
        gain_error_axes.y_axis = (float)((I2C_HMC5883L_XY_EXCITATION/fabs(scalled_axes.y_axis))+gain_error_axes.y_axis)/2;
        gain_error_axes.z_axis = (float)((I2C_HMC5883L_Z_EXCITATION/fabs(scalled_axes.z_axis))+gain_error_axes.z_axis)/2;

        hmc5883l_handle->gain_calibrated = true;
        hmc5883l_handle->gain_error_axes = gain_error_axes;

        ESP_LOGW(TAG, "Gain Offset X-Axis: %f", gain_error_axes.x_axis);
        ESP_LOGW(TAG, "Gain Offset Y-Axis: %f", gain_error_axes.y_axis);
        ESP_LOGW(TAG, "Gain Offset Z-Axis: %f", gain_error_axes.z_axis);
    }

    // configuring the control register for normal mode
    config1_reg.bits.sample_avg = I2C_HMC5883L_SAMPLE_8;
    config1_reg.bits.data_rate  = I2C_HMC5883L_DATA_RATE_15_00;
    config1_reg.bits.bias       = I2C_HMC5883L_BIAS_NORMAL;
    ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

    if(option == I2C_HMC5883L_CAL_AXES_MEAN || option == I2C_HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Offset");
        ESP_LOGW(TAG, "Please rotate the magnetometer 2 or 3 times in complete circules with in one minute .............");

        for(uint8_t i = 0; i < 10; i++){   // disregarding first few data
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );
        }
    
        float x_max=-4000, y_max=-4000, z_max=-4000; 
        float x_min=4000, y_min=4000, z_min=4000;

        //unsigned long t = millis();
        uint64_t t = esp_timer_get_time();
        //while(millis()-t <= 30000){
        while(esp_timer_get_time()-t <= 30000){
            ESP_ERROR_CHECK( i2c_hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle, &raw_axes) );

            scalled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity * gain_error_axes.x_axis;
            scalled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity * gain_error_axes.y_axis;
            scalled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity * gain_error_axes.z_axis;

            x_max = fmax(x_max,scalled_axes.x_axis);
            y_max = fmax(y_max,scalled_axes.y_axis);
            z_max = fmax(z_max,scalled_axes.z_axis);

            x_min = fmin(x_min,scalled_axes.x_axis);
            y_min = fmin(y_min,scalled_axes.y_axis);
            z_min = fmin(z_min,scalled_axes.z_axis);
        }

        offset_axes.x_axis = ((x_max-x_min)/2)-x_max;
        offset_axes.y_axis = ((y_max-y_min)/2)-y_max;
        offset_axes.z_axis = ((z_max-z_min)/2)-z_max;

        hmc5883l_handle->offset_calibrated = true;
        hmc5883l_handle->offset_axes = offset_axes;

        ESP_LOGW(TAG, "Offset X-Axis: %f", offset_axes.x_axis);
        ESP_LOGW(TAG, "Offset Y-Axis: %f", offset_axes.y_axis);
        ESP_LOGW(TAG, "Offset Z-Axis: %f", offset_axes.z_axis);
    }

    // configuring the control register to user defined settings
    config1_reg.bits.bias       = bias;
    config1_reg.bits.data_rate  = rate;
    config1_reg.bits.sample_avg = sample;
    ESP_RETURN_ON_ERROR(i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg), TAG, "write configuration 1 register failed");

    return ESP_OK;
}


esp_err_t i2c_hmc5883l_get_data_status(i2c_hmc5883l_handle_t hmc5883l_handle, bool *const ready, bool *const locked) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle && ready && locked );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_status_register(hmc5883l_handle) );

    /* set output parameters */
    *ready  = hmc5883l_handle->status_reg.bits.data_ready;
    *locked = hmc5883l_handle->status_reg.bits.data_locked;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_mode(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_mode_register(hmc5883l_handle) );

    /* set output parameter */
    *mode = hmc5883l_handle->mode_reg.bits.mode;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_mode(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register from handle and set parameter */
    i2c_hmc5883l_mode_register_t mode_reg = { .reg = hmc5883l_handle->mode_reg.reg };

    /* set register setting */
    mode_reg.bits.mode = mode;

    /* attempt to write to register */
    ESP_ERROR_CHECK( i2c_hmc5883l_set_mode_register(hmc5883l_handle, mode_reg) );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_sample_averages_t *const sample) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_configuration1_register(hmc5883l_handle) );

    /* set output parameter */
    *sample = hmc5883l_handle->config1_reg.bits.sample_avg;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_samples_averaged(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_sample_averages_t sample) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register from handle and set parameter */
    i2c_hmc5883l_configuration1_register_t config1_reg = { .reg = hmc5883l_handle->config1_reg.reg };

    /* set register setting */
    config1_reg.bits.sample_avg = sample;

    /* attempt to write to register */
    ESP_ERROR_CHECK( i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg) );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_data_rates_t *const rate) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_configuration1_register(hmc5883l_handle) );

    /* set output parameter */
    *rate = hmc5883l_handle->config1_reg.bits.data_rate;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_data_rate(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_data_rates_t rate) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register from handle and set parameter */
    i2c_hmc5883l_configuration1_register_t config1_reg = { .reg = hmc5883l_handle->config1_reg.reg };

    /* set register setting */
    config1_reg.bits.data_rate = rate;

    /* attempt to write to register */
    ESP_ERROR_CHECK( i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg) );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_bias(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_biases_t *const bias) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_configuration1_register(hmc5883l_handle) );

    /* set output parameter */
    *bias = hmc5883l_handle->config1_reg.bits.bias;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_bias(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_biases_t bias) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register from handle and set parameter */
    i2c_hmc5883l_configuration1_register_t config1_reg = { .reg = hmc5883l_handle->config1_reg.reg };

    /* set register setting */
    config1_reg.bits.bias = bias;

    /* attempt to write to register */
    ESP_ERROR_CHECK( i2c_hmc5883l_set_configuration1_register(hmc5883l_handle, config1_reg) );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_gain(i2c_hmc5883l_handle_t hmc5883l_handle, i2c_hmc5883l_gains_t *const gain) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_configuration2_register(hmc5883l_handle) );

    /* set output parameter */
    *gain = hmc5883l_handle->config2_reg.bits.gain;

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_set_gain(i2c_hmc5883l_handle_t hmc5883l_handle, const i2c_hmc5883l_gains_t gain) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* copy register from handle and set parameter */
    i2c_hmc5883l_configuration2_register_t config2_reg = { .reg = hmc5883l_handle->config2_reg.reg };

    /* set register setting */
    config2_reg.bits.gain = gain;

    /* attempt to write to register */
    ESP_ERROR_CHECK( i2c_hmc5883l_set_configuration2_register(hmc5883l_handle, config2_reg) );

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_get_gain_sensitivity(i2c_hmc5883l_handle_t hmc5883l_handle, float *const sensitivity) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( i2c_hmc5883l_get_configuration2_register(hmc5883l_handle) );

    /* set output parameter */
    *sensitivity = i2c_hmc5883l_gain_values[hmc5883l_handle->config2_reg.bits.gain];

    return ESP_OK;
}

esp_err_t i2c_hmc5883l_remove(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    return i2c_master_bus_rm_device(hmc5883l_handle->i2c_dev_handle);
}

esp_err_t i2c_hmc5883l_delete(i2c_hmc5883l_handle_t hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hmc5883l_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_hmc5883l_remove(hmc5883l_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(hmc5883l_handle->i2c_dev_handle) {
        free(hmc5883l_handle->i2c_dev_handle);
        free(hmc5883l_handle);
    }

    return ESP_OK;
}