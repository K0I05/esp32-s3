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
 * @file ak8976.c
 *
 * ESP-IDF driver for AK8975 3-axis electronic compass
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ak8975.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define timeout_expired(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "ak8975";

/*
* functions and subrountines
*/

static inline int16_t i2c_ak8975_axis_sensitivity_adjustment(uint8_t asa, i2c_bytes_to_int16_t data) {
    /* see datasheet for details: section 8.3.11 */
    return data.value*((asa-128)*0.5/128+1);
}

static inline esp_err_t i2c_ak8975_process_compass_axes(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_axes_data_t axes_data, i2c_ak8975_compass_axes_data_t *compass_axes) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* apply sensitivity adjustments */
    compass_axes->x_axis = i2c_ak8975_axis_sensitivity_adjustment(ak8975_handle->asa_x_value, axes_data.x_axis);
    compass_axes->y_axis = i2c_ak8975_axis_sensitivity_adjustment(ak8975_handle->asa_y_value, axes_data.y_axis);
    compass_axes->z_axis = i2c_ak8975_axis_sensitivity_adjustment(ak8975_handle->asa_z_value, axes_data.z_axis);

    return ESP_OK;
}

static inline esp_err_t i2c_ak8975_set_mode(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_operating_modes_t mode) {
    i2c_ak8975_control_register_t control_reg;

    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* set operating mode */
    control_reg.bits.mode = mode;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ak8975_set_control_register(ak8975_handle, control_reg), TAG, "set mode failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AK8975_MODE_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t i2c_ak8975_set_self_test(i2c_ak8975_handle_t ak8975_handle, bool state) {
    i2c_ak8975_self_test_control_register_t self_test_reg;

    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* set self-test control register */
    self_test_reg.bits.self_test = state;
    self_test_reg.bits.reserved1 = 0;
    self_test_reg.bits.reserved2 = 0;

    /* attempt i2c self test control register write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_SELF_TEST_RW, self_test_reg.reg), TAG, "write self test control register failed" );

    return ESP_OK;
}

static inline esp_err_t i2c_ak8975_get_device_id(i2c_ak8975_handle_t ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_DEVICE_ID_R, &ak8975_handle->device_id), TAG, "ak8975 device identifier register failed" );
    
    return ESP_OK;
}

static inline esp_err_t i2c_ak8975_get_device_information(i2c_ak8975_handle_t ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_INFO_R, &ak8975_handle->device_info), TAG, "ak8975 device information register failed" );
    
    return ESP_OK;
}

static inline esp_err_t i2c_ak8975_get_axes_data(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_axes_data_t *axes_data) {
    esp_err_t ret           = ESP_OK;
    uint64_t  start_time    = 0;
    bool      data_is_ready = false;

    //I2C_AK8975_MEASUREMENT_DELAY_MS

    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to wait until data is available */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_ak8975_get_data_status(ak8975_handle, &data_is_ready), err, TAG, "data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_AK8975_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (timeout_expired(start_time, (I2C_AK8975_MEASUREMENT_DELAY_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);
    
    /* attempt i2c read data registers transactions */
    //ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte16(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_HXL_DATA_R, &axes_data->x_axis.bytes), err, TAG, "read x-axis lo-hi bytes failed" );
    //ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte16(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_HYL_DATA_R, &axes_data->y_axis.bytes), err, TAG, "read y-axis lo-hi bytes failed" );
    //ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte16(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_HZL_DATA_R, &axes_data->z_axis.bytes), err, TAG, "read z-axis lo-hi bytes failed" );


    i2c_uint48_t rx = { 0, 0, 0, 0, 0, 0};

    /* 6-byte i2c read transaction */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte48(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_HXL_DATA_R, &rx), err, TAG, "read axes (x, y, z) bytes failed" );
    
    axes_data->x_axis.bytes[0] = rx[0];
    axes_data->x_axis.bytes[1] = rx[1];
    axes_data->y_axis.bytes[0] = rx[2];
    axes_data->y_axis.bytes[1] = rx[3];
    axes_data->z_axis.bytes[0] = rx[4];
    axes_data->z_axis.bytes[1] = rx[5];

    /* debug purposes */
    ESP_LOGW(TAG, "ak8975 x-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->x_axis.bytes[0], axes_data->x_axis.bytes[1], axes_data->x_axis.value);
    ESP_LOGW(TAG, "ak8975 y-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->y_axis.bytes[0], axes_data->y_axis.bytes[1], axes_data->y_axis.value);
    ESP_LOGW(TAG, "ak8975 z-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->z_axis.bytes[0], axes_data->z_axis.bytes[1], axes_data->z_axis.value);

    
    /* validate data status */
    bool data_error    = false;
    bool data_overflow = false;

    i2c_ak8975_get_error_status(ak8975_handle, &data_error);
    ESP_LOGE(TAG, "ak8975 data error    %s", data_error ? "true" : "false");

    i2c_ak8975_get_overflow_status(ak8975_handle, &data_overflow);
    ESP_LOGE(TAG, "ak8975 data overflow %s", data_overflow ? "true" : "false");

    if(data_error == true) return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ak8975_get_control_register(i2c_ak8975_handle_t ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_CONTROL_RW, &ak8975_handle->control_reg.reg), TAG, "ak8975 read control register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ak8975_set_control_register(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_control_register_t control_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    control_reg.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_CONTROL_RW, control_reg.reg), TAG, "write control register failed" );

    /* set device handle control register */
    ak8975_handle->control_reg.reg = control_reg.reg;

    return ESP_OK;
}

esp_err_t i2c_ak8975_get_status1_register(i2c_ak8975_handle_t ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_STATUS_1_R, &ak8975_handle->status1_reg.reg), TAG, "read status 1 register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ak8975_get_status2_register(i2c_ak8975_handle_t ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_STATUS_2_R, &ak8975_handle->status2_reg.reg), TAG, "read status 2 register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ak8975_get_asa_registers(i2c_ak8975_handle_t ak8975_handle) {
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_FUSE_ROM), TAG, "fuse rom access mode failed" );

    /* attempt i2c read transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_ASAX_VALUE_R, &ak8975_handle->asa_x_value), TAG, "read asax register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_ASAY_VALUE_R, &ak8975_handle->asa_y_value), TAG, "read asay register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ak8975_handle->i2c_dev_handle, I2C_AK8975_REG_ASAZ_VALUE_R, &ak8975_handle->asa_z_value), TAG, "read asaz register failed" );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_POWER_DOWN), TAG, "power down mode failed" );
    
    return ESP_OK;
}

float i2c_ak8975_process_heading(i2c_ak8975_compass_axes_data_t compass_axes) {
    float heading = atan2((double)compass_axes.y_axis, (double)compass_axes.x_axis) * 180.0/3.14159265 + 180;
    while (heading < 0) heading += 360;
    while (heading > 360) heading -= 360;
    return heading;
}

esp_err_t i2c_ak8975_init(i2c_master_bus_handle_t bus_handle, const i2c_ak8975_config_t *ak8975_config, i2c_ak8975_handle_t *ak8975_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_ak8975_handle_t out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ak8975_config );

    /* validate memory availability for handle */
    out_handle = (i2c_ak8975_handle_t)calloc(1, sizeof(i2c_ak8975_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ak8975 device");

    /* validate memory availability for handle parameters */
    //out_handle->dev_params = (i2c_ak8975_params_t*)calloc(1, sizeof(i2c_ak8975_params_t));
    //ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ak8975 device configuration parameters");

    /* set i2c device configuration */
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ak8975_config->dev_config.device_address,
        .scl_speed_hz       = I2C_AK8975_DATA_RATE_HZ,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    /* attempt to read device identifier */
    ESP_GOTO_ON_ERROR( i2c_ak8975_get_device_id(out_handle), err, TAG, "read device identifier failed" );

    /* attempt to read device information */
    ESP_GOTO_ON_ERROR( i2c_ak8975_get_device_information(out_handle), err, TAG, "read device information failed" );

    /* attempt to read device control register */
    ESP_GOTO_ON_ERROR(i2c_ak8975_get_control_register(out_handle), err, TAG, "read control register failed");

    /* attempt to read device status 1 register */
    ESP_GOTO_ON_ERROR(i2c_ak8975_get_status1_register(out_handle), err, TAG, "read status 1 register failed");

    /* attempt to read device status 2 register */
    ESP_GOTO_ON_ERROR(i2c_ak8975_get_status2_register(out_handle), err, TAG, "read status 2 register failed");

    /* attempt to read device sensitivity adjustment registers */
    ESP_GOTO_ON_ERROR(i2c_ak8975_get_asa_registers(out_handle), err, TAG, "read sensitivity adjustment registers failed");

    /* set device handle */
    *ak8975_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_ak8975_get_compass_axes(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_compass_axes_data_t *compass_axes) {
    esp_err_t               ret = ESP_OK;
    i2c_ak8975_axes_data_t  axes_data;

    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_SINGLE_MEAS), err, TAG, "single measurement mode failed" );

    /* attempt to read data axes */
    ESP_GOTO_ON_ERROR( i2c_ak8975_get_axes_data(ak8975_handle, &axes_data), err, TAG, "read axes data failed" );

    /* attempt to process compass axes sensitivity adjustments */
    ESP_GOTO_ON_ERROR( i2c_ak8975_process_compass_axes(ak8975_handle, axes_data, compass_axes), err, TAG, "compass axes processing failed" );

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ak8975_self_test(i2c_ak8975_handle_t ak8975_handle, i2c_ak8975_compass_axes_data_t *compass_axes) {
    esp_err_t               ret = ESP_OK;
    i2c_ak8975_axes_data_t  axes_data;

    /* validate arguments */
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_POWER_DOWN), err, TAG, "power down mode failed" );

    /* attempt to enable self test control */
    ESP_GOTO_ON_ERROR( i2c_ak8975_set_self_test(ak8975_handle, true), err, TAG, "self test enabled failed" );
    
    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_SELF_TEST), err, TAG, "self test mode failed" );

    /* attempt to read data axes */
    ESP_GOTO_ON_ERROR( i2c_ak8975_get_axes_data(ak8975_handle, &axes_data), err, TAG, "read axes data failed" );

    /* attempt to disable self test control */
    ESP_GOTO_ON_ERROR( i2c_ak8975_set_self_test(ak8975_handle, false), err, TAG, "self test disabled failed" );

    /* attempt to process compass axes sensitivity adjustments */
    ESP_GOTO_ON_ERROR( i2c_ak8975_process_compass_axes(ak8975_handle, axes_data, compass_axes), err, TAG, "compass axes processing failed" );

    return ESP_OK;

    err:
        /* attempt to disable self test control */
        i2c_ak8975_set_self_test(ak8975_handle, false);

        return ret;
}

esp_err_t i2c_ak8975_get_data_status(i2c_ak8975_handle_t ak8975_handle, bool *ready) {
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to read status 1 register */
    ESP_RETURN_ON_ERROR( i2c_ak8975_get_status1_register(ak8975_handle), TAG, "read status 1 register (data ready state) failed" );

    /* set ready state */
    *ready = ak8975_handle->status1_reg.bits.data_ready;

    return ESP_OK;
}

esp_err_t i2c_ak8975_get_error_status(i2c_ak8975_handle_t ak8975_handle, bool *error) {
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to read status 2 register */
    ESP_RETURN_ON_ERROR( i2c_ak8975_get_status2_register(ak8975_handle), TAG, "read status 2 register failed" );

    /* set error state */
    *error = ak8975_handle->status2_reg.bits.data_error;

    return ESP_OK;
}

esp_err_t i2c_ak8975_get_overflow_status(i2c_ak8975_handle_t ak8975_handle, bool *overflow) {
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to read status 2 register */
    ESP_RETURN_ON_ERROR( i2c_ak8975_get_status2_register(ak8975_handle), TAG, "read status 2 register failed" );

    /* set overflow state */
    *overflow = ak8975_handle->status2_reg.bits.sensor_overflow;

    return ESP_OK;
}

esp_err_t i2c_ak8975_power_down(i2c_ak8975_handle_t ak8975_handle) {
    ESP_ARG_CHECK( ak8975_handle );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( i2c_ak8975_set_mode(ak8975_handle, I2C_AK8975_OPMODE_POWER_DOWN), TAG, "power down mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ak8975_rm(i2c_ak8975_handle_t ak8975_handle) {
    ESP_ARG_CHECK( ak8975_handle );

    return i2c_master_bus_rm_device(ak8975_handle->i2c_dev_handle);
}
