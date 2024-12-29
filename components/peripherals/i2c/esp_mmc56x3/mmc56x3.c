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
 * @file mmc56x3.c
 *
 * ESP-IDF driver for MMC56X3 Magnetic sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "mmc56x3.h"
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
 * MMC56X3 definitions
*/

#define I2C_MMC56X3_REG_XOUT_0_R            UINT8_C(0x00)   //!< mmc56x3 I2C Xout[19:12]
#define I2C_MMC56X3_REG_XOUT_1_R            UINT8_C(0x01)   //!< mmc56x3 I2C Xout[11:4]
#define I2C_MMC56X3_REG_YOUT_0_R            UINT8_C(0x02)   //!< mmc56x3 I2C Yout[19:12]
#define I2C_MMC56X3_REG_YOUT_1_R            UINT8_C(0x03)   //!< mmc56x3 I2C Yout[11:4]
#define I2C_MMC56X3_REG_ZOUT_0_R            UINT8_C(0x04)   //!< mmc56x3 I2C Zout[19:12]
#define I2C_MMC56X3_REG_ZOUT_1_R            UINT8_C(0x05)   //!< mmc56x3 I2C Zout[11:4]
#define I2C_MMC56X3_REG_XOUT_2_R            UINT8_C(0x06)   //!< mmc56x3 I2C Xout[3:0]
#define I2C_MMC56X3_REG_YOUT_2_R            UINT8_C(0x07)   //!< mmc56x3 I2C Yout[3:0]
#define I2C_MMC56X3_REG_ZOUT_2_R            UINT8_C(0x08)   //!< mmc56x3 I2C Zout[3:0]
#define I2C_MMC56X3_REG_TOUT_R              UINT8_C(0x09)   //!< mmc56x3 I2C temperature output
#define I2C_MMC56X3_REG_STATUS_1_R          UINT8_C(0x18)   //!< mmc56x3 I2C device status 1
#define I2C_MMC56X3_REG_ODR_W               UINT8_C(0x1a)   //!< mmc56x3 I2C output data rate
#define I2C_MMC56X3_REG_CONTROL_0_W         UINT8_C(0x1b)   //!< mmc56x3 I2C control register 0
#define I2C_MMC56X3_REG_CONTROL_1_W         UINT8_C(0x1c)   //!< mmc56x3 I2C control register 1
#define I2C_MMC56X3_REG_CONTROL_2_W         UINT8_C(0x1d)   //!< mmc56x3 I2C control register 2
#define I2C_MMC56X3_REG_ST_X_TH_W           UINT8_C(0x1e)   //!< mmc56x3 I2C x-axis selftest threshold
#define I2C_MMC56X3_REG_ST_Y_TH_W           UINT8_C(0x1f)   //!< mmc56x3 I2C y-axis selftest threshold
#define I2C_MMC56X3_REG_ST_Z_TH_W           UINT8_C(0x20)   //!< mmc56x3 I2C z-axis selftest threshold
#define I2C_MMC56X3_REG_ST_X_SV_RW          UINT8_C(0x27)   //!< mmc56x3 I2C x-axis selftest set-value
#define I2C_MMC56X3_REG_ST_Y_SV_RW          UINT8_C(0x28)   //!< mmc56x3 I2C y-axis selftest set-value
#define I2C_MMC56X3_REG_ST_Z_SV_RW          UINT8_C(0x29)   //!< mmc56x3 I2C z-axis selftest set-value
#define I2C_MMC56X3_REG_PRODUCT_ID_R        UINT8_C(0x39)   //!< mmc56x3 I2C product identifier


#define I2C_MMC56X3_POWERUP_DELAY_MS        UINT16_C(25)
#define I2C_MMC56X3_APPSTART_DELAY_MS       UINT16_C(10)           //!< mmc56x3 I2C delay in milliseconds app-start
#define I2C_MMC56X3_RESET_DELAY_MS          UINT16_C(20)           //!< mmc56x3 I2C delay in milliseconds after reset
#define I2C_MMC56X3_SETRESET_DELAY_MS       UINT16_C(1)            //!< mmc56x3 I2C delay in milliseconds after set-reset transaction
#define I2C_MMC56X3_WRITE_DELAY_MS          UINT16_C(1)            //!< mmc56x3 I2C delay in milliseconds after write transaction
#define I2C_MMC56X3_DATA_READY_DELAY_MS     UINT16_C(1)            //!< mmc56x3 1ms when checking data ready in a loop
#define I2C_MMC56X3_DATA_POLL_TIMEOUT_MS    UINT16_C(100)          //!< mmc56x3 100ms timeout when making a measurement
#define I2C_MMC56X3_CMD_DELAY_MS            UINT16_C(5)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "mmc56x3";

/*
* functions and subrountines
*/

esp_err_t i2c_mmc56x3_get_status_register(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_STATUS_1_R, &mmc56x3_handle->status_reg.reg), TAG, "mmc56x3 read status register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_control0_register(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_control0_register_t control0_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* set reserved to 0 */
    i2c_mmc56x3_control0_register_t control0 = { .reg = control0_reg.reg };
    control0.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_CONTROL_0_W, control0.reg), TAG, "write control 0 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    /* reset auto-reset parameters */
    control0.bits.continuous_freq_enabled = false;
    control0.bits.auto_st_enabled         = false;
    control0.bits.do_reset                = false;
    control0.bits.do_set                  = false;
    control0.bits.sample_m                = false;
    control0.bits.sample_t                = false;

    /* set device handle control 0 register */
    mmc56x3_handle->control0_reg.reg = control0.reg;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_control1_register(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_control1_register_t control1_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    i2c_mmc56x3_control1_register_t control1 = { .reg = control1_reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_CONTROL_1_W, control1_reg.reg), TAG, "write control 1 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    /* set device handle control 1 register */
    mmc56x3_handle->control1_reg.reg = control1.reg;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_control2_register(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_control2_register_t control2_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* set reserved to 0 */
    i2c_mmc56x3_control2_register_t control2 = { .reg = control2_reg.reg };
    control2.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_CONTROL_2_W, control2.reg), TAG, "write control 2 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    /* set device handle control 2 register */
    mmc56x3_handle->control2_reg.reg = control2_reg.reg;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_get_product_id_register(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_PRODUCT_ID_R, &mmc56x3_handle->product_id), TAG, "mmc56x3 read product identifier register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_init(i2c_master_bus_handle_t bus_handle, const i2c_mmc56x3_config_t *mmc56x3_config, i2c_mmc56x3_handle_t *mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && mmc56x3_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, mmc56x3_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, mmc56x3 device handle initialization failed", mmc56x3_config->dev_config.device_address);

    /* validate data rate if continuous mode is enabled */
    if(mmc56x3_config->continuous_mode_enabled == true) {
        ESP_GOTO_ON_FALSE(mmc56x3_config->data_rate > 0, ESP_ERR_INVALID_ARG, err, TAG, "data rate (odr) must be non-zero in continuous measurement mode, init failed");
    }

    /* validate memory availability for handle */
    i2c_mmc56x3_handle_t out_handle = (i2c_mmc56x3_handle_t)calloc(1, sizeof(i2c_mmc56x3_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mmc56x3 device, init failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = mmc56x3_config->dev_config.device_address,
        .scl_speed_hz       = mmc56x3_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    /* copy configuration */
    out_handle->auto_sr_enabled         = mmc56x3_config->auto_sr_enabled;
    out_handle->continuous_mode_enabled = mmc56x3_config->continuous_mode_enabled;
    out_handle->data_rate               = mmc56x3_config->data_rate;
    out_handle->measurement_bandwidth   = mmc56x3_config->measurement_bandwidth;
    out_handle->declination             = mmc56x3_config->declination;

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR( i2c_mmc56x3_reset(out_handle), err_handle, TAG, "unable to issue soft-reset, init failed" );

    /* configure the device */
    ESP_GOTO_ON_ERROR( i2c_mmc56x3_set_measure_bandwidth(out_handle, out_handle->measurement_bandwidth), err_handle, TAG, "unable to set measure bandwidth, init failed" );
    ESP_GOTO_ON_ERROR( i2c_mmc56x3_set_data_rate(out_handle, out_handle->data_rate), err_handle, TAG, "unable to issue soft-reset, init failed" );
    ESP_GOTO_ON_ERROR( i2c_mmc56x3_set_measure_mode(out_handle, out_handle->continuous_mode_enabled), err_handle, TAG, "unable to set measure mode, init failed" );

    /* set device handle */
    *mmc56x3_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_mmc56x3_get_temperature(i2c_mmc56x3_handle_t mmc56x3_handle, float *const temperature) {
    esp_err_t ret             = ESP_OK;
    uint64_t  start_time      = 0;
    bool      data_is_ready   = false;
    uint8_t   temp_reg;

    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* validate mode */
    if(mmc56x3_handle->control2_reg.bits.continuous_enabled == false) {
        /* copy register */
        i2c_mmc56x3_control0_register_t ctrl0 = { .reg = mmc56x3_handle->control0_reg.reg };
        
        /* trigger temperature measurement */
        ctrl0.bits.sample_t = true;

        /* attempt to write control 0 register */
        ESP_GOTO_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), err, TAG, "write magnetic sample trigger for get magnetic failed." );
    }

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to poll if data is ready or timeout */
        ESP_GOTO_ON_ERROR( i2c_mmc56x3_get_temperature_data_status(mmc56x3_handle, &data_is_ready), err, TAG, "temperature data ready read for get temperature failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (I2C_MMC56X3_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data read transactions */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_TOUT_R, &temp_reg), err, TAG, "read temperature failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    *temperature = temp_reg;
    *temperature *= 0.8; // 0.8C / LSB
    *temperature -= 75;  // 0 value is -75

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_mmc56x3_get_magnetic_axes(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_magnetic_axes_data_t *const magnetic_axes_data) {
    esp_err_t                       ret             = ESP_OK;
    uint64_t                        start_time      = 0;
    bool                            data_is_ready   = false;
    i2c_mmc56x3_control0_register_t ctrl0;
    i2c_uint8_t                     tx = { I2C_MMC56X3_REG_XOUT_0_R };
    uint8_t                         rx[9];

    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* validate mode */
    if(mmc56x3_handle->control2_reg.bits.continuous_enabled == false) {
        /* copy register */
        ctrl0.reg = mmc56x3_handle->control0_reg.reg;

        /* trigger magnetic measurement */
        ctrl0.bits.sample_m = true;

        /* attempt to write control 0 register */
        ESP_GOTO_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), err, TAG, "write magnetic sample trigger for get magnetic axes failed." );
    }

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to poll if data is ready or timeout */
        ESP_GOTO_ON_ERROR( i2c_mmc56x3_get_magnetic_data_status(mmc56x3_handle, &data_is_ready), err, TAG, "magnetic data ready read for get magnetic axes failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (I2C_MMC56X3_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data write-read transactions */
    ESP_GOTO_ON_ERROR( i2c_master_transmit_receive(mmc56x3_handle->i2c_dev_handle, tx, I2C_UINT8_SIZE, rx, sizeof(rx), I2C_XFR_TIMEOUT_MS), err, TAG, "read magnetic axes failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    // convert bytes (20-bit) to int32_t
    int32_t x_axis = (uint32_t)rx[0] << 12 | (uint32_t)rx[1] << 4 | (uint32_t)rx[6] >> 4;
    int32_t y_axis = (uint32_t)rx[2] << 12 | (uint32_t)rx[3] << 4 | (uint32_t)rx[7] >> 4;
    int32_t z_axis = (uint32_t)rx[4] << 12 | (uint32_t)rx[5] << 4 | (uint32_t)rx[8] >> 4;

    // scale to mG by LSB (0.0625mG per LSB resolution) in datasheet (20-bit 524288 counts)
    magnetic_axes_data->x_axis = (float)(x_axis - 524288) * 0.0625;
    magnetic_axes_data->y_axis = (float)(y_axis - 524288) * 0.0625;
    magnetic_axes_data->z_axis = (float)(z_axis - 524288) * 0.0625;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_mmc56x3_get_magnetic_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_get_status_register(mmc56x3_handle), TAG, "read status register for get magnetic data status failed" );

    /* set ready state */
    *ready = mmc56x3_handle->status_reg.bits.data_ready_m;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_get_temperature_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_get_status_register(mmc56x3_handle), TAG, "read status register for get temperature data status failed" );

    /* set ready state */
    *ready = mmc56x3_handle->status_reg.bits.data_ready_t;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_get_data_status(i2c_mmc56x3_handle_t mmc56x3_handle, bool *const magnetic_ready, bool *const temperature_ready) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_get_status_register(mmc56x3_handle), TAG, "read status register for get temperature data status failed" );

    /* set ready state */
    *magnetic_ready     = mmc56x3_handle->status_reg.bits.data_ready_m;
    *temperature_ready  = mmc56x3_handle->status_reg.bits.data_ready_t;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_measure_mode(i2c_mmc56x3_handle_t mmc56x3_handle, const bool continuous) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* validate data rate if continuous mode is enabled */
    if(continuous == true) {
        ESP_RETURN_ON_FALSE(mmc56x3_handle->data_rate > 0, ESP_ERR_INVALID_ARG, TAG, "data rate (odr) must be non-zero in continuous measurement mode, set mode failed");
    }

    /* copy registers */
    i2c_mmc56x3_control0_register_t ctrl0 = { .reg = mmc56x3_handle->control0_reg.reg };
    i2c_mmc56x3_control2_register_t ctrl2 = { .reg = mmc56x3_handle->control2_reg.reg };

    ctrl0.bits.auto_sr_enabled = mmc56x3_handle->auto_sr_enabled;

    if(continuous == true) {
        ctrl0.bits.continuous_freq_enabled  = true; // turn on cmm_freq_en bit
        ctrl2.bits.continuous_enabled       = true; // turn on cmm_en bit
    } else {
        ctrl0.bits.continuous_freq_enabled  = false; // turn off cmm_freq_en bit
        ctrl2.bits.continuous_enabled       = false; // turn off cmm_en bit
    }

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), TAG, "write control 0 register, set mode failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control2_register(mmc56x3_handle, ctrl2), TAG, "write control 2 register, set mode failed" );

    mmc56x3_handle->continuous_mode_enabled = continuous;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_data_rate(i2c_mmc56x3_handle_t mmc56x3_handle, const uint16_t rate) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy register */
    i2c_mmc56x3_control2_register_t ctrl2 = { .reg = mmc56x3_handle->control2_reg.reg };

    /* only 0~255 and 1000 are valid, so just move any high rates to 1000 */
    if(rate > 255) {
        /* set odr range */
        uint8_t odr = 255;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed" );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

        /* enable hpower */
        ctrl2.bits.h_power_enabled = true;

        /* attempt control 2 register write */
        ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control2_register(mmc56x3_handle, ctrl2), TAG, "write control 2 register failed" );
    } else {
        /* set odr range */
        uint8_t odr = (uint8_t)rate;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed" );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

        /* disable hpower */
        ctrl2.bits.h_power_enabled = false;

        /* attempt control 2 register write */
        ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control2_register(mmc56x3_handle, ctrl2), TAG, "write control 2 register failed" );
    }

    mmc56x3_handle->data_rate = rate;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_measure_bandwidth(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_measurement_times_t bandwidth) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy register */
    i2c_mmc56x3_control1_register_t ctrl1 = { .reg = mmc56x3_handle->control1_reg.reg };
    ctrl1.bits.bandwidth = bandwidth;

    /* attempt control 1 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control1_register(mmc56x3_handle, ctrl1), TAG, "unable to write control 1 register, set measure bandwidth failed" );

    mmc56x3_handle->measurement_bandwidth = bandwidth;

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_enable_periodical_set(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_measurement_samples_t samples) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy registers */
    i2c_mmc56x3_control0_register_t ctrl0 = { .reg = mmc56x3_handle->control0_reg.reg };
    i2c_mmc56x3_control2_register_t ctrl2 = { .reg = mmc56x3_handle->control2_reg.reg };

    ctrl0.bits.auto_sr_enabled = true;
    ctrl2.bits.periodical_set_enabled = true;
    ctrl2.bits.periodical_set_samples = samples;

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), TAG, "write control 0 register, enable periodical set failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control2_register(mmc56x3_handle, ctrl2), TAG, "write control 2 register, enable periodical set failed" );

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_disable_periodical_set(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy registers */
    i2c_mmc56x3_control0_register_t ctrl0 = { .reg = mmc56x3_handle->control0_reg.reg };
    i2c_mmc56x3_control2_register_t ctrl2 = { .reg = mmc56x3_handle->control2_reg.reg };

    if(mmc56x3_handle->auto_sr_enabled == false) ctrl0.bits.auto_sr_enabled = false;
    ctrl2.bits.periodical_set_enabled = false;
    ctrl2.bits.periodical_set_samples = I2C_MMC56X3_MEAS_SAMPLE_1;

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), TAG, "write control 0 register, enable periodical set failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control2_register(mmc56x3_handle, ctrl2), TAG, "write control 2 register, enable periodical set failed" );

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_magnetic_set_reset(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy register */
    i2c_mmc56x3_control0_register_t ctrl0 = { .reg = mmc56x3_handle->control0_reg.reg };
    ctrl0.bits.do_set   = true; // turn on set bit
    ctrl0.bits.do_reset = true; // turn on reset bit

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control0_register(mmc56x3_handle, ctrl0), TAG, "write control 0 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_SETRESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_selftest_thresholds(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_selftest_axes_data_t axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt i2c write transactions for each axis */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_X_TH_W, axes_data.x_axis), TAG, "write self-test x-axis threshold register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Y_TH_W, axes_data.y_axis), TAG, "write self-test y-axis threshold register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Z_TH_W, axes_data.z_axis), TAG, "write self-test z-axis threshold register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_get_selftest_set_values(i2c_mmc56x3_handle_t mmc56x3_handle, i2c_mmc56x3_selftest_axes_data_t *const axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt i2c read transactions for each axis */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_X_SV_RW, &axes_data->x_axis), TAG, "read self-test x-axis set value register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Y_SV_RW, &axes_data->y_axis), TAG, "read self-test y-axis set value register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Z_SV_RW, &axes_data->z_axis), TAG, "read self-test z-axis set value register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_set_selftest_set_values(i2c_mmc56x3_handle_t mmc56x3_handle, const i2c_mmc56x3_selftest_axes_data_t axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* attempt i2c write transactions for each axis */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_X_SV_RW, axes_data.x_axis), TAG, "write self-test x-axis set value register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Y_SV_RW, axes_data.y_axis), TAG, "write self-test y-axis set value register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(mmc56x3_handle->i2c_dev_handle, I2C_MMC56X3_REG_ST_Z_SV_RW, axes_data.z_axis), TAG, "write self-test z-axis set value register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_reset(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* copy register */
    i2c_mmc56x3_control1_register_t ctrl1 = { .reg = mmc56x3_handle->control1_reg.reg };

    /* set soft-reset to true */
    ctrl1.bits.sw_reset = true;

    /* attempt control 1 register write */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_control1_register(mmc56x3_handle, ctrl1), TAG, "write control 1 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MMC56X3_RESET_DELAY_MS));

    /* set handle registers to defaults */
    mmc56x3_handle->status_reg.reg   = 0;
    mmc56x3_handle->control0_reg.reg = 0;
    mmc56x3_handle->control1_reg.reg = 0;
    mmc56x3_handle->control2_reg.reg = 0;

    /* attempt magnet set reset */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_magnetic_set_reset(mmc56x3_handle), TAG, "magnetic set-reset failed" );

    /* attempt to set mode */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_set_measure_mode(mmc56x3_handle, false), TAG, "disable continuous mode set failed" );

    return ESP_OK;
}

esp_err_t i2c_mmc56x3_remove(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(mmc56x3_handle->i2c_dev_handle);
}

esp_err_t i2c_mmc56x3_delete(i2c_mmc56x3_handle_t mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( mmc56x3_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_mmc56x3_remove(mmc56x3_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(mmc56x3_handle->i2c_dev_handle) {
        free(mmc56x3_handle->i2c_dev_handle);
        free(mmc56x3_handle);
    }

    return ESP_OK;
}

float i2c_mmc56x3_convert_to_heading(const i2c_mmc56x3_magnetic_axes_data_t magnetic_axes_data) {
    float heading = 0;

    /* honeywell application note AN-203 */
    if(magnetic_axes_data.y_axis > 0.0f) heading = 90.0f - atanf(magnetic_axes_data.x_axis/magnetic_axes_data.y_axis * 180.0f / M_PI);
    if(magnetic_axes_data.y_axis < 0.0f) heading = 270.0f - atanf(magnetic_axes_data.x_axis/magnetic_axes_data.y_axis * 180.0f / M_PI);
    if(magnetic_axes_data.y_axis == 0.0f && magnetic_axes_data.x_axis < 0.0f) heading = 180.0f;
    if(magnetic_axes_data.y_axis == 0.0f && magnetic_axes_data.x_axis > 0.0f) heading = 180.0f;

    return heading;
}

float i2c_mmc56x3_convert_to_true_heading(const float declination, const i2c_mmc56x3_magnetic_axes_data_t magnetic_axes_data) {
    float heading = 0;

    /* honeywell application note AN-203 */
    if(magnetic_axes_data.y_axis > 0.0f) heading = 90.0f - atanf(magnetic_axes_data.x_axis/magnetic_axes_data.y_axis * 180.0f / M_PI);
    if(magnetic_axes_data.y_axis < 0.0f) heading = 270.0f - atanf(magnetic_axes_data.x_axis/magnetic_axes_data.y_axis * 180.0f / M_PI);
    if(magnetic_axes_data.y_axis == 0.0f && magnetic_axes_data.x_axis < 0.0f) heading = 180.0f;
    if(magnetic_axes_data.y_axis == 0.0f && magnetic_axes_data.x_axis > 0.0f) heading = 180.0f;

    return heading + declination;
}

