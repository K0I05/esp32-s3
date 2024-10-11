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
 * @file as7341.c
 *
 * ESP-IDF driver for AS7341 11-channel spectrometer (350nm to 1000nm)
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "as7341.h"
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
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "as7341";

static inline esp_err_t i2c_as7341_get_integration_time(i2c_as7341_handle_t as7341_handle, uint32_t *time) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read astep and atime registers */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_astep_register(as7341_handle), TAG, "read astep register for get integration time failed" );
    ESP_RETURN_ON_ERROR( i2c_as7341_get_atime_register(as7341_handle), TAG, "read atime register for get integration time failed" );

    /* compute integration time */
    *time = (as7341_handle->atime_reg + 1) * (as7341_handle->astep_reg + 1) * 2.78 / 1000;

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for flicker detection.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_setup_smux(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write config transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x00, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x01, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x03, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x04, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x05, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x06, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x07, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x08, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x09, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0a, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0c, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0d, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0e, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0f, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x10, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x11, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x13, 0x60), TAG, "write flicker connected to ADC5 to left of 0x13 register failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for low channels F1-F4, Clear and NIR.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_setup_smux_lo_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write config transactions (F1, F2, F3, F4, NIR, CLEAR) */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x00, 0x30), TAG, "write F3 left set to ADC2 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x01, 0x01), TAG, "write F1 left set to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x03, 0x00), TAG, "write F8 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x04, 0x00), TAG, "write F6 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x05, 0x42), TAG, "write F4 left connected to ADC3/F2 connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x06, 0x00), TAG, "write F5 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x07, 0x00), TAG, "write F7 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x08, 0x50), TAG, "write CLEAR connected ADC4 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x09, 0x00), TAG, "write F5 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0a, 0x00), TAG, "write F7 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0c, 0x20), TAG, "write F2 right connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0d, 0x04), TAG, "write F4 right connected to ADC3 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0e, 0x00), TAG, "write F6/F8 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0f, 0x30), TAG, "write F3 right connected to ADC2 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x10, 0x01), TAG, "write F1 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x11, 0x50), TAG, "write CLEAR right connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x13, 0x06), TAG, "write NIR connected to ADC5 register failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for high channels F5-F8, Clear and NIR.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_setup_smux_hi_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write config transactions (F5, F6, F7, F8, NIR, CLEAR) */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x00, 0x00), TAG, "write F3 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x01, 0x00), TAG, "write F1 left disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x03, 0x40), TAG, "write F8 left connected to ADC3 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x04, 0x02), TAG, "write F6 left connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x05, 0x00), TAG, "write F4/F2 disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x06, 0x10), TAG, "write F5 left connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x07, 0x03), TAG, "write F7 left connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x08, 0x50), TAG, "write CLEAR connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x09, 0x10), TAG, "write F5 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0a, 0x03), TAG, "write F7 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0c, 0x00), TAG, "write F2 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0d, 0x00), TAG, "write F4 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0e, 0x24), TAG, "write F8 right connected to ADC2/F6 right connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x0f, 0x00), TAG, "write F3 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x10, 0x00), TAG, "write F1 right disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x11, 0x50), TAG, "write CLEAR right connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, 0x13, 0x06), TAG, "write NIR connected to ADC5 register failed" );

    return ESP_OK;
}

static inline esp_err_t i2c_as7341_set_smux_lo_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    ESP_RETURN_ON_ERROR( i2c_as7341_disable_spectral_measurement(as7341_handle), TAG, "disable spectral measurement for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_set_smux_command(as7341_handle, I2C_AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_setup_smux_lo_channels(as7341_handle), TAG, "setup SMUX low channels for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_enable_smux(as7341_handle), TAG, "enable SMUX for set SMUX low channels failed" );

    return ESP_OK;
}

static inline esp_err_t i2c_as7341_set_smux_hi_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    ESP_RETURN_ON_ERROR( i2c_as7341_disable_spectral_measurement(as7341_handle), TAG, "disable spectral measurement for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_set_smux_command(as7341_handle, I2C_AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_setup_smux_hi_channels(as7341_handle), TAG, "setup SMUX high channels for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_enable_smux(as7341_handle), TAG, "enable SMUX for set SMUX high channels failed" );

    return ESP_OK;
}



esp_err_t i2c_as7341_get_led_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_LED, &as7341_handle->led_reg.reg), TAG, "read LED register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_led_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_led_register_t led_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_LED, led_reg.reg), TAG, "write LED register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_led_register(as7341_handle), TAG, "read LED register for set LED register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_device_status_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_DEV_STATUS, &as7341_handle->dev_status_reg.reg), TAG, "read device status register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_astatus_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTATUS, &as7341_handle->astatus_reg.reg), TAG, "read astatus register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_status2_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_STATUS2, &as7341_handle->status2_reg.reg), TAG, "read status 2 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_enable_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ENABLE, &as7341_handle->enable_reg.reg), TAG, "read enabled register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_enable_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_enable_register_t enable_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set reserved bits */
    enable_reg.bits.reserved1 = 0;
    enable_reg.bits.reserved2 = 0;
    enable_reg.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ENABLE, enable_reg.reg), TAG, "write enable register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_enable_register(as7341_handle), TAG, "read enable register for set enable register failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_get_auxiliary_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_AUXID, &as7341_handle->aux_id_reg.reg), TAG, "read auxiliary id register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_revision_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_REVID, &as7341_handle->revision_id_reg.reg), TAG, "read revision id register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_part_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ID, &as7341_handle->part_id_reg.reg), TAG, "read part id register failed" );

    return ESP_OK;
}




esp_err_t i2c_as7341_get_config_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG, &as7341_handle->config_reg.reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config_register_t config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set reserved bits */
    config_reg.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG, config_reg.reg), TAG, "write configuration register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config_register(as7341_handle), TAG, "read configuration register for set configuration register failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_get_config0_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG0, &as7341_handle->config0_reg.reg), TAG, "read configuration 0 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config0_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config0_register_t config0_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set reserved bits */
    config0_reg.bits.reserved1 = 0;
    config0_reg.bits.reserved2 = 0;
    config0_reg.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG0, config0_reg.reg), TAG, "write configuration 0 register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config0_register(as7341_handle), TAG, "read configuration 0 register for set configuration 0 register failed" );

    return ESP_OK;
}



esp_err_t i2c_as7341_get_config1_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG1, &as7341_handle->config1_reg.reg), TAG, "read configuration 1 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config1_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config1_register_t config1_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set reserved bits */
    config1_reg.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG1, config1_reg.reg), TAG, "write configurtion 1 register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for set configuration 1 register failed" );

    return ESP_OK;
}



esp_err_t i2c_as7341_get_config6_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG6, &as7341_handle->config6_reg.reg), TAG, "read configuration 6 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config6_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config6_register_t config6_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set reserved bits */
    config6_reg.bits.reserved1 = 0;
    config6_reg.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG6, config6_reg.reg), TAG, "write configuration 6 register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config6_register(as7341_handle), TAG, "read configuration 6 register for set configuration 6 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_atime_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ATIME, &as7341_handle->atime_reg), TAG, "read atime register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_atime_register(i2c_as7341_handle_t as7341_handle, uint8_t atime_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ATIME, atime_reg), TAG, "write atime register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_atime_register(as7341_handle), TAG, "read atime register for set atime register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_astep_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTEP_L, &as7341_handle->astep_reg), TAG, "read astep register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_astep_register(i2c_as7341_handle_t as7341_handle, uint16_t astep_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTEP_L, astep_reg), TAG, "write astep register failed" );

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_astep_register(as7341_handle), TAG, "read astep register for set astep register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_smux_command(i2c_as7341_handle_t as7341_handle, i2c_as7341_smux_commands_t command) {
    i2c_as7341_config6_register_t config6_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set smux command */
    config6_reg.bits.smux_command = command;

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config6_register(as7341_handle, config6_reg), TAG, "write configuration 6 register for set smux command failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_atime(i2c_as7341_handle_t as7341_handle, uint8_t atime) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_atime_register(as7341_handle, atime), TAG, "write atime register for set atime failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_astep(i2c_as7341_handle_t as7341_handle, uint16_t astep) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_astep_register(as7341_handle, astep), TAG, "write astep register for set astep failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_spectral_gain(i2c_as7341_handle_t as7341_handle, i2c_as7341_spectral_gains_t gain) {
    i2c_as7341_config1_register_t config1_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set spectral gain */
    config1_reg.bits.spectral_gain = gain;

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config1_register(as7341_handle, config1_reg), TAG, "write configuration 1 register for set spectral gain failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_enable_flicker_detection(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* enable flicker detection */
    enable_reg.bits.flicker_detection_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable flicker detection failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_flicker_detection(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* disable flicker detection */
    enable_reg.bits.flicker_detection_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for disable flicker detection failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_smux(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* enable smux */
    enable_reg.bits.smux_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable SMUX failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_spectral_measurement(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* enable spectral measurement */
    enable_reg.bits.spectral_measurement_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_spectral_measurement(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* disable spectral measurement */
    enable_reg.bits.spectral_measurement_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_power(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* enable power */
    enable_reg.bits.power_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable power failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_power(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_enable_register_t enable_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    enable_reg.reg = as7341_handle->enable_reg.reg;

    /* disable power */
    enable_reg.bits.power_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for disable power failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_hi_register_bank(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_config0_register_t config0_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    config0_reg.reg = as7341_handle->config0_reg.reg;

    /* enable high registers */
    config0_reg.bits.reg_bank_access = false; // 0 or false to access register 0x80 and above

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config0_register(as7341_handle, config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_lo_register_bank(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_config0_register_t config0_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    config0_reg.reg = as7341_handle->config0_reg.reg;

    /* enable low registers */
    config0_reg.bits.reg_bank_access = true; // 1 or true to access register 0x60 to 0x74

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config0_register(as7341_handle, config0_reg), TAG, "write configuration 0 register for enable low registers failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_enable_led(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_config_register_t    config_reg;
    i2c_as7341_led_register_t       led_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy registers from handle */
    config_reg.reg  = as7341_handle->config_reg.reg;
    led_reg.reg     = as7341_handle->led_reg.reg;

    /* enable led */
    config_reg.bits.led_ldr_control_enabled = true;
    led_reg.bits.led_ldr_enabled            = true;

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_lo_register_bank(as7341_handle), TAG, "enable low register bank for enable LED failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config_register(as7341_handle, config_reg), TAG, "write configuration register for enable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( i2c_as7341_set_led_register(as7341_handle, led_reg), TAG, "write led register for enable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_hi_register_bank(as7341_handle), TAG, "enable high register bank for enable LED failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_led(i2c_as7341_handle_t as7341_handle) {
    i2c_as7341_config_register_t    config_reg;
    i2c_as7341_led_register_t       led_reg;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy registers from handle */
    config_reg.reg  = as7341_handle->config_reg.reg;
    led_reg.reg     = as7341_handle->led_reg.reg;

    /* enable led */
    config_reg.bits.led_ldr_control_enabled = false;
    led_reg.bits.led_ldr_enabled            = false;

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_lo_register_bank(as7341_handle), TAG, "enable low register bank for disable LED failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config_register(as7341_handle, config_reg), TAG, "write configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( i2c_as7341_set_led_register(as7341_handle, led_reg), TAG, "write led register for disable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_hi_register_bank(as7341_handle), TAG, "enable high register bank for disable LED failed" );

    return ESP_OK;
}




esp_err_t i2c_as7341_init(i2c_master_bus_handle_t bus_handle, const i2c_as7341_config_t *as7341_config, i2c_as7341_handle_t *as7341_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_as7341_handle_t  out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && as7341_config );

    /* validate memory availability for handle */
    out_handle = (i2c_as7341_handle_t)calloc(1, sizeof(i2c_as7341_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as7341 device, init failed");

    /* set i2c device configuration */
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = as7341_config->dev_config.device_address,
        .scl_speed_hz       = I2C_AS7341_DATA_RATE_HZ,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus for init failed");
    }

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_POWERUP_DELAY_MS));

    /* attempt to enable power */
    ESP_GOTO_ON_ERROR(i2c_as7341_enable_power(out_handle), err, TAG, "enable power for init failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_APPSTART_DELAY_MS));

    /* attempt to read aux id register */
    ESP_GOTO_ON_ERROR(i2c_as7341_get_auxiliary_id_register(out_handle), err, TAG, "read auxiliary id register for init failed");

    /* attempt to read revision id register */
    ESP_GOTO_ON_ERROR(i2c_as7341_get_revision_id_register(out_handle), err, TAG, "read revision id register for init failed");

    /* attempt to read part id register */
    ESP_GOTO_ON_ERROR(i2c_as7341_get_part_id_register(out_handle), err, TAG, "read part id register for init failed");

    /* set device handle */
    *as7341_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}


esp_err_t i2c_as7341_get_adc_measurements(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_adc_data_t *adc_spectral_data) {
    esp_err_t       ret             = ESP_OK;
    uint32_t        integration_time= 0;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    i2c_uint8_t     tx              = { I2C_AS7341_CH0_ADC_DATA_L };
    uint8_t         rx[12]          = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read integration time */
    ESP_GOTO_ON_ERROR( i2c_as7341_get_integration_time(as7341_handle, &integration_time), err, TAG, "read integration time, for get adc measurements failed." );

    // ************ LOW CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( i2c_as7341_set_smux_lo_channels(as7341_handle), err, TAG, "setup of SMUX low channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    ESP_GOTO_ON_ERROR( i2c_as7341_enable_spectral_measurement(as7341_handle), err, TAG, "enable spectral measurement, low channels, for get adc measurements failed." );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data, low channels, is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_as7341_get_data_status(as7341_handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 20) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from low channels */
    ESP_GOTO_ON_ERROR( i2c_master_transmit_receive(as7341_handle->i2c_dev_handle, tx, I2C_UINT8_SIZE, rx, sizeof(rx), I2C_XFR_TIMEOUT_MS), err, TAG, "read low channel measurements for get adc measurements failed" );

    /* set adc data for low channels */
    adc_spectral_data->f1_415nm = rx[0]  | (rx[1] << 8);
    adc_spectral_data->f2_445nm = rx[2]  | (rx[3] << 8);
    adc_spectral_data->f3_480nm = rx[4]  | (rx[5] << 8);
    adc_spectral_data->f4_515nm = rx[6]  | (rx[7] << 8);
    //adc_spectral_data->clear_0  = rx[8]  | (rx[9] << 8);
    //adc_spectral_data->nir_0    = rx[10] | (rx[11] << 8);


    // ************ HIGH CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( i2c_as7341_set_smux_hi_channels(as7341_handle), err, TAG, "setup of SMUX high channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    ESP_GOTO_ON_ERROR( i2c_as7341_enable_spectral_measurement(as7341_handle), err, TAG, "enable spectral measurement, high channels, for get adc measurements failed." );

    /* set start time for timeout monitoring and reset data ready flag */
    start_time = esp_timer_get_time();
    data_is_ready = false;

    /* attempt to poll until data, high channels, is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_as7341_get_data_status(as7341_handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 20) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from high channels */
    ESP_GOTO_ON_ERROR( i2c_master_transmit_receive(as7341_handle->i2c_dev_handle, tx, I2C_UINT8_SIZE, rx, sizeof(rx), I2C_XFR_TIMEOUT_MS), err, TAG, "read high channel measurements for get adc measurements failed" );

    /* set adc data for high channels */
    adc_spectral_data->f5_555nm = rx[0]  | (rx[1] << 8);
    adc_spectral_data->f6_590nm = rx[2]  | (rx[3] << 8);
    adc_spectral_data->f7_630nm = rx[4]  | (rx[5] << 8);
    adc_spectral_data->f8_680nm = rx[6]  | (rx[7] << 8);
    adc_spectral_data->clear    = rx[8]  | (rx[9] << 8);
    adc_spectral_data->nir      = rx[10] | (rx[11] << 8);

    return ESP_OK;

    err:
        return ret;
}


esp_err_t i2c_as7341_get_data_status(i2c_as7341_handle_t as7341_handle, bool *ready) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_status2_register(as7341_handle), TAG, "read status 2 register (data ready state) failed" );

    /* set ready state */
    *ready = as7341_handle->status2_reg.bits.spectral_valid;

    return ESP_OK;
}

esp_err_t i2c_as7341_get_basic_counts(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_adc_data_t adc_data, i2c_as7341_channels_data_t *data) {
    float       gain_val            = 0;
    uint32_t    integration_time    = 0;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read registers (config1, astep, and atime) */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for get integration time failed" );
    ESP_RETURN_ON_ERROR( i2c_as7341_get_astep_register(as7341_handle), TAG, "read astep register for get integration time failed" );
    ESP_RETURN_ON_ERROR( i2c_as7341_get_atime_register(as7341_handle), TAG, "read atime register for get integration time failed" );

    /* determine gain sensitivity */
    switch (as7341_handle->config1_reg.bits.spectral_gain) {
        case I2C_AS7341_SPECTRAL_GAIN_0_5X:
            gain_val = 0.5;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_1X:
            gain_val = 1;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_2X:
            gain_val = 2;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_4X:
            gain_val = 4;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_8X:
            gain_val = 8;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_16X:
            gain_val = 16;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_32X:
            gain_val = 32;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_64X:
            gain_val = 64;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_128X:
            gain_val = 128;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_256X:
            gain_val = 256;
            break;
        case I2C_AS7341_SPECTRAL_GAIN_512X:
            gain_val = 512;
            break;
    }

    /* compute integration time */
    integration_time = ((as7341_handle->atime_reg + 1) * (as7341_handle->astep_reg + 1) * 2.78 / 1000);

    /* convert adc value to basic counts value */
    data->f1_415nm = (float)adc_data.f1_415nm / (gain_val * integration_time);
    data->f2_445nm = (float)adc_data.f2_445nm / (gain_val * integration_time);
    data->f3_480nm = (float)adc_data.f3_480nm / (gain_val * integration_time);
    data->f4_515nm = (float)adc_data.f4_515nm / (gain_val * integration_time);
    data->f5_555nm = (float)adc_data.f5_555nm / (gain_val * integration_time);
    data->f6_590nm = (float)adc_data.f6_590nm / (gain_val * integration_time);
    data->f7_630nm = (float)adc_data.f7_630nm / (gain_val * integration_time);
    data->f8_680nm = (float)adc_data.f8_680nm / (gain_val * integration_time);
    data->clear    = (float)adc_data.clear    / (gain_val * integration_time);
    data->nir      = (float)adc_data.nir      / (gain_val * integration_time);

    return ESP_OK;
}


esp_err_t i2c_as7341_rm(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(as7341_handle->i2c_dev_handle);
}