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
 * @file max31865.c
 *
 * ESP-IDF driver for MAX31865 RTD temperature sensor
 * 
 * https://github.com/UncleRus/esp-idf-lib/blob/master/components/max31865/max31865.c
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "max31865.h"
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
 * MAX31865 definitions
 */

#define I2C_MAX31865_REG_CONFIG             UINT8_C(0x00)
#define I2C_MAX31865_REG_RTD_MSB            UINT8_C(0x01)
#define I2C_MAX31865_REG_HIGH_FAULT_MSB     UINT8_C(0x03)
#define I2C_MAX31865_REG_LOW_FAULT_MSB      UINT8_C(0x05)
#define I2C_MAX31865_REG_FAULT_STATUS       UINT8_C(0x07)

#define I2C_MAX31865_DATA_POLL_TIMEOUT_MS  UINT16_C(100)
#define I2C_MAX31865_DATA_READY_DELAY_MS   UINT16_C(2)
#define I2C_MAX31865_POWERUP_DELAY_MS      UINT16_C(120)
#define I2C_MAX31865_RESET_DELAY_MS        UINT16_C(25)
#define I2C_MAX31865_SETUP_DELAY_MS        UINT16_C(15)
#define I2C_MAX31865_APPSTART_DELAY_MS     UINT16_C(10)    /*!< max31865 delay after initialization before application start-up */
#define I2C_MAX31865_CMD_DELAY_MS          UINT16_C(5)     /*!< max31865 delay before attempting I2C transactions after a command is issued */
#define I2C_MAX31865_TX_RX_DELAY_MS        UINT16_C(10)    /*!< max31865 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "max31865";

typedef struct {
    float a, b;
} i2c_rtd_coeff_t;

static const i2c_rtd_coeff_t i2c_rtd_coeff[] = {
     [I2C_MAX31865_ITS90]         = { .a = 3.9083e-3f, .b = -5.775e-7f },
     [I2C_MAX31865_DIN43760]      = { .a = 3.9848e-3f, .b = -5.8019e-7f },
     [I2C_MAX31865_US_INDUSTRIAL] = { .a = 3.9692e-3f, .b = -5.8495e-7f },
};




esp_err_t i2c_max31865_rm(i2c_max31865_handle_t max31865_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max31865_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(max31865_handle->i2c_dev_handle);
}

esp_err_t i2c_max31865_del(i2c_max31865_handle_t max31865_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max31865_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_max31865_rm(max31865_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(max31865_handle->i2c_dev_handle) {
        free(max31865_handle->i2c_dev_handle);
        free(max31865_handle);
    }

    return ESP_OK;
}