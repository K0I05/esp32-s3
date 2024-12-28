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
 * @file ds18b20.c
 *
 * ESP-IDF driver for DS18B20 temperature sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ds18b20.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <onewire_bus.h>
#include <onewire_cmd.h>
#include <onewire_crc.h>

/*
 * DS18B20 definitions
 */


#define OWB_DS18B20_CMD_TEMP_CONVERT        UINT8_C(0x44)  /*!< initiate a single temperature conversion */
#define OWB_DS18B20_CMD_SCRATCHPAD_WRITE    UINT8_C(0x4E)  /*!< write 3 bytes of data to the device scratchpad at positions 2, 3 and 4 */
#define OWB_DS18B20_CMD_SCRATCHPAD_READ     UINT8_C(0xBE)  /*!< read 9 bytes of data (including CRC) from the device scratchpad */
#define OWB_DS18B20_CMD_SCRATCHPAD_COPY     UINT8_C(0x48)  /*!< copy the contents of the scratchpad to the device EEPROM */
#define OWB_DS18B20_CMD_EEPROM_RECALL       UINT8_C(0xB8)  /*!< restore alarm trigger values and configuration data from EEPROM to the scratchpad */
#define OWB_DS18B20_CMD_POWER_SUPPLY_READ   UINT8_C(0xB4)  /*!< determine if a device is using parasitic power */

#define OWB_DS18B20_POWERUP_DELAY_MS        UINT16_C(120)
#define OWB_DS18B20_RESET_DELAY_MS          UINT16_C(25)
#define OWB_DS18B20_APPSTART_DELAY_MS       UINT16_C(10)    /*!< ds18b20 delay after initialization before application start-up */



/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "ds18b20";

/**
 * @brief Writes command to DS18B20.
 * 
 * @param ds18b20_handle DS18B20 device handle.
 * @param cmd DS18B20 command value.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t owb_ds18b20_send_command(owb_ds18b20_handle_t ds18b20_handle, const uint8_t cmd) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    // send command
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &ds18b20_handle->owb_dev_address, sizeof(ds18b20_handle->owb_dev_address));
    tx_buffer[sizeof(ds18b20_handle->owb_dev_address) + 1] = cmd;

    ESP_RETURN_ON_ERROR( onewire_bus_write_bytes(ds18b20_handle->owb_bus_handle, tx_buffer, sizeof(tx_buffer)), TAG, "unable to write bytes, send command failed" );

    return ESP_OK;
}

esp_err_t owb_ds18b20_init(onewire_device_t *device, const owb_ds18b20_config_t *ds18b20_config, owb_ds18b20_handle_t *ds18b20_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( device && ds18b20_config );

    // check ROM ID, the family code of DS18B20 is 0x28
    ESP_RETURN_ON_FALSE( (device->address & 0xFF) == 0x28, ESP_ERR_NOT_SUPPORTED, TAG, "%016llX is not a DS18B20 device", device->address );

    /* validate memory availability for handle */
    owb_ds18b20_handle_t out_handle = (owb_ds18b20_handle_t)calloc(1, sizeof(owb_ds18b20_t));
    ESP_RETURN_ON_FALSE( out_handle, ESP_ERR_NO_MEM, TAG, "no memory for device, ds18b20 device handle initialization failed" );

    /* copy configuration */
    out_handle->owb_bus_handle  = device->bus;
    out_handle->owb_dev_address = device->address;
    out_handle->resolution      = ds18b20_config->resolution;

    /* set temperature resolution */
    ESP_RETURN_ON_ERROR( owb_ds18b20_set_resolution(out_handle, out_handle->resolution), TAG, "write temperature resolution failed" );

    /* set device handle */
    *ds18b20_handle = out_handle;

    return ESP_OK;
}

esp_err_t owb_ds18b20_get_measurement(owb_ds18b20_handle_t ds18b20_handle, float *const temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    ESP_RETURN_ON_ERROR( owb_ds18b20_trigger_temperature_conversion(ds18b20_handle), TAG, "trigger temperature conversion failed" );
    ESP_RETURN_ON_ERROR( owb_ds18b20_get_temperature(ds18b20_handle, temperature), TAG, "read temperature failed" );

    return ESP_OK;
}

esp_err_t owb_ds18b20_get_temperature(owb_ds18b20_handle_t ds18b20_handle, float *const temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(ds18b20_handle->owb_bus_handle), TAG, "reset bus error" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( owb_ds18b20_send_command(ds18b20_handle, OWB_DS18B20_CMD_SCRATCHPAD_READ), TAG, "send DS18B20_CMD_READ_SCRATCHPAD failed" );

    // read scratchpad data
    owb_ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(ds18b20_handle->owb_bus_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "error while reading scratchpad data" );

    // check crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");

    const uint8_t lsb_mask[4] = {0x07, 0x03, 0x01, 0x00}; // mask bits not used in low resolution
    uint8_t lsb_masked = scratchpad.temp_lsb & (~lsb_mask[scratchpad.configuration >> 5]);

    // combine the MSB and masked LSB into a signed 16-bit integer
    int16_t temperature_raw = (((int16_t)scratchpad.temp_msb << 8) | lsb_masked);

    // convert the raw temperature to a float,
    *temperature = temperature_raw / 16.0f;

    return ESP_OK;
}

esp_err_t owb_ds18b20_trigger_temperature_conversion(owb_ds18b20_handle_t ds18b20_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(ds18b20_handle->owb_bus_handle), TAG, "reset bus failed" );

    // send command: DS18B20_CMD_CONVERT_TEMP
    ESP_RETURN_ON_ERROR( owb_ds18b20_send_command(ds18b20_handle, OWB_DS18B20_CMD_TEMP_CONVERT), TAG, "send DS18B20_CMD_CONVERT_TEMP failed" );

    // delay proper time for temperature conversion
    const uint16_t delays_ms[] = { 100, 200, 400, 800 };
    vTaskDelay(pdMS_TO_TICKS(delays_ms[ds18b20_handle->resolution]));

    return ESP_OK;
}

esp_err_t owb_ds18b20_set_resolution(owb_ds18b20_handle_t ds18b20_handle, const owb_ds18b20_resolutions_t resolution) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(ds18b20_handle->owb_bus_handle), TAG, "reset bus error" );

    // send command: DS18B20_CMD_WRITE_SCRATCHPAD
    ESP_RETURN_ON_ERROR( owb_ds18b20_send_command(ds18b20_handle, OWB_DS18B20_CMD_SCRATCHPAD_WRITE), TAG, "send DS18B20_CMD_WRITE_SCRATCHPAD failed" );

    // write new resolution to scratchpad
    const uint8_t resolution_data[] = { 0x1F, 0x3F, 0x5F, 0x7F };
    uint8_t tx_buffer[3] = {0};
    tx_buffer[0] = ds18b20_handle->th_user1;
    tx_buffer[1] = ds18b20_handle->tl_user2;
    tx_buffer[2] = resolution_data[resolution];
    ESP_RETURN_ON_ERROR( onewire_bus_write_bytes(ds18b20_handle->owb_bus_handle, tx_buffer, sizeof(tx_buffer)), TAG, "send new resolution failed" );

    ds18b20_handle->resolution = resolution;

    return ESP_OK;
}

esp_err_t owb_ds18b20_delete(owb_ds18b20_handle_t ds18b20_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ds18b20_handle );

    /* validate handle instance and free handles */
    if(ds18b20_handle) {
        free(ds18b20_handle);
    }

    return ESP_OK;
}