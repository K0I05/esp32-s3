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
 * @file sgp4x.c
 *
 * ESP-IDF driver for SGP4X sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "sgp4x.h"
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
 * SGP4X definitions
 */
#define I2C_SGP4X_CRC8_G_POLYNOM            UINT8_C(0x31)       //!< sgp4x I2C CRC8 polynomial

#define I2C_SGP4X_CMD_RESET                 UINT16_C(0x0006)
#define I2C_SGP4X_CMD_RESET_                UINT8_C(0x06)       //!< sgp4x I2C soft-reset command - for some reason this is an 1-byte command
#define I2C_SGP4X_CMD_SERIAL_NUMBER         UINT16_C(0x3682)    //!< sgp4x I2C serial number request command
#define I2C_SGP4X_CMD_EXEC_CONDITIONING     UINT16_C(0x2612)    //!< sgp4x I2C
#define I2C_SGP4X_CMD_MEAS_RAW_SIGNALS      UINT16_C(0x2619)    //!< sgp4x I2C
#define I2C_SGP4X_CMD_EXEC_SELF_TEST        UINT16_C(0x280e)    //!< sgp4x I2C
#define I2C_SGP4X_CMD_TURN_HEATER_OFF       UINT16_C(0x3615)    //!< sgp4x I2C

#define I2C_SGP4X_HUMIDITY_COMP_DEFAULT     (float)(50.0)
#define I2C_SGP4X_TEMPERATURE_COMP_DEFAULT  (float)(25.0)

#define I2C_SGP4X_TEMPERATURE_MAX           (float)(130.0)
#define I2C_SGP4X_TEMPERATURE_MIN           (float)(-45.0)
#define I2C_SGP4X_HUMIDITY_MAX              (float)(100.0)
#define I2C_SGP4X_HUMIDITY_MIN              (float)(0.0)

#define I2C_SGP4X_CONDITIONING_TIMEOUT_MS   UINT16_C(1000) /*!< conditioning timeout 1-sec, 10-sec in total and maximum */
#define I2C_SGP4X_POWERUP_DELAY_MS          UINT16_C(10)
#define I2C_SGP4X_APPSTART_DELAY_MS         UINT16_C(10)    /*!< delay after initialization before application start-up */ 
#define I2C_SGP4X_CMD_DELAY_MS              UINT16_C(5)     /*!< delay before attempting I2C transactions after a command is issued */
#define I2C_SGP4X_RETRY_DELAY_MS            UINT16_C(2)


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "sgp4x";

/*
* functions and subrountines
*/

/**
 * @brief Calculates SGP4X crc8 value for 2-byte data packet.  See datasheet for details.
 *
 * @param[in] data[2] 2-byte data packet to perform crc8 check against.
 * @return uint8_t Calculated crc8 value.
 */
static inline uint8_t i2c_sgp4x_crc8(const uint8_t data[2]) {
    uint8_t crc = 0xff;
    for (size_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ I2C_SGP4X_CRC8_G_POLYNOM : crc << 1;
    }
    return crc;
}

/**
 * @brief Converts `uint16_t` variable from little endian order to
 * big endian order.
 * 
 * @param[in] leo Variable in little endian order.
 * @return uint16_t Converted variable in big endian order
 */
static inline uint16_t i2c_sgp4x_leo_to_beo(const uint16_t leo) {
    return (leo << 8) | (leo >> 8);
}

/**
 * @brief Gets the command execution duration in milli-seconds.  This delay is used after the I2C
 * write transaction to give the SGP4X time to process the command and return the results.
 * 
 * @param[in] command Command to determine duration against.
 * @return uint16_t Command execution duration in milli-seconds.
 */
static inline uint16_t i2c_sgp4x_get_command_duration_ms(const uint16_t command) {
    switch(command) {
        case I2C_SGP4X_CMD_EXEC_CONDITIONING:
            return 50;
        case I2C_SGP4X_CMD_MEAS_RAW_SIGNALS:
            return 50;
        case I2C_SGP4X_CMD_EXEC_SELF_TEST:
            return 320;
        case I2C_SGP4X_CMD_TURN_HEATER_OFF:
            return 1;
        case I2C_SGP4X_CMD_SERIAL_NUMBER:
            return 1;
        case I2C_SGP4X_CMD_RESET:
            return 5;
        default:
            return 50;
    }
}

/**
 * @brief Converts temperature to compensation ticks.
 * 
 * @param[in] temperature Temperature in degrees celcius.
 * @return i2c_bytes_to_uint16_t Temperature compensation ticks
 */
static inline i2c_bytes_to_uint16_t i2c_sgp4x_temperature_to_ticks(const float temperature) {
    const i2c_bytes_to_uint16_t ticks = {.value = (uint16_t)(temperature + 45) * 65535 / 175};
    return ticks;
}

/**
 * @brief Converts humidity to compensation ticks.
 * 
 * @param[in] humidity Humidity in precentage.
 * @return i2c_bytes_to_uint16_t Humidity compensation ticks.
 */
static inline i2c_bytes_to_uint16_t i2c_sgp4x_humidity_to_ticks(const float humidity) {
    const i2c_bytes_to_uint16_t ticks = {.value = (uint16_t)llround(humidity * 65535 / 100)};
    return ticks;
}

/**
 * @brief Writes 2-byte I2C command to SGP4X in big-endian order.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @param[in] command I2C command to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_sgp4x_write_command(i2c_sgp4x_handle_t sgp4x_handle, const uint16_t command) {
    const i2c_bytes_to_uint16_t cmd_bytes = { .value = command };
    const i2c_uint16_t          tx_buffer = { cmd_bytes.bytes[1], cmd_bytes.bytes[0] };

    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit(sgp4x_handle->i2c_dev_handle, tx_buffer, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write to i2c device handle, write command failed" );

    return ESP_OK;
}

/**
 * @brief Reads serial number register from SGP4X.
 * 
 * @param[in] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_sgp4x_get_serial_number_register(i2c_sgp4x_handle_t sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_write_command(sgp4x_handle, I2C_SGP4X_CMD_SERIAL_NUMBER), TAG, "unable to write to i2c device handle, read serial number failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_SERIAL_NUMBER)));

    /* attempt i2c read transaction */
    i2c_uint72_t rx_buffer = {};
    ESP_RETURN_ON_ERROR( i2c_master_receive(sgp4x_handle->i2c_dev_handle, rx_buffer, I2C_UINT72_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to read to i2c device handle, read serial number failed");

    /* set 2-byte serial number parts */
    const i2c_bytes_to_uint16_t sn_1 = {
        .bytes[0] = rx_buffer[0],
        .bytes[1] = rx_buffer[1]
    };
    const i2c_bytes_to_uint16_t sn_2 = {
        .bytes[0] = rx_buffer[3],
        .bytes[1] = rx_buffer[4]
    };
    const i2c_bytes_to_uint16_t sn_3 = {
        .bytes[0] = rx_buffer[6],
        .bytes[1] = rx_buffer[7]
    };

    /* validate crc for each serial number part */
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(sn_1.bytes) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 1, read serial number failed" );
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(sn_2.bytes) == rx_buffer[5]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 2, read serial number failed" );
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(sn_3.bytes) == rx_buffer[8]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 3, read serial number failed" );

    /* set handle serial number */
    sgp4x_handle->serial_number = (((uint64_t)sn_1.value) << 32) | (((uint64_t)sn_2.value) << 16) | ((uint64_t)sn_3.value);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_sgp4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sgp4x_config_t *sgp4x_config, i2c_sgp4x_handle_t *sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && sgp4x_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, sgp4x_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, sgp4x device handle initialization failed", sgp4x_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_sgp4x_handle_t out_handle = (i2c_sgp4x_handle_t)calloc(1, sizeof(i2c_sgp4x_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for device, sgp4x device handle initialization failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = sgp4x_config->dev_config.device_address,
        .scl_speed_hz       = sgp4x_config->dev_config.scl_speed_hz
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "unable to add device to master bus, sgp4x device handle initialization failed");
    }

    /* copy configuration */

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* attempt to reset the device */
    //ESP_GOTO_ON_ERROR(i2c_sgp4x_reset(out_handle), err_handle, TAG, "unable to soft-reset device, sgp4x device handle initialization failed");

    /* attempt to read device serial number */
    ESP_GOTO_ON_ERROR(i2c_sgp4x_get_serial_number_register(out_handle), err_handle, TAG, "unable to read device serial number, sgp4x device handle initialization failed");

    /* set device handle */
    *sgp4x_handle = out_handle;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_sgp4x_execute_compensated_conditioning(i2c_sgp4x_handle_t sgp4x_handle, const float temperature, const float humidity, uint16_t *sraw_voc) {
    const uint8_t               rx_retry_max   = 5;
    const i2c_bytes_to_uint16_t command        = { .value = I2C_SGP4X_CMD_EXEC_CONDITIONING };
    esp_err_t                   ret            = ESP_OK;
    uint8_t                     rx_retry_count = 0;
    i2c_bytes_to_uint16_t       crc8_buffer    = { .value = 0 };
    i2c_uint64_t                tx_buffer      = {};
    i2c_uint24_t                rx_buffer      = {};

    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    // validate range of temperature compensation parameter
    if(temperature > I2C_SGP4X_TEMPERATURE_MAX || temperature < I2C_SGP4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, execute compensated conditioning failed");
    }

    // validate range of humidity compensation parameter
    if(humidity > I2C_SGP4X_HUMIDITY_MAX || humidity < I2C_SGP4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, execute compensated conditioning failed");
    }

    /* convert compensation parameters to ticks */
    const i2c_bytes_to_uint16_t temperature_ticks = i2c_sgp4x_temperature_to_ticks(temperature);
    const i2c_bytes_to_uint16_t humidity_ticks    = i2c_sgp4x_humidity_to_ticks(humidity);

    /* calculate crc8 for compensation parameter ticks - big-endian order */
    crc8_buffer.bytes[0] = temperature_ticks.bytes[1];
    crc8_buffer.bytes[1] = temperature_ticks.bytes[0];
    const uint8_t temperature_ticks_crc8 = i2c_sgp4x_crc8(crc8_buffer.bytes);
    crc8_buffer.bytes[0] = humidity_ticks.bytes[1];
    crc8_buffer.bytes[1] = humidity_ticks.bytes[0];
    const uint8_t humidity_ticks_crc8 = i2c_sgp4x_crc8(crc8_buffer.bytes);

    /* construct tx packet - big-endian order */
    tx_buffer[0] = command.bytes[1];
    tx_buffer[1] = command.bytes[0];
    tx_buffer[2] = humidity_ticks.bytes[1];
    tx_buffer[3] = humidity_ticks.bytes[0];
    tx_buffer[4] = humidity_ticks_crc8;
    tx_buffer[5] = temperature_ticks.bytes[1];
    tx_buffer[6] = temperature_ticks.bytes[0];
    tx_buffer[7] = temperature_ticks_crc8;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(sgp4x_handle->i2c_dev_handle, tx_buffer, I2C_UINT64_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write to i2c device handle, execute compensated conditioning failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_EXEC_CONDITIONING)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(sgp4x_handle->i2c_dev_handle, rx_buffer, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, execute compensated conditioning failed" );

    /* validate crc from rx result - little-endian order */
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(rx_buffer) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8, execute compensated conditioning failed" );

    /* set output parameter */
    *sraw_voc = (uint16_t)rx_buffer[0] << 8 | (uint16_t)rx_buffer[1];

    return ESP_OK;
}

esp_err_t i2c_sgp4x_execute_conditioning(i2c_sgp4x_handle_t sgp4x_handle, uint16_t *sraw_voc) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt to execute compensated conditioning */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_execute_compensated_conditioning(sgp4x_handle, I2C_SGP4X_TEMPERATURE_COMP_DEFAULT, I2C_SGP4X_HUMIDITY_COMP_DEFAULT, sraw_voc), TAG, "unable to write to i2c device handle, execute conditioning failed");

    return ESP_OK;
}

esp_err_t i2c_sgp4x_measure_compensated_raw_signals(i2c_sgp4x_handle_t sgp4x_handle, const float temperature, const float humidity, uint16_t *sraw_voc, uint16_t *sraw_nox) {
    const uint8_t               rx_retry_max   = 5;
    const i2c_bytes_to_uint16_t command        = { .value = I2C_SGP4X_CMD_MEAS_RAW_SIGNALS };
    esp_err_t                   ret            = ESP_OK;
    uint8_t                     rx_retry_count = 0;
    i2c_bytes_to_uint16_t       crc8_buffer    = { .value = 0 };
    i2c_uint64_t                tx_buffer      = {};
    i2c_uint48_t                rx_buffer      = {};

    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    // validate range of temperature compensation parameter
    if(temperature > I2C_SGP4X_TEMPERATURE_MAX || temperature < I2C_SGP4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, measure compensated raw signals failed");
    }

    // validate range of humidity compensation parameter
    if(humidity > I2C_SGP4X_HUMIDITY_MAX || humidity < I2C_SGP4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, measure compensated raw signals failed");
    }

    /* convert compensation parameters to ticks */
    const i2c_bytes_to_uint16_t temperature_ticks = i2c_sgp4x_temperature_to_ticks(temperature);
    const i2c_bytes_to_uint16_t humidity_ticks    = i2c_sgp4x_humidity_to_ticks(humidity);

    /* calculate crc8 for compensation parameter ticks - big-endian order */
    crc8_buffer.bytes[0] = temperature_ticks.bytes[1];
    crc8_buffer.bytes[1] = temperature_ticks.bytes[0];
    const uint8_t temperature_ticks_crc8 = i2c_sgp4x_crc8(crc8_buffer.bytes);
    crc8_buffer.bytes[0] = humidity_ticks.bytes[1];
    crc8_buffer.bytes[1] = humidity_ticks.bytes[0];
    const uint8_t humidity_ticks_crc8 = i2c_sgp4x_crc8(crc8_buffer.bytes);

    /* construct tx packet - big-endian order */
    tx_buffer[0] = command.bytes[1];
    tx_buffer[1] = command.bytes[0];
    tx_buffer[2] = humidity_ticks.bytes[1];
    tx_buffer[3] = humidity_ticks.bytes[0];
    tx_buffer[4] = humidity_ticks_crc8;
    tx_buffer[5] = temperature_ticks.bytes[1];
    tx_buffer[6] = temperature_ticks.bytes[0];
    tx_buffer[7] = temperature_ticks_crc8;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(sgp4x_handle->i2c_dev_handle, tx_buffer, I2C_UINT64_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write to i2c device handle, measure compensated raw signals failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_MEAS_RAW_SIGNALS)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(sgp4x_handle->i2c_dev_handle, rx_buffer, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, measure compensated raw signals failed" );

    /* validate crc from rx result - little-endian order */
    crc8_buffer.bytes[0] = rx_buffer[0];
    crc8_buffer.bytes[1] = rx_buffer[1];
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(crc8_buffer.bytes) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 for sraw_voc, measure compensated raw signals failed" );
    crc8_buffer.bytes[0] = rx_buffer[3];
    crc8_buffer.bytes[1] = rx_buffer[4];
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(crc8_buffer.bytes) == rx_buffer[5]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 for sraw_nox, measure compensated raw signals failed" );

    /* set output parameters */
    *sraw_voc = (uint16_t)rx_buffer[0] << 8 | (uint16_t)rx_buffer[1];
    *sraw_nox = (uint16_t)rx_buffer[3] << 8 | (uint16_t)rx_buffer[4];

    return ESP_OK;
}

esp_err_t i2c_sgp4x_measure_raw_signals(i2c_sgp4x_handle_t sgp4x_handle, uint16_t *sraw_voc, uint16_t *sraw_nox) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt to execute compensated raw signals */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_measure_compensated_raw_signals(sgp4x_handle, 25.0f, 50.0f, sraw_voc, sraw_nox), TAG, "unable to write to i2c device handle, measure raw signals failed");

    return ESP_OK;
}

esp_err_t i2c_sgp4x_execute_self_test(i2c_sgp4x_handle_t sgp4x_handle, i2c_sgp4x_self_test_result_t *const result) {
    const uint8_t rx_retry_max   = 5;
    esp_err_t     ret            = ESP_OK;
    uint8_t       rx_retry_count = 0;
    i2c_uint24_t  rx_buffer      = {};

    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_write_command(sgp4x_handle, I2C_SGP4X_CMD_EXEC_SELF_TEST), TAG, "unable to write to i2c device handle, execute self-test failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_EXEC_SELF_TEST)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(sgp4x_handle->i2c_dev_handle, rx_buffer, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, execute self-test failed" );

    /* validate crc from result */
    ESP_RETURN_ON_FALSE( (i2c_sgp4x_crc8(rx_buffer) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8, execute self-test failed" );

    /* set results - lsb */
    result->integrity = rx_buffer[0];

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SGP4X_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_sgp4x_turn_heater_off(i2c_sgp4x_handle_t sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_write_command(sgp4x_handle, I2C_SGP4X_CMD_TURN_HEATER_OFF), TAG, "unable to write to i2c device handle, turn off heater failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_TURN_HEATER_OFF)));

    return ESP_OK;
}

esp_err_t i2c_sgp4x_reset(i2c_sgp4x_handle_t sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_write_command(sgp4x_handle, I2C_SGP4X_CMD_RESET), TAG, "unable to write to i2c device handle, soft-reset failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(i2c_sgp4x_get_command_duration_ms(I2C_SGP4X_CMD_RESET)));

    return ESP_OK;
}

esp_err_t i2c_sgp4x_remove(i2c_sgp4x_handle_t sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    return i2c_master_bus_rm_device(sgp4x_handle->i2c_dev_handle);
}

esp_err_t i2c_sgp4x_delete(i2c_sgp4x_handle_t sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sgp4x_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_sgp4x_remove(sgp4x_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(sgp4x_handle->i2c_dev_handle) {
        free(sgp4x_handle->i2c_dev_handle);
        free(sgp4x_handle);
    }

    return ESP_OK;
}