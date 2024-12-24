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
 * @file sht4x.c
 *
 * ESP-IDF driver for SHT4x air temperature and relative humidity sensor
 * 
 * https://github.com/Sensirion/embedded-sht/releases
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "sht4x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * SHT4X definitions
*/

#define I2C_SHT4X_CRC8_G_POLYNOM        UINT8_C(0x31)   //!< sht4x I2C CRC8 polynomial

#define I2C_SHT4X_CMD_RESET             UINT8_C(0x94)   //!< sht4x I2C soft-reset command 
#define I2C_SHT4X_CMD_SERIAL            UINT8_C(0x89)   //!< sht4x I2C serial number request command
#define I2C_SHT4X_CMD_MEAS_HIGH         UINT8_C(0xFD)   //!< sht4x I2C high resolution measurement command
#define I2C_SHT4X_CMD_MEAS_MED          UINT8_C(0xF6)   //!< sht4x I2C medium resolution measurement command
#define I2C_SHT4X_CMD_MEAS_LOW          UINT8_C(0xE0)   //!< sht4x I2C low resolution measurement command
#define I2C_SHT4X_CMD_MEAS_H_HIGH_LONG  UINT8_C(0x39)   //!< sht4x I2C high resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_HIGH_SHORT UINT8_C(0x32)   //!< sht4x I2C high resolution measurement command with heater enabled short pulse
#define I2C_SHT4X_CMD_MEAS_H_MED_LONG   UINT8_C(0x2F)   //!< sht4x I2C medium resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_MED_SHORT  UINT8_C(0x24)   //!< sht4x I2C medium resolution measurement command with heater enabled short pulse
#define I2C_SHT4X_CMD_MEAS_H_LOW_LONG   UINT8_C(0x1E)   //!< sht4x I2C low resolution measurement command with heater enabled long pulse
#define I2C_SHT4X_CMD_MEAS_H_LOW_SHORT  UINT8_C(0x15)   //!< sht4x I2C low resolution measurement command with heater enabled short pulse

#define I2C_SHT4X_TEMPERATURE_MAX       (float)(125.0)  //!< sht4x maximum temperature range
#define I2C_SHT4X_TEMPERATURE_MIN       (float)(-40.0)  //!< sht4x minimum temperature range
#define I2C_SHT4X_HUMIDITY_MAX          (float)(100.0)  //!< sht4x maximum humidity range
#define I2C_SHT4X_HUMIDITY_MIN          (float)(0.0)    //!< sht4x minimum humidity range

#define I2C_SHT4X_POWERUP_DELAY_MS      UINT16_C(5)     /*!< sht4x delay on power-up before attempting I2C transactions */
#define I2C_SHT4X_APPSTART_DELAY_MS     UINT16_C(10)    /*!< sht4x delay after initialization before application start-up */
#define I2C_SHT4X_RESET_DELAY_MS        UINT16_C(25)    /*!< sht4x delay before attempting I2C transactions after a reset is issued */
#define I2C_SHT4X_CMD_DELAY_MS          UINT16_C(5)     /*!< sht4x delay before attempting I2C transactions after a command is issued */
#define I2C_SHT4X_RETRY_DELAY_MS        UINT16_C(2)     /*!< sht4x delay between an I2C receive transaction retry */
#define I2C_SHT4X_TX_RX_DELAY_MS        UINT16_C(10)    /*!< sht4x delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "sht4x";

/*
* functions and subrountines
*/

/**
 * @brief Calculates SHT4X crc8 value.  See datasheet for details.
 *
 * @param[in] data[] Data buffer to perform crc8 check against.
 * @param[in] len Length of `data` buffer.
 * @return uint8_t Caculated crc8 value.
 */
static inline uint8_t i2c_sht4x_crc8(const uint8_t data[], const size_t len) {
    uint8_t crc = 0xff;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ I2C_SHT4X_CRC8_G_POLYNOM : crc << 1;
    }
    return crc;
}

/**
 * @brief Gets SHT4X measurement duration in milli-seconds from device handle.  See datasheet for details.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @return size_t Measurement duration in milliseconds.
 */
static inline size_t i2c_sht4x_get_duration_ms(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    if (!sht4x_handle) return 2;
    switch (sht4x_handle->heater_mode) {
        case I2C_SHT4X_HEATER_HIGH_LONG:
        case I2C_SHT4X_HEATER_MEDIUM_LONG:
        case I2C_SHT4X_HEATER_LOW_LONG:
            return 1100;
        case I2C_SHT4X_HEATER_HIGH_SHORT:
        case I2C_SHT4X_HEATER_MEDIUM_SHORT:
        case I2C_SHT4X_HEATER_LOW_SHORT:
            return 110;
        default:
            switch (sht4x_handle->repeat_mode) {
                case I2C_SHT4X_REPEAT_HIGH:
                    return 10;
                case I2C_SHT4X_REPEAT_MEDIUM:
                    return 5; 
                default:
                    return 2;
            }
    }
}

/**
 * @brief Gets SHT4X measurement tick duration from device handle.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @return size_t Measurement duration in ticks.
 */
static inline size_t i2c_sht4x_get_tick_duration(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    if (!sht4x_handle) return 1;
    size_t res = pdMS_TO_TICKS(i2c_sht4x_get_duration_ms(sht4x_handle));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets SHT4X measurement command from device handle parameters.  See datasheet for details.
 *
 * @param[in] sht4x_handle SHT4X device handle.
 * @return uint8_t SHT4X command value.
 */
static inline uint8_t i2c_sht4x_get_command(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    if (!sht4x_handle) return I2C_SHT4X_CMD_MEAS_LOW;
    switch (sht4x_handle->heater_mode) {
        case I2C_SHT4X_HEATER_HIGH_LONG:
            return I2C_SHT4X_CMD_MEAS_H_HIGH_LONG;
        case I2C_SHT4X_HEATER_HIGH_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_HIGH_SHORT;
        case I2C_SHT4X_HEATER_MEDIUM_LONG:
            return I2C_SHT4X_CMD_MEAS_H_MED_LONG;
        case I2C_SHT4X_HEATER_MEDIUM_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_MED_SHORT;
        case I2C_SHT4X_HEATER_LOW_LONG:
            return I2C_SHT4X_CMD_MEAS_H_LOW_LONG;
        case I2C_SHT4X_HEATER_LOW_SHORT:
            return I2C_SHT4X_CMD_MEAS_H_LOW_SHORT;
        default:
            switch (sht4x_handle->repeat_mode) {
                case I2C_SHT4X_REPEAT_HIGH:
                    return I2C_SHT4X_CMD_MEAS_HIGH;
                case I2C_SHT4X_REPEAT_MEDIUM:
                    return I2C_SHT4X_CMD_MEAS_MED;
                default:
                    return I2C_SHT4X_CMD_MEAS_LOW;
            }
    }
}

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humiity in percent.
 * @param[out] dewpoint calculated dewpoint temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_sht4x_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK(temperature && humidity && dewpoint);

    // validate range of temperature parameter
    if(temperature > I2C_SHT4X_TEMPERATURE_MAX || temperature < I2C_SHT4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dewpoint failed");
    }

    // validate range of humidity parameter
    if(humidity > I2C_SHT4X_HUMIDITY_MAX || humidity < I2C_SHT4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dewpoint failed");
    }
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

/**
 * @brief Read serial number register from SHT4X.
 *
 * @param[in] sht4x_config configuration of sht4x device
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_sht4x_get_serial_number_register(i2c_sht4x_handle_t sht4x_handle) {
    i2c_uint8_t i2c_tx_buffer 	= { I2C_SHT4X_CMD_SERIAL };
    i2c_uint48_t i2c_rx_buffer	= { 0, 0, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(sht4x_handle->i2c_dev_handle, i2c_tx_buffer, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write to i2c device handle, get serial number failed");
	
    /* delay before attempting i2c read transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(sht4x_handle->i2c_dev_handle, i2c_rx_buffer, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to read to i2c device handle, get serial number failed");
	
    /* set serial number */
    sht4x_handle->serial_number = ((uint32_t)i2c_rx_buffer[0] << 24) | ((uint32_t)i2c_rx_buffer[1] << 16) | ((uint32_t)i2c_rx_buffer[3] << 8) | i2c_rx_buffer[4];

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_sht4x_init(i2c_master_bus_handle_t bus_handle, const i2c_sht4x_config_t *sht4x_config, i2c_sht4x_handle_t *sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && sht4x_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, sht4x_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, sht4x device handle initialization failed", sht4x_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_sht4x_handle_t out_handle = (i2c_sht4x_handle_t)calloc(1, sizeof(i2c_sht4x_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for device, sht4x device handle initialization failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = sht4x_config->dev_config.device_address,
        .scl_speed_hz       = sht4x_config->dev_config.scl_speed_hz
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "unable to add device to master bus, sht4x device handle initialization failed");
    }

    /* copy configuration */
    out_handle->heater_mode = sht4x_config->heater_mode;
    out_handle->repeat_mode = sht4x_config->repeat_mode;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(i2c_sht4x_reset(out_handle), err_handle, TAG, "unable to soft-reset device, sht4x device handle initialization failed");

    /* set device handle */
    *sht4x_handle = out_handle;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_sht4x_get_measurement(i2c_sht4x_handle_t sht4x_handle, float *const temperature, float *const humidity) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    size_t delay_ticks 			= 0;
    esp_err_t ret               = ESP_OK;
    i2c_uint8_t i2c_tx_buffer 	= { 0 };
    i2c_uint48_t i2c_rx_buffer	= { 0, 0, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle && temperature && humidity );
    
    /* get command and measurement duration from handle settings */
    i2c_tx_buffer[0] = i2c_sht4x_get_command(sht4x_handle);
    delay_ticks      = i2c_sht4x_get_tick_duration(sht4x_handle);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(sht4x_handle->i2c_dev_handle, i2c_tx_buffer, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to write to i2c device handle, get measurement failed");
	
	/* delay task - allow time for the sensor to process measurement request */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = i2c_master_receive(sht4x_handle->i2c_dev_handle, i2c_rx_buffer, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, get measurement failed" );
	
    /* validate crc values */
    if (i2c_rx_buffer[2] != i2c_sht4x_crc8(i2c_rx_buffer, 2) || i2c_rx_buffer[5] != i2c_sht4x_crc8(i2c_rx_buffer + 3, 2)) return ESP_ERR_INVALID_CRC;

	// convert sht4x results to engineering units of measure (C and %)
    *temperature = ((uint16_t)i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1]) * 175.0 / 65535.0 - 45.0;
    *humidity    = ((uint16_t)i2c_rx_buffer[3] << 8 | i2c_rx_buffer[4]) * 125.0 / 65535.0 - 6.0;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_sht4x_get_measurements(i2c_sht4x_handle_t sht4x_handle, float *const temperature, float *const humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle && temperature && humidity && dewpoint );

    ESP_RETURN_ON_ERROR( i2c_sht4x_get_measurement(sht4x_handle, temperature, humidity), TAG, "unable to read measurement, read measurements failed" );

    ESP_RETURN_ON_ERROR( i2c_sht4x_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "unable to calculate dewpoint, read measurements failed" );

    return ESP_OK;
}

esp_err_t i2c_sht4x_get_repeat_mode(i2c_sht4x_handle_t sht4x_handle, i2c_sht4x_repeat_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* set output parameter */
    *mode = sht4x_handle->repeat_mode;

    return ESP_OK;
}
esp_err_t i2c_sht4x_set_repeat_mode(i2c_sht4x_handle_t sht4x_handle, const i2c_sht4x_repeat_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* set handle setting */
    sht4x_handle->repeat_mode = mode;

    return ESP_OK;
}

esp_err_t i2c_sht4x_get_heater_mode(i2c_sht4x_handle_t sht4x_handle, i2c_sht4x_heater_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* set output parameter */
    *mode = sht4x_handle->heater_mode;

    return ESP_OK;
}

esp_err_t i2c_sht4x_set_heater_mode(i2c_sht4x_handle_t sht4x_handle, const i2c_sht4x_heater_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* set handle setting */
    sht4x_handle->heater_mode = mode;

    return ESP_OK;
}

esp_err_t i2c_sht4x_reset(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_cmd(sht4x_handle->i2c_dev_handle, I2C_SHT4X_CMD_RESET), TAG, "unable to write to device handle, sht4x device reset failed");

    /* delay before next command - soft-reset */
    vTaskDelay(pdMS_TO_TICKS(I2C_SHT4X_RESET_DELAY_MS));

    /* sht4x attempt to read device serial number */
    ESP_RETURN_ON_ERROR(i2c_sht4x_get_serial_number_register(sht4x_handle), TAG, "unable to read serial number, sht4x device reset failed");

    return ESP_OK;
}

esp_err_t i2c_sht4x_remove(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    return i2c_master_bus_rm_device(sht4x_handle->i2c_dev_handle);
}

esp_err_t i2c_sht4x_delete(i2c_sht4x_handle_t sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( sht4x_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_sht4x_remove(sht4x_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(sht4x_handle->i2c_dev_handle) {
        free(sht4x_handle->i2c_dev_handle);
        free(sht4x_handle);
    }

    return ESP_OK;
}