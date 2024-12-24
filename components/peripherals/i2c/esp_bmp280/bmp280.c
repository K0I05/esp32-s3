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
#include <esp_timer.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * possible BMP280 registers
 */
#define I2C_BMP280_REG_TEMP_XLSB            UINT8_C(0xFC)
#define I2C_BMP280_REG_TEMP_LSB             UINT8_C(0xFB)
#define I2C_BMP280_REG_TEMP_MSB             UINT8_C(0xFA)
#define I2C_BMP280_REG_TEMP                 (I2C_BMP280_REG_TEMP_MSB)
#define I2C_BMP280_REG_PRESS_XLSB           UINT8_C(0xF9) 
#define I2C_BMP280_REG_PRESS_LSB            UINT8_C(0xF8)
#define I2C_BMP280_REG_PRESS_MSB            UINT8_C(0xF7)
#define I2C_BMP280_REG_PRESSURE             (I2C_BMP280_REG_PRESS_MSB)
#define I2C_BMP280_REG_CONFIG               UINT8_C(0xF5) 
#define I2C_BMP280_REG_CTRL                 UINT8_C(0xF4)
#define I2C_BMP280_REG_STATUS               UINT8_C(0xF3)
#define I2C_BMP280_REG_CTRL_HUM             UINT8_C(0xF2)
#define I2C_BMP280_REG_RESET                UINT8_C(0xE0)
#define I2C_BMP280_REG_ID                   UINT8_C(0xD0)
#define I2C_BMP280_REG_CALIB                UINT8_C(0x88)
#define I2C_BMP280_REG_HUM_CALIB            UINT8_C(0x88)
#define I2C_BMP280_RESET_VALUE              UINT8_C(0xB6)

#define I2C_BMP280_TYPE_BMP280              UINT8_C(0x58)  //!< BMP280
#define I2C_BMP280_TYPE_BME280              UINT8_C(0x60)  //!< BME280

#define I2C_BMP280_DATA_POLL_TIMEOUT_MS     UINT16_C(250) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define I2C_BMP280_DATA_READY_DELAY_MS      UINT16_C(1)
#define I2C_BMP280_POWERUP_DELAY_MS         UINT16_C(25)  // start-up time is 2-ms
#define I2C_BMP280_APPSTART_DELAY_MS        UINT16_C(25)
#define I2C_BMP280_RESET_DELAY_MS           UINT16_C(25)
#define I2C_BMP280_CMD_DELAY_MS             UINT16_C(5)
#define I2C_BMP280_TX_RX_DELAY_MS           UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "bmp280";


/**
 * @brief temperature compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[in] adc_temperature raw adc temperature.
 * @param[out] fine_temperature fine temperature.
 * @return fine temperature in degrees Celsius.
 */
static inline int32_t i2c_bmp280_compensate_temperature(i2c_bmp280_handle_t bmp280_handle, const int32_t adc_temperature, int32_t *const fine_temperature) {
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
static inline uint32_t i2c_bmp280_compensate_pressure(i2c_bmp280_handle_t bmp280_handle, const int32_t adc_pressure, const int32_t fine_temperature) {
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
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp280_get_cal_factors(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
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

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));
    
    return ESP_OK;
}

/**
 * @brief reads fixed measurements (temperature and pressure) from the bmp280.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[out] temperature fixed temperature.
 * @param[out] pressure fixed temperature.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp280_get_fixed_measurements(i2c_bmp280_handle_t bmp280_handle, int32_t *const temperature, uint32_t *const pressure) {
    esp_err_t       ret             = ESP_OK;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    int32_t         adc_press;
    int32_t         adc_temp;
    int32_t         fine_temp;
    i2c_uint48_t    data;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle && temperature && pressure );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_bmp280_get_data_status(bmp280_handle, &data_is_ready), err, TAG, "data ready ready for get fixed measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, I2C_BMP280_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte48(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_PRESSURE, &data), err, TAG, "read temperature and pressure data failed" );

    adc_press = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp  = data[3] << 12 | data[4] << 4 | data[5] >> 4;

    ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);
    ESP_LOGD(TAG, "ADC pressure: %" PRIi32, adc_press);

    *temperature = i2c_bmp280_compensate_temperature(bmp280_handle, adc_temp, &fine_temp);
    *pressure    = i2c_bmp280_compensate_pressure(bmp280_handle, adc_press, fine_temp);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief reads fixed temperature measurement from the bmp280.  see datasheet for details.
 *
 * @param[in] bmp280_handle bmp280 device handle.
 * @param[out] temperature fixed temperature.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp280_get_fixed_temperature(i2c_bmp280_handle_t bmp280_handle, int32_t *const temperature) {
    esp_err_t       ret             = ESP_OK;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    int32_t         adc_temp;
    int32_t         fine_temp;
    i2c_uint24_t    data;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle && temperature );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_bmp280_get_data_status(bmp280_handle, &data_is_ready), err, TAG, "data ready ready for get fixed measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, I2C_BMP280_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte24(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_TEMP, &data), err, TAG, "read temperature data failed" );

    adc_temp  = data[0] << 12 | data[1] << 4 | data[2] >> 4;

    ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);

    *temperature = i2c_bmp280_compensate_temperature(bmp280_handle, adc_temp, &fine_temp);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief reads calibration factor, control measurement, and configuration registers from bmp280.
 * 
 * @param bmp280_handle bmp280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp280_get_registers(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_cal_factors(bmp280_handle), TAG, "read calibration factors for get registers failed" );

    /* attempt read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bmp280_handle), TAG, "read control measurement register for get registers failed" );

    /* attempt read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_configuration_register(bmp280_handle), TAG, "read configuration register for get registers failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_chip_id_register(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_ID, &bmp280_handle->dev_type), TAG, "read chip identifier register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_status_register(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_STATUS, &bmp280_handle->status_reg.reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_control_measurement_register(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, &bmp280_handle->ctrl_meas_reg.reg), TAG, "read control measurement register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_control_measurement_register(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_control_measurement_register_t ctrl_meas_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, ctrl_meas_reg.reg), TAG, "write control measurement register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bmp280_handle), TAG, "read control measurement register for set control measurement register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_configuration_register(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, &bmp280_handle->config_reg.reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_configuration_register(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_configuration_register_t config_reg) {
    i2c_bmp280_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy register */
    config.reg = config_reg.reg;

    /* set reserved to 0 */
    config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_configuration_register(bmp280_handle), TAG, "read configuration register for set configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp280_config_t *bmp280_config, i2c_bmp280_handle_t *bmp280_handle) {
    i2c_bmp280_configuration_register_t         config_reg;
    i2c_bmp280_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && bmp280_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, bmp280_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bmp280 device handle initialization failed", bmp280_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_bmp280_handle_t out_handle = (i2c_bmp280_handle_t)calloc(1, sizeof(i2c_bmp280_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device for init");

    /* validate memory availability for handle calibration factors */
    out_handle->dev_cal_factors = (i2c_bmp280_cal_factors_t*)calloc(1, sizeof(i2c_bmp280_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device calibration factors for init");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bmp280_config->dev_config.device_address,
        .scl_speed_hz       = bmp280_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(i2c_bmp280_get_chip_id_register(out_handle), err_handle, TAG, "read chip identifier for init failed");
    if(out_handle->dev_type != I2C_BMP280_TYPE_BMP280) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", out_handle->dev_type);
    }

    /* attempt to reset the device and initialize registers */
    ESP_GOTO_ON_ERROR(i2c_bmp280_reset(out_handle), err_handle, TAG, "soft-reset and initialize registers for init failed");

    /* copy configuration and control measurement registers from handle */
    config_reg.reg      = out_handle->config_reg.reg;
    ctrl_meas_reg.reg   = out_handle->ctrl_meas_reg.reg;

    /* initialize configuration register from configuration params */
    config_reg.bits.standby_time = bmp280_config->standby_time;
    config_reg.bits.iir_filter   = bmp280_config->iir_filter;

    /* initialize control measurement register from configuration params */
    if (bmp280_config->power_mode == I2C_BMP280_POWER_MODE_FORCED || bmp280_config->power_mode == I2C_BMP280_POWER_MODE_FORCED1) {
        // initial mode for forced is sleep
        ctrl_meas_reg.bits.power_mode               = I2C_BMP280_POWER_MODE_SLEEP;
        ctrl_meas_reg.bits.temperature_oversampling = bmp280_config->temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = bmp280_config->pressure_oversampling;
    } else {
        ctrl_meas_reg.bits.power_mode               = bmp280_config->power_mode;
        ctrl_meas_reg.bits.temperature_oversampling = bmp280_config->temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = bmp280_config->pressure_oversampling;
    }
    
    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_bmp280_set_configuration_register(out_handle, config_reg), err_handle, TAG, "write configuration register for init failed");

    /* attempt to write control measurement register */
    ESP_GOTO_ON_ERROR(i2c_bmp280_set_control_measurement_register(out_handle, ctrl_meas_reg), err_handle, TAG, "write control measurement register for init failed");

    /* copy configuration */
    *bmp280_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_bmp280_get_measurements(i2c_bmp280_handle_t bmp280_handle, float *const temperature, float *const pressure) {
    int32_t  fixed_temperature;
    uint32_t fixed_pressure;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle && temperature && pressure );

    /* attempt to read fixed measurements (temperature & pressure) */
    ESP_ERROR_CHECK( i2c_bmp280_get_fixed_measurements(bmp280_handle, &fixed_temperature, &fixed_pressure) );

    /* set output parameters */
    *temperature = (float)fixed_temperature / 100;
    *pressure    = (float)fixed_pressure / 256;

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_temperature(i2c_bmp280_handle_t bmp280_handle, float *const temperature) {
    int32_t  fixed_temperature;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle && temperature );

    /* attempt to read fixed temperature measurement */
    ESP_ERROR_CHECK( i2c_bmp280_get_fixed_temperature(bmp280_handle, &fixed_temperature) );

    /* set output parameter */
    *temperature = (float)fixed_temperature / 100;

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_pressure(i2c_bmp280_handle_t bmp280_handle, float *const pressure) {
    int32_t  fixed_temperature;
    uint32_t fixed_pressure;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle && pressure );

    /* attempt to read fixed measurements (temperature & pressure) */
    ESP_ERROR_CHECK( i2c_bmp280_get_fixed_measurements(bmp280_handle, &fixed_temperature, &fixed_pressure) );

    /* set output parameter */
    *pressure = (float)fixed_pressure / 256;

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_data_status(i2c_bmp280_handle_t bmp280_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_status_register(bmp280_handle), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    if(bmp280_handle->status_reg.bits.measuring == true) {
        *ready = false;
    } else {
        *ready = true;
    }

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_power_mode(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_power_modes_t *const power_mode) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bmp280_handle), TAG, "read control measurement register for get power mode failed" );

    /* set power mode */
    *power_mode = bmp280_handle->ctrl_meas_reg.bits.power_mode;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_power_mode(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_power_modes_t power_mode) {
    i2c_bmp280_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bmp280_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.power_mode = power_mode;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_set_control_measurement_register(bmp280_handle, ctrl_meas_reg), TAG, "write control measurement register for set power mode failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_pressure_oversampling(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_pressure_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bmp280_handle), TAG, "read control measurement register for get pressure oversampling failed" );

    /* set oversampling */
    *oversampling = bmp280_handle->ctrl_meas_reg.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_pressure_oversampling(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_pressure_oversampling_t oversampling) {
    i2c_bmp280_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bmp280_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_set_control_measurement_register(bmp280_handle, ctrl_meas_reg), TAG, "write control measurement register for set pressure oversampling failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_temperature_oversampling(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_temperature_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bmp280_handle), TAG, "read control measurement register for get temperature oversampling failed" );

    /* set oversampling */
    *oversampling = bmp280_handle->ctrl_meas_reg.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_temperature_oversampling(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_temperature_oversampling_t oversampling) {
    i2c_bmp280_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bmp280_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.temperature_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_set_control_measurement_register(bmp280_handle, ctrl_meas_reg), TAG, "write control measurement register for set temperature oversampling failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_standby_time(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_standby_times_t *const standby_time) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_configuration_register(bmp280_handle), TAG, "read configuration register for get standby time failed" );

    /* set standby time */
    *standby_time = bmp280_handle->config_reg.bits.standby_time;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_standby_time(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_standby_times_t standby_time) {
    i2c_bmp280_configuration_register_t   config_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy configuration register from handle */
    config_reg.reg = bmp280_handle->config_reg.reg;

    /* initialize configuration register */
    config_reg.bits.standby_time = standby_time;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_set_configuration_register(bmp280_handle, config_reg), TAG, "write configuration register for set stanby time failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_get_iir_filter(i2c_bmp280_handle_t bmp280_handle, i2c_bmp280_iir_filters_t *const iir_filter) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_configuration_register(bmp280_handle), TAG, "read configuration register for get IIR filter failed" );

    /* set standby time */
    *iir_filter = bmp280_handle->config_reg.bits.iir_filter;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_iir_filter(i2c_bmp280_handle_t bmp280_handle, const i2c_bmp280_iir_filters_t iir_filter) {
    i2c_bmp280_configuration_register_t   config_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* copy configuration register from handle */
    config_reg.reg = bmp280_handle->config_reg.reg;

    /* initialize configuration register */
    config_reg.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_set_configuration_register(bmp280_handle, config_reg), TAG, "write configuration register for set IIR filter failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_reset(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp280_handle->i2c_dev_handle, I2C_BMP280_REG_RESET, I2C_BMP280_RESET_VALUE), TAG, "write reset register for reset failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_RESET_DELAY_MS)); // check is busy in timeout loop...

    ESP_RETURN_ON_ERROR( i2c_bmp280_get_registers(bmp280_handle), TAG, "read registers for reset failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp280_remove(i2c_bmp280_handle_t bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    return i2c_master_bus_rm_device(bmp280_handle->i2c_dev_handle);
}

esp_err_t i2c_bmp280_delete(i2c_bmp280_handle_t bmp280_handle){
    /* validate arguments */
    ESP_ARG_CHECK( bmp280_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_bmp280_remove(bmp280_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(bmp280_handle->i2c_dev_handle) {
        free(bmp280_handle->i2c_dev_handle);
        free(bmp280_handle);
    }

    return ESP_OK;
}
