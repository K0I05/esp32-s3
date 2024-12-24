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
 * ESP-IDF driver for BMP390 temperature and pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bmp390.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * possible BMP390 registers
 */
#define I2C_BMP390_REG_TEMP_XLSB            UINT8_C(0x07)
#define I2C_BMP390_REG_TEMP_LSB             UINT8_C(0x08)
#define I2C_BMP390_REG_TEMP_MSB             UINT8_C(0x09)
#define I2C_BMP390_REG_TEMP                 (I2C_BMP390_REG_TEMP_XLSB)
#define I2C_BMP390_REG_PRESS_XLSB           UINT8_C(0x04) 
#define I2C_BMP390_REG_PRESS_LSB            UINT8_C(0x05)
#define I2C_BMP390_REG_PRESS_MSB            UINT8_C(0x06)
#define I2C_BMP390_REG_PRESSURE             (I2C_BMP390_REG_PRESS_XLSB)
#define I2C_BMP390_REG_SNRTIME_XLSB         UINT8_C(0x0C)
#define I2C_BMP390_REG_SNRTIME_LSB          UINT8_C(0x0D)
#define I2C_BMP390_REG_SNRTIME_MSB          UINT8_C(0x0E)
#define I2C_BMP390_REG_SNRTIME              (I2C_BMP390_REG_SNRTIME_XLSB)
#define I2C_BMP390_REG_EVENT                UINT8_C(0x10)
#define I2C_BMP390_REG_CONFIG               UINT8_C(0x1F)
#define I2C_BMP390_REG_PWRCTRL              UINT8_C(0x1B)
#define I2C_BMP390_REG_OSR                  UINT8_C(0x1C)
#define I2C_BMP390_REG_ODR                  UINT8_C(0x1D) 
#define I2C_BMP390_REG_STATUS               UINT8_C(0x03)
#define I2C_BMP390_REG_INT_STATUS           UINT8_C(0x11) 
#define I2C_BMP390_REG_INT_CNTRL            UINT8_C(0x19)
#define I2C_BMP390_REG_CHIP_ID              UINT8_C(0x00)
#define I2C_BMP390_REG_ERR                  UINT8_C(0x02)
#define I2C_BMP390_REG_CMD                  UINT8_C(0x7E)
#define I2C_BMP390_SFTRESET_CMD             UINT8_C(0xB6)

#define I2C_BMPBMP390_CHIP_ID_DFLT          UINT8_C(0x60)  //!< BMP390 default

#define I2C_BMP390_DATA_POLL_TIMEOUT_MS     UINT16_C(1000) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define I2C_BMP390_DATA_READY_DELAY_MS      UINT16_C(1)
#define I2C_BMP390_POWERUP_DELAY_MS         UINT16_C(25)  // start-up time is 2-ms
#define I2C_BMP390_APPSTART_DELAY_MS        UINT16_C(25)
#define I2C_BMP390_RESET_DELAY_MS           UINT16_C(25)
#define I2C_BMP390_CMD_DELAY_MS             UINT16_C(5)
#define I2C_BMP390_TX_RX_DELAY_MS           UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "bmp390";


/**
 * @brief Temperature compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] bmp390_handle BMP390 device handle.
 * @param[in] adc_temperature Raw adc temperature.
 * @return Compensated temperature in degrees Celsius.
 */
static inline double i2c_bmp390_compensate_temperature(i2c_bmp390_handle_t bmp390_handle, const uint32_t adc_temperature) {
    double var1 = (double)(adc_temperature - bmp390_handle->dev_conv_cal_factors->PAR_T1);
    double var2 = (double)(var1 * bmp390_handle->dev_conv_cal_factors->PAR_T2);
    //
    bmp390_handle->dev_conv_cal_factors->t_lin = var2 + (var1 * var1) * bmp390_handle->dev_conv_cal_factors->PAR_T3;

    return bmp390_handle->dev_conv_cal_factors->t_lin;
}

/**
 * @brief Pressure compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] bmp390_handle BMP390 device handle.
 * @param[in] adc_pressure Raw adc pressure.
 * @return Compensated pressure in pascal.
 */
static inline double i2c_bmp390_compensate_pressure(i2c_bmp390_handle_t bmp390_handle, const uint32_t adc_pressure) {
    double dat1 = bmp390_handle->dev_conv_cal_factors->PAR_P6 * bmp390_handle->dev_conv_cal_factors->t_lin;
    double dat2 = bmp390_handle->dev_conv_cal_factors->PAR_P7 * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin;
    double dat3 = bmp390_handle->dev_conv_cal_factors->PAR_P8 * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin;
    double var1 = bmp390_handle->dev_conv_cal_factors->PAR_P5 + dat1 + dat2 + dat3;
    //
    dat1 = bmp390_handle->dev_conv_cal_factors->PAR_P2 * bmp390_handle->dev_conv_cal_factors->t_lin;
    dat2 = bmp390_handle->dev_conv_cal_factors->PAR_P3 * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin;
    dat3 = bmp390_handle->dev_conv_cal_factors->PAR_P4 * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin * bmp390_handle->dev_conv_cal_factors->t_lin;
    double var2 = (double)adc_pressure * (bmp390_handle->dev_conv_cal_factors->PAR_P1 + dat1 + dat2 + dat3);
    //
    dat1 = (double)adc_pressure * (double)adc_pressure;
    dat2 = bmp390_handle->dev_conv_cal_factors->PAR_P9 + bmp390_handle->dev_conv_cal_factors->PAR_P10 * bmp390_handle->dev_conv_cal_factors->t_lin;
    dat3 = dat1 * dat2;
    double dat4 = dat3 + (double)adc_pressure * (double)adc_pressure * (double)adc_pressure * bmp390_handle->dev_conv_cal_factors->PAR_P11;
    //
    return var1 + var2 + dat4;
}

/**
 * @brief Reads calibration factors onboard the BMP390 and applies floating point correction factors.  See datasheet for details.
 *
 * @param[in] bmp390_handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp390_get_cal_factors(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x31, &bmp390_handle->dev_cal_factors->dig_T1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x33, &bmp390_handle->dev_cal_factors->dig_T2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x35, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_T3) );
    /* bmp280 attempt to request P1-P10 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x36, (uint16_t *)&bmp390_handle->dev_cal_factors->dig_P1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x38, (uint16_t *)&bmp390_handle->dev_cal_factors->dig_P2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x3a, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P3) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x3b, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P4) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x3c, &bmp390_handle->dev_cal_factors->dig_P5) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x3e, &bmp390_handle->dev_cal_factors->dig_P6) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x40, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P7) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x41, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P8) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bmp390_handle->i2c_dev_handle, 0x42, (uint16_t *)&bmp390_handle->dev_cal_factors->dig_P9) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x44, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P10) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, 0x45, (uint8_t *)&bmp390_handle->dev_cal_factors->dig_P11) );

    /*
    ESP_LOGW(TAG, "Calibration data received:");
    ESP_LOGW(TAG, "dig_T1=%u", bmp390_handle->dev_cal_factors->dig_T1);
    ESP_LOGW(TAG, "dig_T2=%u", bmp390_handle->dev_cal_factors->dig_T2);
    ESP_LOGW(TAG, "dig_T3=%d", bmp390_handle->dev_cal_factors->dig_T3);
    ESP_LOGW(TAG, "dig_P1=%d", bmp390_handle->dev_cal_factors->dig_P1);
    ESP_LOGW(TAG, "dig_P2=%d", bmp390_handle->dev_cal_factors->dig_P2);
    ESP_LOGW(TAG, "dig_P3=%d", bmp390_handle->dev_cal_factors->dig_P3);
    ESP_LOGW(TAG, "dig_P4=%d", bmp390_handle->dev_cal_factors->dig_P4);
    ESP_LOGW(TAG, "dig_P5=%u", bmp390_handle->dev_cal_factors->dig_P5);
    ESP_LOGW(TAG, "dig_P6=%u", bmp390_handle->dev_cal_factors->dig_P6);
    ESP_LOGW(TAG, "dig_P7=%d", bmp390_handle->dev_cal_factors->dig_P7);
    ESP_LOGW(TAG, "dig_P8=%d", bmp390_handle->dev_cal_factors->dig_P8);
    ESP_LOGW(TAG, "dig_P9=%d", bmp390_handle->dev_cal_factors->dig_P9);
    ESP_LOGW(TAG, "dig_P10=%d", bmp390_handle->dev_cal_factors->dig_P10);
    ESP_LOGW(TAG, "dig_P11=%d", bmp390_handle->dev_cal_factors->dig_P11);
    */

    /* convert calibration factors to floating point numbers */
    bmp390_handle->dev_conv_cal_factors->PAR_T1 = (float)bmp390_handle->dev_cal_factors->dig_T1 / powf(2.0f, -8.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_T2 = (float)bmp390_handle->dev_cal_factors->dig_T2 / powf(2.0f, 30.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_T3 = (float)bmp390_handle->dev_cal_factors->dig_T3 / powf(2.0f, 48.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P1 = ((float)bmp390_handle->dev_cal_factors->dig_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P2 = ((float)bmp390_handle->dev_cal_factors->dig_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P3 = (float)bmp390_handle->dev_cal_factors->dig_P3 / powf(2.0f, 32.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P4 = (float)bmp390_handle->dev_cal_factors->dig_P4 / powf(2.0f, 37.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P5 = (float)bmp390_handle->dev_cal_factors->dig_P5 / powf(2.0f, -3.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P6 = (float)bmp390_handle->dev_cal_factors->dig_P6 / powf(2.0f, 6.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P7 = (float)bmp390_handle->dev_cal_factors->dig_P7 / powf(2.0f, 8.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P8 = (float)bmp390_handle->dev_cal_factors->dig_P8 / powf(2.0f, 15.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P9 = (float)bmp390_handle->dev_cal_factors->dig_P9 / powf(2.0f, 48.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P10 = (float)bmp390_handle->dev_cal_factors->dig_P10 / powf(2.0f, 48.0f);
    bmp390_handle->dev_conv_cal_factors->PAR_P11 = (float)bmp390_handle->dev_cal_factors->dig_P11 / powf(2.0f, 65.0f);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));
    
    return ESP_OK;
}

/**
 * @brief Reads calibration factor, control measurement, and configuration registers from BMP390 and initializes handle registers.
 * 
 * @param bmp390_handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bmp390_get_registers(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt read chip identifier register */
    ESP_RETURN_ON_ERROR(i2c_bmp390_get_chip_id_register(bmp390_handle), TAG, "read chip indentifier for get registers failed");

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_cal_factors(bmp390_handle), TAG, "read calibration factors for get registers failed" );

    /* attempt read oversampling register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_oversampling_register(bmp390_handle), TAG, "read oversampling register for get registers failed" );

    /* attempt read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_configuration_register(bmp390_handle), TAG, "read configuration register for get registers failed" );

    /* attempt read power control register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_power_control_register(bmp390_handle), TAG, "read power control register for get registers failed" );

    /* attempt read output data rate register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_output_data_rate_register(bmp390_handle), TAG, "read output data rate register for get registers failed" );

    /* attempt read interrupt register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_interrupt_control_register(bmp390_handle), TAG, "read interrupt control register for get registers failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_chip_id_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_CHIP_ID, &bmp390_handle->dev_type), TAG, "read chip identifier register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_status_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_STATUS, &bmp390_handle->status_reg.reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_interrupt_status_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_INT_STATUS, &bmp390_handle->interrupt_status_reg.reg), TAG, "read interrupt status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_interrupt_control_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_INT_CNTRL, &bmp390_handle->interrupt_ctrl_reg.reg), TAG, "read interrupt control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_interrupt_control_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_interrupt_control_register_t interrupt_control_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy register */
    i2c_bmp390_interrupt_control_register_t interrupt_control = { .reg = interrupt_control_reg.reg };

    /* set register reserved settings */
    interrupt_control.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_INT_CNTRL, interrupt_control.reg), TAG, "write power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_interrupt_control_register(bmp390_handle), TAG, "read power control register for set control measurement register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_power_control_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_PWRCTRL, &bmp390_handle->power_ctrl_reg.reg), TAG, "read power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_power_control_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_power_control_register_t power_control_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy register */
    i2c_bmp390_power_control_register_t power_control = { .reg = power_control_reg.reg };

    /* set register reserved settings */
    power_control.bits.reserved1 = 0;
    power_control.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_PWRCTRL, power_control.reg), TAG, "write power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_power_control_register(bmp390_handle), TAG, "read power control register for set control measurement register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_output_data_rate_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_ODR, &bmp390_handle->output_data_rate_reg.reg), TAG, "read output data rate register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_output_data_rate_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_output_data_rate_register_t output_data_rate_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy register */
    i2c_bmp390_output_data_rate_register_t output_data_rate = { .reg = output_data_rate_reg.reg };

    /* set register reserved settings */
    output_data_rate.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_ODR, output_data_rate.reg), TAG, "write output data rate register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_output_data_rate_register(bmp390_handle), TAG, "read output data rate register for set output data rate register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_oversampling_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_OSR, &bmp390_handle->oversampling_reg.reg), TAG, "read oversampling register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_oversampling_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_oversampling_register_t oversampling_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy register */
    i2c_bmp390_oversampling_register_t oversampling = { .reg = oversampling_reg.reg };

    /* set register reserved settings */
    oversampling.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_OSR, oversampling.reg), TAG, "write oversampling register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_oversampling_register(bmp390_handle), TAG, "read oversampling register for set oversampling register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_configuration_register(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_CONFIG, &bmp390_handle->config_reg.reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_configuration_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_configuration_register_t config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy register */
    i2c_bmp390_configuration_register_t config = { .reg = config_reg.reg };

    /* set register reserved settings */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_configuration_register(bmp390_handle), TAG, "read configuration register for set configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp390_config_t *bmp390_config, i2c_bmp390_handle_t *bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && bmp390_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, bmp390_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bmp280 device handle initialization failed", bmp390_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_bmp390_handle_t out_handle = (i2c_bmp390_handle_t)calloc(1, sizeof(i2c_bmp390_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device for init");

    /* validate memory availability for handle calibration factors */
    out_handle->dev_cal_factors = (i2c_bmp390_cal_factors_t*)calloc(1, sizeof(i2c_bmp390_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device calibration factors for init");

    /* validate memory availability for handle converted calibration factors */
    out_handle->dev_conv_cal_factors = (i2c_bmp390_conv_cal_factors_t*)calloc(1, sizeof(i2c_bmp390_conv_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_conv_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device converted calibration factors for init");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bmp390_config->dev_config.device_address,
        .scl_speed_hz       = bmp390_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(i2c_bmp390_get_chip_id_register(out_handle), err_handle, TAG, "read chip identifier for init failed");
    if(out_handle->dev_type != I2C_BMPBMP390_CHIP_ID_DFLT) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", out_handle->dev_type);
    }

    /* attempt to reset the device and initialize handle registers */
    ESP_GOTO_ON_ERROR(i2c_bmp390_reset(out_handle), err_handle, TAG, "soft-reset and initialize registers for init failed");

    /* copy configuration and registers from handle */
    i2c_bmp390_power_control_register_t     power_ctrl_reg        = { .reg = out_handle->power_ctrl_reg.reg };
    i2c_bmp390_configuration_register_t     config_reg            = { .reg = out_handle->config_reg.reg };
    i2c_bmp390_oversampling_register_t      oversampling_reg      = { .reg = out_handle->oversampling_reg.reg };
    i2c_bmp390_output_data_rate_register_t  output_data_rate_reg  = { .reg = out_handle->output_data_rate_reg.reg };
    i2c_bmp390_interrupt_control_register_t interrupt_control_reg = { .reg = out_handle->interrupt_status_reg.reg };

    /* initialize configuration registers from configuration */
    output_data_rate_reg.bits.output_data_rate     = bmp390_config->output_data_rate;
    config_reg.bits.iir_filter                     = bmp390_config->iir_filter;
    power_ctrl_reg.bits.pressure_enabled           = true;
    power_ctrl_reg.bits.temperature_enabled        = true;
    power_ctrl_reg.bits.power_mode                 = bmp390_config->power_mode;
    oversampling_reg.bits.temperature_oversampling = bmp390_config->temperature_oversampling;
    oversampling_reg.bits.pressure_oversampling    = bmp390_config->pressure_oversampling;
    interrupt_control_reg.bits.irq_data_ready_enabled = true;
    
    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_bmp390_set_configuration_register(out_handle, config_reg), err_handle, TAG, "write configuration register for init failed");

    /* attempt to write oversampling register */
    ESP_GOTO_ON_ERROR(i2c_bmp390_set_oversampling_register(out_handle, oversampling_reg), err_handle, TAG, "write oversampling register for init failed");

    /* attempt to write to power control register */
    ESP_GOTO_ON_ERROR(i2c_bmp390_set_power_control_register(out_handle, power_ctrl_reg), err_handle, TAG, "write power control register for init failed");

    /* attempt to write to output data rate register */
    ESP_GOTO_ON_ERROR(i2c_bmp390_set_output_data_rate_register(out_handle, output_data_rate_reg), err_handle, TAG, "write output data rate register for init failed");

    /* attempt to write to interrupt control register */
    ESP_GOTO_ON_ERROR(i2c_bmp390_set_interrupt_control_register(out_handle, interrupt_control_reg), err_handle, TAG, "write interrupt control register for init failed");

    /* copy configuration */
    *bmp390_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_bmp390_get_measurements(i2c_bmp390_handle_t bmp390_handle, float *const temperature, float *const pressure) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle && temperature && pressure );

    /* initialize local variables */
    esp_err_t    ret                  = ESP_OK;
    uint64_t     start_time           = esp_timer_get_time(); /* set start time for timeout monitoring */
    bool         pressure_is_ready    = false;
    bool         temperature_is_ready = false;
    i2c_uint48_t rx                   = {};

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_bmp390_get_data_status(bmp390_handle, &temperature_is_ready, &pressure_is_ready), err, TAG, "data ready ready for get measurements failed." );
        
        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, I2C_BMP390_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (pressure_is_ready == false && temperature_is_ready == false);

    // read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte48(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_PRESSURE, &rx), err, TAG, "read temperature and pressure data failed" );
    
    // concat pressure and temperature adc values
    uint32_t adc_press, adc_temp;
    uint32_t data_xlsb, data_lsb, data_msb;
    data_xlsb = (uint32_t)rx[0];
    data_lsb  = (uint32_t)rx[1] << 8;
    data_msb  = (uint32_t)rx[2] << 16;
    adc_press = data_msb | data_lsb | data_xlsb;
    data_xlsb = (uint32_t)rx[3];
    data_lsb  = (uint32_t)rx[4] << 8;
    data_msb  = (uint32_t)rx[5] << 16;
    adc_temp  = data_msb | data_lsb | data_xlsb;

    ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);
    ESP_LOGD(TAG, "ADC pressure: %" PRIi32, adc_press);

    /* apply compensation and convert pressure and temperature values to engineering units of measure */
    *temperature = i2c_bmp390_compensate_temperature(bmp390_handle, adc_temp);
    *pressure    = i2c_bmp390_compensate_pressure(bmp390_handle, adc_press);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_bmp390_get_status(i2c_bmp390_handle_t bmp390_handle, bool *const temperature_ready, bool *const pressure_ready, bool *const command_ready) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_status_register(bmp390_handle), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = bmp390_handle->status_reg.bits.temperature_data_ready;
    *pressure_ready    = bmp390_handle->status_reg.bits.pressure_data_ready;
    *command_ready     = bmp390_handle->status_reg.bits.command_ready;

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_data_status(i2c_bmp390_handle_t bmp390_handle, bool *const temperature_ready, bool *const pressure_ready) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_status_register(bmp390_handle), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = bmp390_handle->status_reg.bits.temperature_data_ready;
    *pressure_ready    = bmp390_handle->status_reg.bits.pressure_data_ready;

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_power_mode(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_power_modes_t *const power_mode) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read power control register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_power_control_register(bmp390_handle), TAG, "read power control register for get power mode failed" );

    /* set output parameter */
    *power_mode = bmp390_handle->power_ctrl_reg.bits.power_mode;

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_power_mode(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_power_modes_t power_mode) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy power control register from handle */
    i2c_bmp390_power_control_register_t power_ctrl_reg = { .reg = bmp390_handle->power_ctrl_reg.reg };

    /* set register setting */
    power_ctrl_reg.bits.power_mode = power_mode;

    /* attempt to write power control register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_set_power_control_register(bmp390_handle, power_ctrl_reg), TAG, "write power control register for set power mode failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_pressure_oversampling(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_pressure_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_oversampling_register(bmp390_handle), TAG, "read oversampling register for get pressure oversampling failed" );

    /* set output parameter */
    *oversampling = bmp390_handle->oversampling_reg.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_pressure_oversampling(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_pressure_oversampling_t oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy oversampling register from handle */
    i2c_bmp390_oversampling_register_t oversampling_reg = { .reg = bmp390_handle->oversampling_reg.reg };

    /* set register setting */
    oversampling_reg.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_set_oversampling_register(bmp390_handle, oversampling_reg), TAG, "write oversampling register for set pressure oversampling failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_temperature_oversampling(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_temperature_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_oversampling_register(bmp390_handle), TAG, "read oversampling register for get temperature oversampling failed" );

    /* set output parameter */
    *oversampling = bmp390_handle->oversampling_reg.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_temperature_oversampling(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_temperature_oversampling_t oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy oversampling register from handle */
    i2c_bmp390_oversampling_register_t oversampling_reg = { .reg = bmp390_handle->oversampling_reg.reg };

    /* set register setting */
    oversampling_reg.bits.temperature_oversampling = oversampling;

    /* attempt to write oversampling register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_set_oversampling_register(bmp390_handle, oversampling_reg), TAG, "write oversampling register for set temperature oversampling failed" );

    return ESP_OK;
}


esp_err_t i2c_bmp280_get_output_data_rate(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_output_data_rates_t *const output_data_rate) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_output_data_rate_register(bmp390_handle), TAG, "read output data rate register for get standby time failed" );

    /* set output parameter */
    *output_data_rate = bmp390_handle->output_data_rate_reg.bits.output_data_rate;

    return ESP_OK;
}

esp_err_t i2c_bmp280_set_output_data_rate(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_output_data_rates_t output_data_rate) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy configuration register from handle */
    i2c_bmp390_output_data_rate_register_t output_data_rate_reg = { .reg = bmp390_handle->output_data_rate_reg.reg };

    /* set register setting */
    output_data_rate_reg.bits.output_data_rate  = output_data_rate;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_set_output_data_rate_register(bmp390_handle, output_data_rate_reg), TAG, "write output data rate register for set stanby time failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_get_iir_filter(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_iir_filters_t *const iir_filter) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_get_configuration_register(bmp390_handle), TAG, "read configuration register for get IIR filter failed" );

    /* set output parameter */
    *iir_filter = bmp390_handle->config_reg.bits.iir_filter;

    return ESP_OK;
}

esp_err_t i2c_bmp390_set_iir_filter(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_iir_filters_t iir_filter) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* copy configuration register from handle */
    i2c_bmp390_configuration_register_t config_reg = { .reg = bmp390_handle->config_reg.reg };

    /* set register setting */
    config_reg.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp390_set_configuration_register(bmp390_handle, config_reg), TAG, "write configuration register for set IIR filter failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_reset(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bmp390_handle->i2c_dev_handle, I2C_BMP390_REG_CMD, I2C_BMP390_SFTRESET_CMD), TAG, "write reset register for reset failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP390_RESET_DELAY_MS)); // check is busy in timeout loop...

    ESP_RETURN_ON_ERROR( i2c_bmp390_get_registers(bmp390_handle), TAG, "read registers for reset failed" );

    return ESP_OK;
}

esp_err_t i2c_bmp390_remove(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    return i2c_master_bus_rm_device(bmp390_handle->i2c_dev_handle);
}

esp_err_t i2c_bmp390_delete(i2c_bmp390_handle_t bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bmp390_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_bmp390_remove(bmp390_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(bmp390_handle->i2c_dev_handle) {
        free(bmp390_handle->i2c_dev_handle);
        free(bmp390_handle);
    }

    return ESP_OK;
}
