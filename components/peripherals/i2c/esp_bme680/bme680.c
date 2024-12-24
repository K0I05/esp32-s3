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
 * @file bme680.c
 *
 * ESP-IDF driver for BME680 temperature, humidity, pressure, and gas sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bme680.h"
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
#define I2C_BMP280_REG_TEMP_XLSB    0xFC /* bits: 7-4 */
#define I2C_BMP280_REG_TEMP_LSB     0xFB
#define I2C_BMP280_REG_TEMP_MSB     0xFA
#define I2C_BMP280_REG_TEMP         (I2C_BMP280_REG_TEMP_MSB)
#define I2C_BMP280_REG_PRESS_XLSB   0xF9 /* bits: 7-4 */
#define I2C_BMP280_REG_PRESS_LSB    0xF8
#define I2C_BMP280_REG_PRESS_MSB    0xF7
#define I2C_BMP280_REG_PRESSURE     (I2C_BMP280_REG_PRESS_MSB)
#define I2C_BMP280_REG_CONFIG       0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define I2C_BMP280_REG_CTRL         0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define I2C_BMP280_REG_STATUS       0xF3 /* bits: 3 measuring; 0 im_update */
#define I2C_BMP280_REG_CTRL_HUM     0xF2 /* bits: 2-0 osrs_h; */
#define I2C_BMP280_REG_RESET        0xE0
#define I2C_BMP280_REG_ID           0xD0
#define I2C_BMP280_REG_CALIB        0x88
#define I2C_BMP280_REG_HUM_CALIB    0x88
#define I2C_BMP280_RESET_VALUE      0xB6

#define I2C_BME680_CHIP_ID          0x61

#define I2C_BMP280_DATA_POLL_TIMEOUT_MS  UINT16_C(250) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define I2C_BMP280_DATA_READY_DELAY_MS   UINT16_C(1)
#define I2C_BMP280_POWERUP_DELAY_MS      UINT16_C(25)  // start-up time is 2-ms
#define I2C_BMP280_APPSTART_DELAY_MS     UINT16_C(25)
#define I2C_BMP280_RESET_DELAY_MS        UINT16_C(25)
#define I2C_BMP280_CMD_DELAY_MS          UINT16_C(5)
#define I2C_BMP280_TX_RX_DELAY_MS        UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "bme680";


/**
 * @brief temperature compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] bme680_handle bmp280 device handle.
 * @param[in] adc_temperature raw adc temperature.
 * @return temperature in degrees Celsius.
 */
static inline float i2c_bme680_compensate_temperature(i2c_bme680_handle_t bme680_handle, const uint32_t adc_temperature) {
    float var1;
    float var2;
    float calc_temp;

    /* calculate var1 data */
    var1 = ((((float)adc_temperature / 16384.0f) - ((float)bme680_handle->dev_cal_factors->par_T1 / 1024.0f)) * ((float)bme680_handle->dev_cal_factors->par_T2));

    /* calculate var2 data */
    var2 =
        (((((float)adc_temperature / 131072.0f) - ((float)bme680_handle->dev_cal_factors->par_T1 / 8192.0f)) *
          (((float)adc_temperature / 131072.0f) - ((float)bme680_handle->dev_cal_factors->par_T1 / 8192.0f))) * ((float)bme680_handle->dev_cal_factors->par_T3 * 16.0f));

    /* t_fine value*/
    bme680_handle->dev_cal_factors->temperature_fine = (var1 + var2);

    /* compensated temperature data*/
    calc_temp = ((bme680_handle->dev_cal_factors->temperature_fine) / 5120.0f);

    return calc_temp;
}

static inline float i2c_bme680_compensate_humidity(i2c_bme680_handle_t bme680_handle, const uint16_t adc_humidity) {
    float calc_hum;
    float var1;
    float var2;
    float var3;
    float var4;
    float temp_comp;

    /* compensated temperature data*/
    temp_comp = ((bme680_handle->dev_cal_factors->temperature_fine) / 5120.0f);
    var1 = (float)((float)adc_humidity) -
           (((float)bme680_handle->dev_cal_factors->par_H1 * 16.0f) + (((float)bme680_handle->dev_cal_factors->par_H3 / 2.0f) * temp_comp));
    var2 = var1 *
           ((float)(((float)bme680_handle->dev_cal_factors->par_H2 / 262144.0f) *
                    (1.0f + (((float)bme680_handle->dev_cal_factors->par_H4 / 16384.0f) * temp_comp) +
                     (((float)bme680_handle->dev_cal_factors->par_H5 / 1048576.0f) * temp_comp * temp_comp))));
    var3 = (float)bme680_handle->dev_cal_factors->par_H6 / 16384.0f;
    var4 = (float)bme680_handle->dev_cal_factors->par_H7 / 2097152.0f;
    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if (calc_hum > 100.0f)
    {
        calc_hum = 100.0f;
    }
    else if (calc_hum < 0.0f)
    {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

/**
 * @brief pressure compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] bme680_handle bmp280 device handle.
 * @param[in] adc_pressure raw adc pressure.
 * @return Pa, 24 integer bits and 8 fractional bits.
 */
static inline float i2c_bme680_compensate_pressure(i2c_bme680_handle_t bme680_handle, const uint32_t adc_pressure) {
    float var1;
    float var2;
    float var3;
    float calc_pres;

    var1 = (((float)bme680_handle->dev_cal_factors->temperature_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)bme680_handle->dev_cal_factors->par_P6) / (131072.0f));
    var2 = var2 + (var1 * ((float)bme680_handle->dev_cal_factors->par_P5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)bme680_handle->dev_cal_factors->par_P4) * 65536.0f);
    var1 = (((((float)bme680_handle->dev_cal_factors->par_P3 * var1 * var1) / 16384.0f) + ((float)bme680_handle->dev_cal_factors->par_P2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)bme680_handle->dev_cal_factors->par_P1));
    calc_pres = (1048576.0f - ((float)adc_pressure));

    /* Avoid exception caused by division by zero */
    if ((int)var1 != 0)
    {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)bme680_handle->dev_cal_factors->par_P9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)bme680_handle->dev_cal_factors->par_P8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (bme680_handle->dev_cal_factors->par_P10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)bme680_handle->dev_cal_factors->par_P7 * 128.0f)) / 16.0f);
    }
    else
    {
        calc_pres = 0;
    }

    return calc_pres;
}

static inline float i2c_bme680_compensate_gas_resistance_low(i2c_bme680_handle_t bme680_handle, uint16_t adc_gas_res, uint8_t gas_range) {
    float calc_gas_res;
    float var1;
    float var2;
    float var3;
    float gas_res_f = adc_gas_res;
    float gas_range_f = (1U << gas_range); /*lint !e790 / Suspicious truncation, integral to float */
    const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    var1 = (1340.0f + (5.0f * bme680_handle->dev_cal_factors->range_switching_error));
    var2 = (var1) * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);
    calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * gas_range_f * (((gas_res_f - 512.0f) / var2) + 1.0f));

    return calc_gas_res;
}

static inline float i2c_bme680_compensate_gas_resistance_high(uint16_t adc_gas_res, uint8_t gas_range) {
    float calc_gas_res;
    uint32_t var1 = UINT32_C(262144) >> gas_range;
    int32_t var2 = (int32_t)adc_gas_res - INT32_C(512);

    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;

    calc_gas_res = 1000000.0f * (float)var1 / (float)var2;

    return calc_gas_res;
}

static inline uint8_t i2c_bme680_compensate_heater_resistance(i2c_bme680_handle_t bme680_handle, uint16_t temperature) {
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    uint8_t res_heat;

    if (temperature > 400) /* Cap temperature */
    {
        temperature = 400;
    }

    var1 = (((float)bme680_handle->dev_cal_factors->par_G1 / (16.0f)) + 49.0f);
    var2 = ((((float)bme680_handle->dev_cal_factors->par_G2 / (32768.0f)) * (0.0005f)) + 0.00235f);
    var3 = ((float)bme680_handle->dev_cal_factors->par_G3 / (1024.0f));
    var4 = (var1 * (1.0f + (var2 * (float)temperature)));
    var5 = (var4 + (var3 * (float)bme680_handle->ambient_temperature)); //?? where is this variable set
    res_heat =
        (uint8_t)(3.4f *
                  ((var5 * (4 / (4 + (float)bme680_handle->dev_cal_factors->res_heat_range)) *
                    (1 / (1 + ((float)bme680_handle->dev_cal_factors->res_heat_val * 0.002f)))) -
                   25));

    return res_heat;
}

static inline uint8_t i2c_bme680_compute_gas_wait(uint16_t duration) {
    uint8_t factor = 0;
    uint8_t durval;

    if (duration >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    }
    else {
        while (duration > 0x3F) {
            duration = duration / 4;
            factor += 1;
        }

        durval = (uint8_t)(duration + (factor * 64));
    }

    return durval;
}

static inline uint8_t i2c_bme680_compute_heater_shared_duration(uint16_t duration) {
    uint8_t factor = 0;
    uint8_t heatdurval;

    if (duration >= 0x783) {
        heatdurval = 0xff; /* Max duration */
    } else {
        /* Step size of 0.477ms */
        duration = (uint16_t)(((uint32_t)duration * 1000) / 477);
        while (duration > 0x3F) {
            duration = duration >> 2;
            factor += 1;
        }

        heatdurval = (uint8_t)(duration + (factor * 64));
    }

    return heatdurval;
}

static inline uint32_t i2c_bme680_get_measurement_duration(i2c_bme680_handle_t bme680_handle, const i2c_bme680_config_t *bme680_config, const i2c_bme680_power_modes_t power_mode) {
    int8_t rslt;
    uint32_t meas_dur = 0; /* Calculate in us */
    uint32_t meas_cycles;
    const uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };

    /* validate arguments */
    if((bme680_handle && bme680_config) != NULL) {
        meas_cycles = os_to_meas_cycles[bme680_config->temperature_oversampling];
        meas_cycles += os_to_meas_cycles[bme680_config->pressure_oversampling];
        meas_cycles += os_to_meas_cycles[bme680_config->humidity_oversampling];

        /* TPH measurement duration */
        meas_dur = meas_cycles * UINT32_C(1963);
        meas_dur += UINT32_C(477 * 4); /* TPH switching duration */
        meas_dur += UINT32_C(477 * 5); /* Gas measurement duration */

        //if (op_mode != BME68X_PARALLEL_MODE) {
        //    meas_dur += UINT32_C(1000); /* Wake up duration of 1ms */
        //}
    }

    return meas_dur;
}



/**
 * @brief reads calibration factors onboard the bme680.  see datasheet for details.
 *
 * @param[in] bme680_handle bmp280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bme680_get_cal_factors(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* bme680 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0xe9, &bme680_handle->dev_cal_factors->par_T1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x8a, (uint16_t *)&bme680_handle->dev_cal_factors->par_T2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x8c, (uint8_t *)&bme680_handle->dev_cal_factors->par_T3) );
    /* bme680 attempt to request H1-H7 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0xe2, &bme680_handle->dev_cal_factors->par_H1) ); // 2-bytes
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0xe2, &bme680_handle->dev_cal_factors->par_H2) ); // 2-bytes
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xe4, (uint8_t *)&bme680_handle->dev_cal_factors->par_H3) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xe5, (uint8_t *)&bme680_handle->dev_cal_factors->par_H4) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xe6, (uint8_t *)&bme680_handle->dev_cal_factors->par_H5) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xe7, &bme680_handle->dev_cal_factors->par_H6) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xe8, (uint8_t *)&bme680_handle->dev_cal_factors->par_H7) );
    /* bme680 attempt to request P1-P10 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x8e, &bme680_handle->dev_cal_factors->par_P1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x90, (uint16_t *)&bme680_handle->dev_cal_factors->par_P2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x92, (uint8_t *)&bme680_handle->dev_cal_factors->par_P3) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x94, (uint16_t *)&bme680_handle->dev_cal_factors->par_P4) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x96, (uint16_t *)&bme680_handle->dev_cal_factors->par_P5) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x99, (uint8_t *)&bme680_handle->dev_cal_factors->par_P6) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x98, (uint8_t *)&bme680_handle->dev_cal_factors->par_P7) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x9c, (uint16_t *)&bme680_handle->dev_cal_factors->par_P8) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0x9e, (uint16_t *)&bme680_handle->dev_cal_factors->par_P9) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xa0, &bme680_handle->dev_cal_factors->par_P10) );
    /* bme680 attempt to request G1-G3 calibration values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xed, (uint8_t *)&bme680_handle->dev_cal_factors->par_G1) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint16(bme680_handle->i2c_dev_handle, 0xeb, (uint16_t *)&bme680_handle->dev_cal_factors->par_G2) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0xee, (uint8_t *)&bme680_handle->dev_cal_factors->par_G3) );
    /* bme680 attempt to request gas range and switching error values from device */
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x2b, (uint8_t *)&bme680_handle->dev_cal_factors->gas_range) );
    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, 0x04, (uint8_t *)&bme680_handle->dev_cal_factors->range_switching_error) );

    ESP_LOGD(TAG, "Calibration data received:");
    //
    ESP_LOGD(TAG, "par_T1=%u", bme680_handle->dev_cal_factors->par_T1);
    ESP_LOGD(TAG, "par_T2=%d", bme680_handle->dev_cal_factors->par_T2);
    ESP_LOGD(TAG, "par_T3=%d", bme680_handle->dev_cal_factors->par_T3);
    //
    ESP_LOGD(TAG, "par_P1=%u", bme680_handle->dev_cal_factors->par_P1);
    ESP_LOGD(TAG, "par_P2=%d", bme680_handle->dev_cal_factors->par_P2);
    ESP_LOGD(TAG, "par_P3=%d", bme680_handle->dev_cal_factors->par_P3);
    ESP_LOGD(TAG, "par_P4=%d", bme680_handle->dev_cal_factors->par_P4);
    ESP_LOGD(TAG, "par_P5=%d", bme680_handle->dev_cal_factors->par_P5);
    ESP_LOGD(TAG, "par_P6=%d", bme680_handle->dev_cal_factors->par_P6);
    ESP_LOGD(TAG, "par_P7=%d", bme680_handle->dev_cal_factors->par_P7);
    ESP_LOGD(TAG, "par_P8=%d", bme680_handle->dev_cal_factors->par_P8);
    ESP_LOGD(TAG, "par_P9=%d", bme680_handle->dev_cal_factors->par_P9);
    ESP_LOGD(TAG, "par_P10=%d", bme680_handle->dev_cal_factors->par_P10);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));
    
    return ESP_OK;
}

/**
 * @brief reads fixed measurements (temperature and pressure) from the bmp280.  see datasheet for details.
 *
 * @param[in] bme680_handle bmp280 device handle.
 * @param[out] temperature fixed temperature.
 * @param[out] pressure fixed temperature.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bme680_get_fixed_measurements(i2c_bme680_handle_t bme680_handle, int32_t *const temperature, uint32_t *const pressure) {
    esp_err_t       ret             = ESP_OK;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    int32_t         adc_press;
    int32_t         adc_temp;
    int32_t         fine_temp;
    i2c_uint48_t    data;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle && temperature && pressure );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_bme680_get_data_status(bme680_handle, &data_is_ready), err, TAG, "data ready ready for get fixed measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, I2C_BMP280_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte48(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_PRESSURE, &data), err, TAG, "read temperature and pressure data failed" );

    adc_press = data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp  = data[3] << 12 | data[4] << 4 | data[5] >> 4;

    ESP_LOGD(TAG, "ADC temperature: %" PRIi32, adc_temp);
    ESP_LOGD(TAG, "ADC pressure: %" PRIi32, adc_press);

    *temperature = i2c_bmp280_compensate_temperature(bme680_handle, adc_temp, &fine_temp);
    *pressure    = i2c_bmp280_compensate_pressure(bme680_handle, adc_press, fine_temp);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}


/**
 * @brief reads calibration factor, control measurement, and configuration registers from bmp280.
 * 
 * @param bme680_handle bmp280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_bme680_get_registers(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_cal_factors(bme680_handle), TAG, "read calibration factors for get registers failed" );

    /* attempt read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_control_measurement_register(bme680_handle), TAG, "read control measurement register for get registers failed" );

    /* attempt read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bmp280_get_configuration_register(bme680_handle), TAG, "read configuration register for get registers failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_get_chip_id_register(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_ID, &bme680_handle->chip_id), TAG, "read chip identifier register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bme680_get_status0_register(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_STATUS, &bme680_handle->status0_reg.reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bme680_get_control_measurement_register(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, &bme680_handle->ctrl_meas_reg.reg), TAG, "read control measurement register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bme680_set_control_measurement_register(i2c_bme680_handle_t bme680_handle, const i2c_bme680_control_measurement_register_t ctrl_meas_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_CTRL, ctrl_meas_reg.reg), TAG, "write control measurement register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_control_measurement_register(bme680_handle), TAG, "read control measurement register for set control measurement register failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_get_configuration_register(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, &bme680_handle->config_reg.reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_bme680_set_configuration_register(i2c_bme680_handle_t bme680_handle, const i2c_bme680_configuration_register_t config_reg) {
    i2c_bme680_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* copy register */
    config.reg = config_reg.reg;

    /* set reserved to 0 */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_configuration_register(bme680_handle), TAG, "read configuration register for set configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_init(i2c_master_bus_handle_t bus_handle, const i2c_bme680_config_t *bme680_config, i2c_bme680_handle_t *bme680_handle) {
    i2c_bme680_configuration_register_t         config_reg;
    i2c_bme680_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && bme680_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, bme680_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bmp280 device handle initialization failed", bme680_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_bme680_handle_t out_handle = (i2c_bme680_handle_t)calloc(1, sizeof(i2c_bme680_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device for init");

    /* validate memory availability for handle calibration factors */
    out_handle->dev_cal_factors = (i2c_bme680_cal_factors_t*)calloc(1, sizeof(i2c_bme680_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device calibration factors for init");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = bme680_config->dev_config.device_address,
        .scl_speed_hz       = bme680_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(i2c_bme680_get_chip_id_register(out_handle), err_handle, TAG, "read chip identifier for init failed");
    if(out_handle->chip_id != I2C_BME680_CHIP_ID) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", out_handle->chip_id);
    }

    /* attempt to reset the device and initialize registers */
    ESP_GOTO_ON_ERROR(i2c_bme680_reset(out_handle), err_handle, TAG, "soft-reset and initialize registers for init failed");

    /* copy configuration and control measurement registers from handle */
    config_reg.reg      = out_handle->config_reg.reg;
    ctrl_meas_reg.reg   = out_handle->ctrl_meas_reg.reg;

    /* initialize configuration register from configuration params */
    config_reg.bits.iir_filter   = bme680_config->iir_filter;

    /* initialize control measurement register from configuration params */
    if (bme680_config->power_mode == I2C_BME680_POWER_MODE_FORCED) {
        // initial mode for forced is sleep
        ctrl_meas_reg.bits.power_mode               = I2C_BME680_POWER_MODE_SLEEP;
        ctrl_meas_reg.bits.temperature_oversampling = bme680_config->temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = bme680_config->pressure_oversampling;
    } else {
        ctrl_meas_reg.bits.power_mode               = bme680_config->power_mode;
        ctrl_meas_reg.bits.temperature_oversampling = bme680_config->temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = bme680_config->pressure_oversampling;
    }
    
    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(i2c_bme680_set_configuration_register(out_handle, config_reg), err_handle, TAG, "write configuration register for init failed");

    /* attempt to write control measurement register */
    ESP_GOTO_ON_ERROR(i2c_bme680_set_control_measurement_register(out_handle, ctrl_meas_reg), err_handle, TAG, "write control measurement register for init failed");

    /* copy configuration */
    *bme680_handle = out_handle;

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

esp_err_t i2c_bme680_get_measurements(i2c_bme680_handle_t bme680_handle, float *const temperature, float *const pressure) {
    int32_t  fixed_temperature;
    uint32_t fixed_pressure;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle && temperature && pressure );

    /* attempt to read fixed measurements (temperature & pressure) */
    ESP_ERROR_CHECK( i2c_bme680_get_fixed_measurements(bme680_handle, &fixed_temperature, &fixed_pressure) );

    /* set output parameters */
    *temperature = (float)fixed_temperature / 100;
    *pressure    = (float)fixed_pressure / 256;

    return ESP_OK;
}



esp_err_t i2c_bme680_get_data_status(i2c_bme680_handle_t bme680_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_status_register(bme680_handle), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    if(bme680_handle->status0_reg.bits.measuring == true) {
        *ready = false;
    } else {
        *ready = true;
    }

    return ESP_OK;
}

esp_err_t i2c_bme680_get_power_mode(i2c_bme680_handle_t bme680_handle, i2c_bme680_power_modes_t *const power_mode) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_control_measurement_register(bme680_handle), TAG, "read control measurement register for get power mode failed" );

    /* set power mode */
    *power_mode = bme680_handle->ctrl_meas_reg.bits.power_mode;

    return ESP_OK;
}

esp_err_t i2c_bme680_set_power_mode(i2c_bme680_handle_t bme680_handle, const i2c_bme680_power_modes_t power_mode) {
    i2c_bme680_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bme680_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.power_mode = power_mode;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_set_control_measurement_register(bme680_handle, ctrl_meas_reg), TAG, "write control measurement register for set power mode failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_get_pressure_oversampling(i2c_bme680_handle_t bme680_handle, i2c_bme680_pressure_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_control_measurement_register(bme680_handle), TAG, "read control measurement register for get pressure oversampling failed" );

    /* set oversampling */
    *oversampling = bme680_handle->ctrl_meas_reg.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bme680_set_pressure_oversampling(i2c_bme680_handle_t bme680_handle, const i2c_bme680_pressure_oversampling_t oversampling) {
    i2c_bme680_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bme680_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_set_control_measurement_register(bme680_handle, ctrl_meas_reg), TAG, "write control measurement register for set pressure oversampling failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_get_temperature_oversampling(i2c_bme680_handle_t bme680_handle, i2c_bme680_temperature_oversampling_t *const oversampling) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_control_measurement_register(bme680_handle), TAG, "read control measurement register for get temperature oversampling failed" );

    /* set oversampling */
    *oversampling = bme680_handle->ctrl_meas_reg.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t i2c_bme680_set_temperature_oversampling(i2c_bme680_handle_t bme680_handle, const i2c_bme680_temperature_oversampling_t oversampling) {
    i2c_bme680_control_measurement_register_t   ctrl_meas_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* copy control measurement register from handle */
    ctrl_meas_reg.reg = bme680_handle->ctrl_meas_reg.reg;

    /* initialize control measurement register */
    ctrl_meas_reg.bits.temperature_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( i2c_bme680_set_control_measurement_register(bme680_handle, ctrl_meas_reg), TAG, "write control measurement register for set temperature oversampling failed" );

    return ESP_OK;
}


esp_err_t i2c_bme680_get_iir_filter(i2c_bme680_handle_t bme680_handle, i2c_bme680_iir_filters_t *const iir_filter) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_bme680_get_configuration_register(bme680_handle), TAG, "read configuration register for get IIR filter failed" );

    /* set standby time */
    *iir_filter = bme680_handle->config_reg.bits.iir_filter;

    return ESP_OK;
}

esp_err_t i2c_bme680_set_iir_filter(i2c_bme680_handle_t bme680_handle, const i2c_bme680_iir_filters_t iir_filter) {
    i2c_bme680_configuration_register_t   config_reg;

    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* copy configuration register from handle */
    config_reg.reg = bme680_handle->config_reg.reg;

    /* initialize configuration register */
    config_reg.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_bme680_set_configuration_register(bme680_handle, config_reg), TAG, "write configuration register for set IIR filter failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_reset(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(bme680_handle->i2c_dev_handle, I2C_BMP280_REG_RESET, I2C_BMP280_RESET_VALUE), TAG, "write reset register for reset failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(I2C_BMP280_RESET_DELAY_MS)); // check is busy in timeout loop...

    ESP_RETURN_ON_ERROR( i2c_bme680_get_registers(bme680_handle), TAG, "read registers for reset failed" );

    return ESP_OK;
}

esp_err_t i2c_bme680_remove(i2c_bme680_handle_t bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    return i2c_master_bus_rm_device(bme680_handle->i2c_dev_handle);
}

esp_err_t i2c_bme680_delete(i2c_bme680_handle_t bme680_handle){
    /* validate arguments */
    ESP_ARG_CHECK( bme680_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_bme680_remove(bme680_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(bme680_handle->i2c_dev_handle) {
        free(bme680_handle->i2c_dev_handle);
        free(bme680_handle);
    }

    return ESP_OK;
}
