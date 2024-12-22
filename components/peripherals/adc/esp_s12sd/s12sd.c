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
 * @file s12sd.c
 *
 * ESP-IDF driver for GUVA-S12SD UV sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "s12sd.h"
#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_system.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "s12sd";

/*
* functions and subrountines
*/

/**
 * @brief converts millivolt (0 to 1500mV) to uv index. see GUVA-S12SD datasheet for details.
 *
 * @param[in] milli_volt voltage, in millivolts, to convert
 * @return uv index (0 to 11), an out-of-range value returns 255
 */
static inline uint8_t adc_s12sd_convert_uv_index(const float milli_volt) {
    if(milli_volt > ADC_UV_MV_TO_INDEX_0_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_0_MAX) {
        return 0;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_1_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_1_MAX) {
        return 1;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_2_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_2_MAX) {
        return 2;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_3_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_3_MAX) {
        return 3;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_4_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_4_MAX) {
        return 4;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_5_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_5_MAX) {
        return 5;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_6_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_6_MAX) {
        return 6;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_7_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_7_MAX) {
        return 7;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_8_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_8_MAX) {
        return 8;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_9_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_9_MAX) {
        return 9;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_10_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_10_MAX) {
        return 10;
    } else if(milli_volt > ADC_UV_MV_TO_INDEX_11_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_11_MAX) {
        return 11;
    } else {
        return 255;
    }
}

static inline bool adc_s12sd_calibration_init(const adc_s12sd_config_t *s12sd_config, adc_cali_handle_t *cal_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = s12sd_config->unit,
            .chan = s12sd_config->channel,
            .atten = ADC_UV_ATTEN,
            .bitwidth = ADC_UV_DIGI_BIT_WIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = s12sd_config->unit,
            .atten = ADC_UV_ATTEN,
            .bitwidth = ADC_UV_DIGI_BIT_WIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *cal_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static inline void adc_s12sd_calibration_deinit(adc_cali_handle_t cal_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK( adc_cali_delete_scheme_curve_fitting(cal_handle) );

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK( adc_cali_delete_scheme_line_fitting(cal_handle) );
#endif
}

esp_err_t adc_s12sd_init(const adc_s12sd_config_t *s12sd_config, adc_s12sd_handle_t *s12sd_handle) {
    esp_err_t           ret = ESP_OK;
    adc_s12sd_handle_t  out_handle;

    ESP_ARG_CHECK( s12sd_config && s12sd_handle );

    out_handle = (adc_s12sd_handle_t)calloc(1, sizeof(adc_s12sd_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for adc s12sd device");

    adc_oneshot_unit_init_cfg_t init_conf = {
        .unit_id = s12sd_config->unit,
    };

    ESP_GOTO_ON_ERROR(adc_oneshot_new_unit(&init_conf, &out_handle->adc_dev_handle), err, TAG, "adc s12sd device new one-shot handle failed");

    adc_oneshot_chan_cfg_t os_conf = {
        .bitwidth = ADC_UV_DIGI_BIT_WIDTH,
        .atten    = ADC_UV_ATTEN,
    };

    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(out_handle->adc_dev_handle, s12sd_config->channel, &os_conf), err, TAG, "adc s12sd device configuration (one-shot) failed");

    out_handle->adc_calibrate = adc_s12sd_calibration_init(s12sd_config, &out_handle->adc_cal_handle);

    /* copy configuration and set device handle */
    out_handle->adc_unit            = s12sd_config->unit;
    out_handle->adc_channel         = s12sd_config->channel;
    *s12sd_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->adc_dev_handle) {
            adc_oneshot_del_unit(out_handle->adc_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t adc_s12sd_measure(adc_s12sd_handle_t s12sd_handle, uint8_t *uv_index) {
    esp_err_t   ret         = ESP_OK;
    int         avg_sum     = 0;
    float       avg_volt    = 0;

    ESP_ARG_CHECK( s12sd_handle && uv_index );

    for (int i=0; i<ADC_UV_SAMPLE_SIZE; i++) {
        int adc_raw;
        int adc_volt;

        ret = adc_oneshot_read(s12sd_handle->adc_dev_handle, s12sd_handle->adc_channel, &adc_raw);

        if (ret == ESP_OK && s12sd_handle->adc_calibrate == true) {
            ret = adc_cali_raw_to_voltage(s12sd_handle->adc_cal_handle, adc_raw, &adc_volt);

            if(ret != ESP_OK) return ret; // abort altogether

            avg_sum += adc_volt;
        } else {
            if(s12sd_handle->adc_calibrate == false) return ESP_ERR_INVALID_STATE;

            return ret; // abort altogether
        }
    }

    // average voltage (mV)
    avg_volt = avg_sum / ADC_UV_SAMPLE_SIZE;

    // convert voltage to uv index
    *uv_index = adc_s12sd_convert_uv_index(avg_volt);

    //ESP_LOGI(TAG, "ADC%d Channel[%d] Samples(%d) Cali Voltage: %.2f mV", s12sd_handle->adc_unit, s12sd_handle->adc_channel, ADC_UV_SAMPLE_SIZE, avg_volt);

    //ESP_LOGI(TAG, "Unit: 1, Channel: 0, Value (mV): %.2f, UV Index: %u", avg_volt, *uv_index);

    return ESP_OK;
}


esp_err_t adc_s12sd_deinit(adc_s12sd_handle_t s12sd_handle) {
    esp_err_t ret = ESP_OK;

    ESP_ARG_CHECK( s12sd_handle );

    ret = adc_oneshot_del_unit(s12sd_handle->adc_dev_handle);

    if (s12sd_handle->adc_calibrate) {
        adc_s12sd_calibration_deinit(s12sd_handle->adc_cal_handle);
    }

    return ret;
}