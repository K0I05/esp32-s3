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
 * @file s12sd.h
 * @defgroup drivers guva-s12sd uv sensor
 * @{
 *
 * ESP-IDF driver for guva-s12sd uv sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __S12SD_H__
#define __S12SD_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_system.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GUVA-S12SD definitions
*/

#define ADC_UV_UNIT_DEFAULT        ADC_UNIT_1
#define ADC_UV_CHANNEL_DEFAULT     ADC_CHANNEL_0
#define ADC_UV_SAMPLE_SIZE         (1000)
#define ADC_UV_ATTEN               ADC_ATTEN_DB_12
#define ADC_UV_DIGI_BIT_WIDTH      (12)   //!< adc bit width at 12-bits

/* UV millivolt to uv index upper and lower limit definitions
 * see GUVA-S12SD datasheet for details
*/

#define ADC_UV_MV_TO_INDEX_0_MIN    (-1)
#define ADC_UV_MV_TO_INDEX_0_MAX    (49)
#define ADC_UV_MV_TO_INDEX_1_MIN    ADC_UV_MV_TO_INDEX_0_MAX
#define ADC_UV_MV_TO_INDEX_1_MAX    (227)
#define ADC_UV_MV_TO_INDEX_2_MIN    ADC_UV_MV_TO_INDEX_1_MAX
#define ADC_UV_MV_TO_INDEX_2_MAX    (318)
#define ADC_UV_MV_TO_INDEX_3_MIN    ADC_UV_MV_TO_INDEX_2_MAX
#define ADC_UV_MV_TO_INDEX_3_MAX    (408)
#define ADC_UV_MV_TO_INDEX_4_MIN    ADC_UV_MV_TO_INDEX_3_MAX
#define ADC_UV_MV_TO_INDEX_4_MAX    (503)
#define ADC_UV_MV_TO_INDEX_5_MIN    ADC_UV_MV_TO_INDEX_4_MAX
#define ADC_UV_MV_TO_INDEX_5_MAX    (606)
#define ADC_UV_MV_TO_INDEX_6_MIN    ADC_UV_MV_TO_INDEX_5_MAX
#define ADC_UV_MV_TO_INDEX_6_MAX    (696)
#define ADC_UV_MV_TO_INDEX_7_MIN    ADC_UV_MV_TO_INDEX_6_MAX
#define ADC_UV_MV_TO_INDEX_7_MAX    (795)
#define ADC_UV_MV_TO_INDEX_8_MIN    ADC_UV_MV_TO_INDEX_7_MAX
#define ADC_UV_MV_TO_INDEX_8_MAX    (881)
#define ADC_UV_MV_TO_INDEX_9_MIN    ADC_UV_MV_TO_INDEX_8_MAX
#define ADC_UV_MV_TO_INDEX_9_MAX    (976)
#define ADC_UV_MV_TO_INDEX_10_MIN   ADC_UV_MV_TO_INDEX_9_MAX
#define ADC_UV_MV_TO_INDEX_10_MAX   (1079)
#define ADC_UV_MV_TO_INDEX_11_MIN   ADC_UV_MV_TO_INDEX_10_MAX
#define ADC_UV_MV_TO_INDEX_11_MAX   (1500) // 1170+ but set a max of 1500

/* macro definotions */

#define ADC_UV_S12SD_CONFIG_DEFAULT {               \
    .unit = ADC_UV_UNIT_DEFAULT,                    \
    .channel = ADC_UV_CHANNEL_DEFAULT,  } 

/*
 * GUVA-S12SD prototype, enumerator and sructure declerations
*/

typedef struct i2c_s12sd_t i2c_s12sd_t;
typedef struct adc_s12sd_t *adc_s12sd_handle_t;

/**
 * @brief adc s12sd device configuration.
 */
typedef struct {
    uint8_t     unit;     /*!< adc unit */
    uint8_t     channel;  /*!< adc channel */
} adc_s12sd_config_t;

/**
 * @brief adc s12sd device handle.
 */
struct adc_s12sd_t {
    adc_oneshot_unit_handle_t   adc_dev_handle; /*!< adc device handle */
    adc_cali_handle_t           adc_cal_handle; /*!< adc calibration handle */
    bool                        adc_calibrate;  /*!< adc calibration initialization flag */
    uint8_t                     adc_unit;       /*!< adc unit */
    uint8_t                     adc_channel;    /*!< adc channel */
};


/**
 * @brief initializes an adc s12sd device.
 *
 * @param[in] s12sd_config configuration of s12sd device
 * @param[out] s12sd_handle s12sd device handle
 * @return ESP_OK: init success.
 */
esp_err_t adc_s12sd_init(const adc_s12sd_config_t *s12sd_config, adc_s12sd_handle_t *s12sd_handle);

/**
 * @brief Measure s12sd device.
 *
 * @param[in] s12sd_handle s12sd device handle
 * @param[out] uv_index uv index (1 to 11)
 * @return ESP_OK: init success.
 */
esp_err_t adc_s12sd_measure(adc_s12sd_handle_t s12sd_handle, uint8_t *uv_index);


/**
 * @brief Deinitialized s12sd device.
 *
 * @param[in] s12sd_handle s12sd device handle
 * @return ESP_OK: init success.
 */
esp_err_t adc_s12sd_deinit(adc_s12sd_handle_t s12sd_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __S12SD_H__
