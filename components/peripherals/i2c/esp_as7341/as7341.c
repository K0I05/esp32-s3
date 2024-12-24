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
 * AS7341 definitions
*/

#define I2C_AS7341_PART_ID              UINT8_C(0x09)   //!< as7341 I2C device part identifier

//#define I2C_AS7341_ASTATUS              (0x60)  //!< as7341 (see i2c_as7341_astatus_register_t)
//#define I2C_AS7341_CH0_ADC_DATA_L       (0x61)  //!< as7341
//#define I2C_AS7341_CH0_ADC_DATA_H       (0x62)  //!< as7341
#define I2C_AS7341_ITIME_L              UINT8_C(0x63)  //!< as7341
#define I2C_AS7341_ITIME_M              UINT8_C(0x64)  //!< as7341
#define I2C_AS7341_ITIME_H              UINT8_C(0x65)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_L       (0x66)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_H       (0x67)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_L       (0x68)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_H       (0x69)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_L       (0x6a)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_H       (0x6b)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_L       (0x6c)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_H       (0x6d)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_L       (0x6e)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_H       (0x6f)  //!< as7341

#define I2C_AS7341_CONFIG               UINT8_C(0x70)  //!< as7341 (see i2c_as7341_config_register_t)
#define I2C_AS7341_DEV_STATUS           UINT8_C(0x71)  //!< as7341 (see i2c_as7341_device_status_register_t)
#define I2C_AS7341_EDGE                 UINT8_C(0x72)  //!< as7341
#define I2C_AS7341_GPIO1                UINT8_C(0x73)  //!< as7341 (see i2c_as7341_gpio1_register_t)
#define I2C_AS7341_LED                  UINT8_C(0x74)  //!< as7341 (see i2c_as7341_led_register_t)
#define I2C_AS7341_ENABLE               UINT8_C(0x80)  //!< as7341 (see i2c_as7341_enable_register_t)
#define I2C_AS7341_ATIME                UINT8_C(0x81)  //!< as7341
#define I2C_AS7341_WTIME                UINT8_C(0x83)  //!< as7341
#define I2C_AS7341_SP_TH_L_LSB          UINT8_C(0x84)  //!< as7341
#define I2C_AS7341_SP_TH_L_MSB          UINT8_C(0x85)  //!< as7341
#define I2C_AS7341_SP_TH_H_LSB          UINT8_C(0x86)  //!< as7341
#define I2C_AS7341_SP_TH_H_MSB          UINT8_C(0x87)  //!< as7341
#define I2C_AS7341_AUXID                UINT8_C(0x90)  //!< as7341
#define I2C_AS7341_REVID                UINT8_C(0x91)  //!< as7341
#define I2C_AS7341_ID                   UINT8_C(0x92)  //!< as7341

#define I2C_AS7341_INT_STATUS           UINT8_C(0x93)  //!< as7341 (see i2c_as7341_interrupt_status_register_t)

#define I2C_AS7341_ASTATUS              UINT8_C(0x94)  //!< as7341 (see i2c_as7341_astatus_register_t)
#define I2C_AS7341_CH0_ADC_DATA_L       UINT8_C(0x95)  //!< as7341
#define I2C_AS7341_CH0_ADC_DATA_H       UINT8_C(0x96)  //!< as7341
#define I2C_AS7341_CH1_ADC_DATA_L       UINT8_C(0x97)  //!< as7341
#define I2C_AS7341_CH1_ADC_DATA_H       UINT8_C(0x98)  //!< as7341
#define I2C_AS7341_CH2_ADC_DATA_L       UINT8_C(0x99)  //!< as7341
#define I2C_AS7341_CH2_ADC_DATA_H       UINT8_C(0x9a)  //!< as7341
#define I2C_AS7341_CH3_ADC_DATA_L       UINT8_C(0x9b)  //!< as7341
#define I2C_AS7341_CH3_ADC_DATA_H       UINT8_C(0x9c)  //!< as7341
#define I2C_AS7341_CH4_ADC_DATA_L       UINT8_C(0x9d)  //!< as7341
#define I2C_AS7341_CH4_ADC_DATA_H       UINT8_C(0x9e)  //!< as7341
#define I2C_AS7341_CH5_ADC_DATA_L       UINT8_C(0x9f)  //!< as7341
#define I2C_AS7341_CH5_ADC_DATA_H       UINT8_C(0xa0)  //!< as7341

#define I2C_AS7341_STATUS2              UINT8_C(0xa3)  //!< as7341 (see i2c_as7341_status2_register_t)
#define I2C_AS7341_STATUS3              UINT8_C(0xa4)  //!< as7341 (see i2c_as7341_status3_register_t)
#define I2C_AS7341_STATUS5              UINT8_C(0xa6)  //!< as7341 (see i2c_as7341_status5_register_t)
#define I2C_AS7341_STATUS6              UINT8_C(0xa7)  //!< as7341 (see i2c_as7341_status6_register_t)

#define I2C_AS7341_CONFIG0              UINT8_C(0xa9)  //!< as7341 (see i2c_as7341_config0_register_t)
#define I2C_AS7341_CONFIG1              UINT8_C(0xaa)  //!< as7341 (see i2c_as7341_config1_register_t)
#define I2C_AS7341_CONFIG3              UINT8_C(0xac)  //!< as7341 (see i2c_as7341_config3_register_t)
#define I2C_AS7341_CONFIG6              UINT8_C(0xaf)  //!< as7341 (see i2c_as7341_config6_register_t)
#define I2C_AS7341_CONFIG8              UINT8_C(0xb1)  //!< as7341 (see i2c_as7341_config8_register_t)
#define I2C_AS7341_CONFIG9              UINT8_C(0xb2)  //!< as7341 (see i2c_as7341_config9_register_t)
#define I2C_AS7341_CONFIG10             UINT8_C(0xb3)  //!< as7341 (see i2c_as7341_config10_register_t)
#define I2C_AS7341_CONFIG12             UINT8_C(0xb5)  //!< as7341 (see i2c_as7341_config12_register_t)

#define I2C_AS7341_PERS                 UINT8_C(0xbd)  //!< as7341 (see i2c_as7341_pers_register_t)
#define I2C_AS7341_GPIO2                UINT8_C(0xbe)  //!< as7341 (see i2c_as7341_gpio2_register_t)
#define I2C_AS7341_ASTEP_L              UINT8_C(0xca)  //!< as7341
#define I2C_AS7341_ASTEP_H              UINT8_C(0xcb)  //!< as7341
#define I2C_AS7341_AGC_GAIN_MAX         UINT8_C(0xcf)  //!< as7341 (see i2c_as7341_agc_gain_register_t)
#define I2C_AS7341_AZ_CONFIG            UINT8_C(0xd6)  //!< as7341
#define I2C_AS7341_FD_TIME1             UINT8_C(0xd8)  //!< as7341
#define I2C_AS7341_FD_TIME2             UINT8_C(0xda)  //!< as7341 (see i2c_as7341_fd_time2_register_t)
#define I2C_AS7341_FD_CONFIG0           UINT8_C(0xd7)  //!< as7341 (see i2c_as7341_fd_config0_register_t)
#define I2C_AS7341_FD_STATUS            UINT8_C(0xdb)  //!< as7341 (see i2c_as7341_fd_status_register_t)
#define I2C_AS7341_INTENAB              UINT8_C(0xf9)  //!< as7341 (see i2c_as7341_interrupt_enable_register_t)
#define I2C_AS7341_CONTROL              UINT8_C(0xfa)  //!< as7341 (see i2c_as7341_control_register_t)


#define I2C_AS7341_DATA_POLL_TIMEOUT_MS UINT16_C(1000)
#define I2C_AS7341_DATA_READY_DELAY_MS  UINT16_C(1)
#define I2C_AS7341_POWERUP_DELAY_MS     UINT16_C(200)
#define I2C_AS7341_APPSTART_DELAY_MS    UINT16_C(200)
#define I2C_AS7341_RESET_DELAY_MS       UINT16_C(25)
#define I2C_AS7341_SETUP_DELAY_MS       UINT16_C(15)
#define I2C_AS7341_CMD_DELAY_MS         UINT16_C(5)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "as7341";



/**
 * @brief Converts an ADC spectral gain sensitivity enumerator to a spectral gain sensitivity multiplier.
 * 
 * @param gain ADC spectral gain sensitivity enumerator.
 * @return float ADC spectral gain sensitivity multiplier.
 */
static inline float i2c_as7341_get_spectral_gain_sensitivity(i2c_as7341_spectral_gains_t gain) {
    /* determine gain sensitivity */
    switch (gain) {
        case I2C_AS7341_SPECTRAL_GAIN_0_5X:
            return 0.5;
        case I2C_AS7341_SPECTRAL_GAIN_1X:
            return 1;
        case I2C_AS7341_SPECTRAL_GAIN_2X:
            return 2;
        case I2C_AS7341_SPECTRAL_GAIN_4X:
            return 4;
        case I2C_AS7341_SPECTRAL_GAIN_8X:
            return 8;
        case I2C_AS7341_SPECTRAL_GAIN_16X:
            return 16;
        case I2C_AS7341_SPECTRAL_GAIN_32X:
            return 32;
        case I2C_AS7341_SPECTRAL_GAIN_64X:
            return 64;
        case I2C_AS7341_SPECTRAL_GAIN_128X:
            return 128;
        case I2C_AS7341_SPECTRAL_GAIN_256X:
            return 256;
        case I2C_AS7341_SPECTRAL_GAIN_512X:
            return 512;
        default:
            return 1;
    }
}

/**
 * @brief Calculated integration time (𝑡𝑖𝑛𝑡 = (𝐴𝑇𝐼𝑀𝐸 + 1) × (𝐴𝑆𝑇𝐸𝑃 + 1) × 2.78μ𝑠), from ATIME and ASTEP registers, in milli-seconds.
 * 
 * @param as7341_handle AS7341 device handle.
 * @param time Integration time in milli-seconds
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_get_integration_time(i2c_as7341_handle_t as7341_handle, double *time) {
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

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

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

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for low channels (F1-F4, Clear and NIR).
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_set_smux_lo_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    ESP_RETURN_ON_ERROR( i2c_as7341_disable_spectral_measurement(as7341_handle), TAG, "disable spectral measurement for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_set_smux_command(as7341_handle, I2C_AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_setup_smux_lo_channels(as7341_handle), TAG, "setup SMUX low channels for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_enable_smux(as7341_handle), TAG, "enable SMUX for set SMUX low channels failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for high channels (F5-F8, Clear and NIR).
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_set_smux_hi_channels(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    ESP_RETURN_ON_ERROR( i2c_as7341_disable_spectral_measurement(as7341_handle), TAG, "disable spectral measurement for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_set_smux_command(as7341_handle, I2C_AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_setup_smux_hi_channels(as7341_handle), TAG, "setup SMUX high channels for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( i2c_as7341_enable_smux(as7341_handle), TAG, "enable SMUX for set SMUX high channels failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for flicker detection.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_setup_smux_flicker_detection(i2c_as7341_handle_t as7341_handle) {
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

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads all registers from AS7341 and initializes handle registers.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_as7341_get_registers(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read LED register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_led_register(as7341_handle), TAG, "read LED register for read registers failed");

    /* attempt to read ASTATUS register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_astatus_register(as7341_handle), TAG, "read ASTATUS register for read registers failed");

    /* attempt to read status 2 register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_status2_register(as7341_handle), TAG, "read status 2 register for read registers failed");

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_enable_register(as7341_handle), TAG, "read enable register for read registers failed");

    /* attempt to read aux id register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_auxiliary_id_register(as7341_handle), TAG, "read auxiliary id register for read registers failed");

    /* attempt to read revision id register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_revision_id_register(as7341_handle), TAG, "read revision id register for read registers failed");

    /* attempt to read part id register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_part_id_register(as7341_handle), TAG, "read part id register for read registers failed");

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_config_register(as7341_handle), TAG, "read configuration register for read registers failed");

    /* attempt to read configuration 0 register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_config0_register(as7341_handle), TAG, "read configuration 0 register for read registers failed");

    /* attempt to read configuration 1 register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for read registers failed");

    /* attempt to read configuration 6 register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_config6_register(as7341_handle), TAG, "read configuration 6 register for read registers failed");

    /* attempt to read flicker detection status register */
    ESP_RETURN_ON_ERROR(i2c_as7341_get_flicker_detection_status_register(as7341_handle), TAG, "read flicker deteciont status register for read registers failed");

    return ESP_OK;
}

esp_err_t i2c_as7341_get_led_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_LED, &as7341_handle->led_reg.reg), TAG, "read LED register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_led_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_led_register_t led_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_LED, led_reg.reg), TAG, "write LED register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_led_register(as7341_handle), TAG, "read LED register for set LED register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_astatus_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTATUS, &as7341_handle->astatus_reg.reg), TAG, "read astatus register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_get_status2_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_STATUS2, &as7341_handle->status2_reg.reg), TAG, "read status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_enable_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    i2c_as7341_enable_register_t enable = { .reg = 0 };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable), TAG, "disable enable register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_enable_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ENABLE, &as7341_handle->enable_reg.reg), TAG, "read enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_enable_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_enable_register_t enable_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register */
    i2c_as7341_enable_register_t enable = { .reg = enable_reg.reg };

    /* set reserved bits */
    enable.bits.reserved1 = 0;
    enable.bits.reserved2 = 0;
    enable.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ENABLE, enable.reg), TAG, "write enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_enable_register(as7341_handle), TAG, "read enable register for set enable register failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_get_auxiliary_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_AUXID, &as7341_handle->aux_id_reg.reg), TAG, "read auxiliary id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_get_revision_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_REVID, &as7341_handle->revision_id_reg.reg), TAG, "read revision id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_get_part_id_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );


    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ID, &as7341_handle->part_id_reg.reg), TAG, "read part id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_get_config_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG, &as7341_handle->config_reg.reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_config_register_t config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register */
    i2c_as7341_config_register_t config = { .reg = config_reg.reg };

    /* set reserved bits */
    config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config_register(as7341_handle), TAG, "read configuration register for set configuration register failed" );

    return ESP_OK;
}


esp_err_t i2c_as7341_get_config0_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG0, &as7341_handle->config0_reg.reg), TAG, "read configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config0_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_config0_register_t config0_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register */
    i2c_as7341_config0_register_t config0 = { .reg = config0_reg.reg };

    /* set reserved bits */
    config0.bits.reserved1 = 0;
    config0.bits.reserved2 = 0;
    config0.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG0, config0.reg), TAG, "write configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config0_register(as7341_handle), TAG, "read configuration 0 register for set configuration 0 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_config1_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG1, &as7341_handle->config1_reg.reg), TAG, "read configuration 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config1_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_config1_register_t config1_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy regiser */
    i2c_as7341_config1_register_t config1 = { .reg = config1_reg.reg };

    /* set reserved bits */
    config1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG1, config1.reg), TAG, "write configurtion 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for set configuration 1 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_config6_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG6, &as7341_handle->config6_reg.reg), TAG, "read configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_config6_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_config6_register_t config6_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register */
    i2c_as7341_config6_register_t config6 = { .reg = config6_reg.reg };

    /* set reserved bits */
    config6.bits.reserved1 = 0;
    config6.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_CONFIG6, config6.reg), TAG, "write configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config6_register(as7341_handle), TAG, "read configuration 6 register for set configuration 6 register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_atime_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ATIME, &as7341_handle->atime_reg), TAG, "read atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_atime_register(i2c_as7341_handle_t as7341_handle, const uint8_t atime_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_ATIME, atime_reg), TAG, "write atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_atime_register(as7341_handle), TAG, "read atime register for set atime register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_astep_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTEP_L, &as7341_handle->astep_reg), TAG, "read astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_astep_register(i2c_as7341_handle_t as7341_handle, const uint16_t astep_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(as7341_handle->i2c_dev_handle, I2C_AS7341_ASTEP_L, astep_reg), TAG, "write astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_astep_register(as7341_handle), TAG, "read astep register for set astep register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_flicker_detection_status_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_FD_STATUS, &as7341_handle->fd_status_reg.reg), TAG, "read flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_as7341_set_flicker_detection_status_register(i2c_as7341_handle_t as7341_handle, const i2c_as7341_flicker_detection_status_register_t flicker_detection_status_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register */
    i2c_as7341_flicker_detection_status_register_t flicker_detection_status = { .reg = flicker_detection_status_reg.reg };

    /* set reserved bits */
    flicker_detection_status.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(as7341_handle->i2c_dev_handle, I2C_AS7341_FD_STATUS, flicker_detection_status.reg), TAG, "write flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_flicker_detection_status_register(as7341_handle), TAG, "read flicker detection status register for set flicker detection status register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_clear_flicker_detection_status_register(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* clear resettable flags */
    i2c_as7341_flicker_detection_status_register_t flicker_detection_status = {
        .bits.fd_saturation_detected = true,
        .bits.fd_measurement_valid   = true,
        .bits.fd_100hz_flicker_valid = true,
        .bits.fd_120hz_flicker_valid = true
    };

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_flicker_detection_status_register(as7341_handle, flicker_detection_status), TAG, "write flicker detection status register for clear flicker detection status register failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_hi_register_bank(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_config0_register_t config0_reg = { .reg = as7341_handle->config0_reg.reg };

    /* enable high registers */
    config0_reg.bits.reg_bank_access = false; // 0 or false to access register 0x80 and above

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config0_register(as7341_handle, config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_lo_register_bank(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_config0_register_t config0_reg = { .reg = as7341_handle->config0_reg.reg };

    /* enable low registers */
    config0_reg.bits.reg_bank_access = true; // 1 or true to access register 0x60 to 0x74

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config0_register(as7341_handle, config0_reg), TAG, "write configuration 0 register for enable low registers failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_set_smux_command(i2c_as7341_handle_t as7341_handle, const i2c_as7341_smux_commands_t command) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* set smux command */
    i2c_as7341_config6_register_t config6_reg = { .bits.smux_command = command };

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config6_register(as7341_handle, config6_reg), TAG, "write configuration 6 register for set smux command failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_init(i2c_master_bus_handle_t bus_handle, const i2c_as7341_config_t *as7341_config, i2c_as7341_handle_t *as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && as7341_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, as7341_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, as7341 device handle initialization failed", as7341_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_as7341_handle_t out_handle = (i2c_as7341_handle_t)calloc(1, sizeof(i2c_as7341_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as7341 device, init failed");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = as7341_config->dev_config.device_address,
        .scl_speed_hz       = as7341_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    /* attempt to enable power */
    ESP_GOTO_ON_ERROR(i2c_as7341_enable_power(out_handle), err_handle, TAG, "enable power for init failed");

    /* attempt to read handle registers */
    ESP_GOTO_ON_ERROR(i2c_as7341_get_registers(out_handle), err_handle, TAG, "read handle registers for init failed");

    /* copy configuration */

    /* attempt to write atime configuration */
    ESP_GOTO_ON_ERROR(i2c_as7341_set_atime(out_handle, as7341_config->atime), err_handle, TAG, "write atime for init failed");

    /* attempt to write astep configuration */
    ESP_GOTO_ON_ERROR(i2c_as7341_set_astep(out_handle, as7341_config->astep), err_handle, TAG, "write astep for init failed");

    /*attempt to write spectral gain configuration */
    ESP_GOTO_ON_ERROR(i2c_as7341_set_spectral_gain(out_handle, as7341_config->spectral_gain), err_handle, TAG, "write spectral gain for init failed");

    /* set device handle */
    *as7341_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_as7341_get_spectral_measurements(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_spectral_data_t *spectral_data) {
    esp_err_t   ret              = ESP_OK;
    double      integration_time = 0;
    uint64_t    start_time       = 0;
    bool        data_is_ready    = false;
    i2c_uint8_t tx               = { I2C_AS7341_CH0_ADC_DATA_L };
    uint8_t     rx[12]           = { 0 };

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
    spectral_data->f1 = rx[0]  | (rx[1] << 8);
    spectral_data->f2 = rx[2]  | (rx[3] << 8);
    spectral_data->f3 = rx[4]  | (rx[5] << 8);
    spectral_data->f4 = rx[6]  | (rx[7] << 8);
    //adc_spectral_data->clear_0  = rx[8]  | (rx[9] << 8);
    //adc_spectral_data->nir_0    = rx[10] | (rx[11] << 8);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));


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
    spectral_data->f5    = rx[0]  | (rx[1] << 8);
    spectral_data->f6    = rx[2]  | (rx[3] << 8);
    spectral_data->f7    = rx[4]  | (rx[5] << 8);
    spectral_data->f8    = rx[6]  | (rx[7] << 8);
    spectral_data->clear = rx[8]  | (rx[9] << 8);
    spectral_data->nir   = rx[10] | (rx[11] << 8);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_as7341_get_basic_counts(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_spectral_data_t spectral_data, i2c_as7341_channels_basic_counts_data_t *basic_counts_data) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to read registers (config1) */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for get integration time failed" );

    /* determine gain sensitivity */
    float gain = i2c_as7341_get_spectral_gain_sensitivity(as7341_handle->config1_reg.bits.spectral_gain);

    /* compute integration time */
    double integration_time = 0;
    ESP_RETURN_ON_ERROR( i2c_as7341_get_integration_time(as7341_handle, &integration_time), TAG, "read integration time failed" );

    /* convert adc value to basic counts value */
    basic_counts_data->f1       = (float)spectral_data.f1 / gain * integration_time;
    basic_counts_data->f2       = (float)spectral_data.f2 / gain * integration_time;
    basic_counts_data->f3       = (float)spectral_data.f3 / gain * integration_time;
    basic_counts_data->f4       = (float)spectral_data.f4 / gain * integration_time;
    basic_counts_data->f5       = (float)spectral_data.f5 / gain * integration_time;
    basic_counts_data->f6       = (float)spectral_data.f6 / gain * integration_time;
    basic_counts_data->f7       = (float)spectral_data.f7 / gain * integration_time;
    basic_counts_data->f8       = (float)spectral_data.f8 / gain * integration_time;
    basic_counts_data->clear    = (float)spectral_data.clear / gain * integration_time;
    basic_counts_data->nir      = (float)spectral_data.nir   / gain * integration_time;

    return ESP_OK;
}

esp_err_t i2c_as7341_get_flicker_detection_status(i2c_as7341_handle_t as7341_handle, i2c_as7341_flicker_detection_states_t *state) {
    esp_err_t ret           = ESP_OK;
    uint64_t  start_time    = 0;
    bool      data_is_ready = false;

    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to disable enable register */
    ESP_RETURN_ON_ERROR( i2c_as7341_disable_enable_register(as7341_handle), TAG, "disable enable register, for get flicker detection status failed." );

    /* attempt to enable power */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_power(as7341_handle), TAG, "enable power, for get flicker detection status failed" );

    /* attempt to write SMU configuration from RAM to set SMUX chain registers */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_smux_command(as7341_handle, I2C_AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for get flicker detection status failed" );

    /* attempt to setup SMUX flicker detection */
    ESP_RETURN_ON_ERROR( i2c_as7341_setup_smux_flicker_detection(as7341_handle), TAG, "setup SMUX for flicker detection, for get flicker detection status failed" );

    /* attempt to enable SMUX */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_smux(as7341_handle), TAG, "enable SMUX, for get flicker detection status failed" );

    /* attempt to enable spectral measurement */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_spectral_measurement(as7341_handle), TAG, "enable spectral measurement, for get flicker detection status failed." );

    /* attempt to enable flicker detection */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_flicker_detection(as7341_handle), TAG, "enable flicker detection, for get flicker detection status failed" );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data, high channels, is available or timeout */
    do {
        /* attempt to check if flicker detection measurement is ready */
        ESP_GOTO_ON_ERROR( i2c_as7341_get_flicker_detection_status_register(as7341_handle), err, TAG, "read flicker detection status register, for get flicker detection status failed" );

        /* if the measurement is valid or saturation is detected, set data ready flag to true */
        if(as7341_handle->fd_status_reg.bits.fd_measurement_valid == true || as7341_handle->fd_status_reg.bits.fd_saturation_detected == true) {
            data_is_ready = true;
        }

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(5));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (500) * 1000)) // start with 500 ms
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to disable flicker detection */
    ESP_RETURN_ON_ERROR( i2c_as7341_disable_flicker_detection(as7341_handle), TAG, "disable flicker detection, for get flicker detection status failed" );

    ESP_LOGW(TAG, "FD Status Register:  0x%02x (0b%s)", as7341_handle->fd_status_reg.reg, uint8_to_binary(as7341_handle->fd_status_reg.reg));

    /* set output parameter */
    if(as7341_handle->fd_status_reg.bits.fd_saturation_detected == true) {
        *state = I2C_AS7341_FLICKER_DETECTION_SATURATED;
    } else {
        if(as7341_handle->fd_status_reg.bits.fd_measurement_valid == true &&
            as7341_handle->fd_status_reg.bits.fd_100hz_flicker_valid == false &&
            as7341_handle->fd_status_reg.bits.fd_120hz_flicker_valid == false) {
            *state = I2C_AS7341_FLICKER_DETECTION_UNKNOWN;
        } else {
            if(as7341_handle->fd_status_reg.bits.fd_100hz_flicker_valid == true) {
                *state = I2C_AS7341_FLICKER_DETECTION_100HZ;
            } else if(as7341_handle->fd_status_reg.bits.fd_120hz_flicker_valid == true) {
                *state = I2C_AS7341_FLICKER_DETECTION_120HZ;
            } else {
                *state = I2C_AS7341_FLICKER_DETECTION_INVALID;
            }
        }
    }

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

esp_err_t i2c_as7341_get_atime(i2c_as7341_handle_t as7341_handle, uint8_t *const atime) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_atime_register(as7341_handle), TAG, "read atime register for get atime failed" );

    /* set output parameter */
    *atime = as7341_handle->atime_reg;

    return ESP_OK;
}

esp_err_t i2c_as7341_set_atime(i2c_as7341_handle_t as7341_handle, const uint8_t atime) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_atime_register(as7341_handle, atime), TAG, "write atime register for set atime failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_astep(i2c_as7341_handle_t as7341_handle, uint16_t *const astep) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_astep_register(as7341_handle), TAG, "read atime register for get astep failed" );

    /* set output parameter */
    *astep = as7341_handle->astep_reg;

    return ESP_OK;
}

esp_err_t i2c_as7341_set_astep(i2c_as7341_handle_t as7341_handle, const uint16_t astep) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_astep_register(as7341_handle, astep), TAG, "write astep register for set astep failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_spectral_gain(i2c_as7341_handle_t as7341_handle, i2c_as7341_spectral_gains_t *const gain) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config1_register(as7341_handle), TAG, "read configuration 1 register for get spectral gain failed" );

    /* set output parameter */
    *gain = as7341_handle->config1_reg.bits.spectral_gain;

    return ESP_OK;
}

esp_err_t i2c_as7341_set_spectral_gain(i2c_as7341_handle_t as7341_handle, const i2c_as7341_spectral_gains_t gain) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_config1_register_t config1_reg = { .reg = as7341_handle->config1_reg.reg };

    /* set spectral gain */
    config1_reg.bits.spectral_gain = gain;

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config1_register(as7341_handle, config1_reg), TAG, "write configuration 1 register for set spectral gain failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_get_ambient_light_sensing_mode(i2c_as7341_handle_t as7341_handle, i2c_as7341_als_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_lo_register_bank(as7341_handle), TAG, "enable low register bank for get ambient light sensing mode failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( i2c_as7341_get_config_register(as7341_handle), TAG, "read configuration register for get ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_hi_register_bank(as7341_handle), TAG, "enable high register bank for get ambient light sensing mode failed" );

    /* set output parameter */
    *mode = as7341_handle->config_reg.bits.irq_mode;

    return ESP_OK;
}

esp_err_t i2c_as7341_set_ambient_light_sensing_mode(i2c_as7341_handle_t as7341_handle, const i2c_as7341_als_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_config_register_t config_reg = { .reg = as7341_handle->config_reg.reg };

    /* set mode */
    config_reg.bits.irq_mode = mode;

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_lo_register_bank(as7341_handle), TAG, "enable low register bank for set ambient light sensing mode failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config_register(as7341_handle, config_reg), TAG, "write configuration register for set ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_hi_register_bank(as7341_handle), TAG, "enable high register bank for set ambient light sensing mode failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_flicker_detection(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* enable flicker detection */
    enable_reg.bits.flicker_detection_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable flicker detection failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_flicker_detection(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* disable flicker detection */
    enable_reg.bits.flicker_detection_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for disable flicker detection failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_smux(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* enable smux */
    enable_reg.bits.smux_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable SMUX failed" );

    /* validate SMUX operation completed */
    uint16_t timeout = 1000;
    for (uint16_t time = 0; time < timeout; time++) {
        // The SMUXEN bit is cleared once the SMUX operation is finished
        ESP_RETURN_ON_ERROR( i2c_as7341_get_enable_register(as7341_handle), TAG, "read enable register for enable SMUX failed" );

        if (!as7341_handle->enable_reg.bits.smux_enabled) {
            return ESP_OK;
        }

        /* delay before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_AS7341_CMD_DELAY_MS));
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t i2c_as7341_enable_spectral_measurement(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* enable spectral measurement */
    enable_reg.bits.spectral_measurement_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_spectral_measurement(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* disable spectral measurement */
    enable_reg.bits.spectral_measurement_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_power(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* enable power */
    enable_reg.bits.power_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for enable power failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_disable_power(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy register from handle */
    i2c_as7341_enable_register_t enable_reg = { .reg = as7341_handle->enable_reg.reg };

    /* disable power */
    enable_reg.bits.power_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_enable_register(as7341_handle, enable_reg), TAG, "write enable register for disable power failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_enable_led(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy registers from handle */
    i2c_as7341_config_register_t config_reg = { .reg = as7341_handle->config_reg.reg };
    i2c_as7341_led_register_t    led_reg    = { .reg = as7341_handle->led_reg.reg };

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
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* copy registers from handle */
    i2c_as7341_config_register_t config_reg = { .reg = as7341_handle->config_reg.reg };
    i2c_as7341_led_register_t    led_reg    = { .reg = as7341_handle->led_reg.reg };

    /* enable led */
    config_reg.bits.led_ldr_control_enabled = false;
    led_reg.bits.led_ldr_enabled            = false;

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_lo_register_bank(as7341_handle), TAG, "enable low register bank for enable LED failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( i2c_as7341_set_config_register(as7341_handle, config_reg), TAG, "write configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( i2c_as7341_set_led_register(as7341_handle, led_reg), TAG, "write led register for disable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( i2c_as7341_enable_hi_register_bank(as7341_handle), TAG, "enable high register bank for enable LED failed" );

    return ESP_OK;
}

esp_err_t i2c_as7341_remove(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(as7341_handle->i2c_dev_handle);
}

esp_err_t i2c_as7341_delete(i2c_as7341_handle_t as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( as7341_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_as7341_remove(as7341_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(as7341_handle->i2c_dev_handle) {
        free(as7341_handle->i2c_dev_handle);
        free(as7341_handle);
    }

    return ESP_OK;
}