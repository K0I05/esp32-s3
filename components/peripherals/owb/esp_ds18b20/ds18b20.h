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
 * @file ds18b20.h
 * @defgroup drivers ds18b20
 * @{
 *
 * ESP-IDF driver for ds18b20 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <onewire_device.h>


#ifdef __cplusplus
extern "C"
{
#endif

/*
 * DS18B20 definitions
 */


/*
 * DS18B20 macro definitions
 */

#define OWB_DS18B20_CONFIG_DEFAULT { .resolution = OWB_DS18B20_RESOLUTION_10B }



/**
 * @brief DS18B20 supported resolutions enumerator.
 */
typedef enum {
    OWB_DS18B20_RESOLUTION_9B = 0,  /*!<  9bit, needs ~93.75ms convert time (Tconv/8) */
    OWB_DS18B20_RESOLUTION_10B,     /*!< 10bit, needs ~187.5ms convert time (Tconv/4) */
    OWB_DS18B20_RESOLUTION_11B,     /*!< 11bit, needs ~375ms convert time (Tconv/2) */
    OWB_DS18B20_RESOLUTION_12B,     /*!< 12bit, needs ~750ms convert time (Tconv) */
} owb_ds18b20_resolutions_t;

/**
 * @brief DS18B20 OWB scratchpad structure.
 */
typedef struct __attribute__((packed)) {
    uint8_t temp_lsb;      /*!< lsb of temperature */
    uint8_t temp_msb;      /*!< msb of temperature */
    uint8_t th_user1;      /*!< th register or user byte 1 */
    uint8_t tl_user2;      /*!< tl register or user byte 2 */
    uint8_t configuration; /*!< resolution configuration register */
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t crc_value;     /*!< crc value of scratchpad data */
} owb_ds18b20_scratchpad_t;

/**
 * @brief DS18B20 OWB device configuration structure.
 */
typedef struct {
    owb_ds18b20_resolutions_t resolution;
} owb_ds18b20_config_t;

/**
 * @brief DS18B20 OWB device structure.
 */
struct owb_ds18b20_t {
    onewire_bus_handle_t owb_bus_handle;
    onewire_device_address_t owb_dev_address;
    uint8_t th_user1;
    uint8_t tl_user2;
    owb_ds18b20_resolutions_t resolution;
};

/**
 * @brief DS18B20 OWB device structure definition.
 */
typedef struct owb_ds18b20_t owb_ds18b20_t;

/**
 * @brief DS18B20 OWB device handle definition.
 */
typedef struct owb_ds18b20_t *owb_ds18b20_handle_t;



/**
 * @brief Initializes an DS18B20 device onto the one wire master bus.
 *
 * @param[in] device One wire device.
 * @param[in] ds18b20_config Configuration of DS18B20 device.
 * @param[out] ds18b20_handle DS18B20 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_init(onewire_device_t *device, const owb_ds18b20_config_t *ds18b20_config, owb_ds18b20_handle_t *ds18b20_handle);

/**
 * @brief Triggers temperature conversion and reads temperature from DS18B20.
 * 
 * @param ds18b20_handle DS18B20 device handle.
 * @param temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_get_measurement(owb_ds18b20_handle_t ds18b20_handle, float *const temperature);

/**
 * @brief Reads temperature from DS18B20.
 * 
 * @note The trigger temperature conversion function must be called first.
 *
 * @param ds18b20_handle DS18B20 device handle.
 * @param temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_get_temperature(owb_ds18b20_handle_t ds18b20_handle, float *const temperature);

/**
 * @brief Trigger temperature conversion of DS18B20.
 * 
 * @param ds18b20_handle DS18B20 device handle. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_trigger_temperature_conversion(owb_ds18b20_handle_t ds18b20_handle);

/**
 * @brief Writes temperature conversion resolution to DS18B20.
 * 
 * @param ds18b20_handle DS18B20 device handle. 
 * @param resolution DS18B20 temperature conversion resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_set_resolution(owb_ds18b20_handle_t ds18b20_handle, const owb_ds18b20_resolutions_t resolution);

/**
 * @brief Removes an DS18B20 device from master bus and frees handle.
 * 
 * @param ds18b20_handle DS18B20 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t owb_ds18b20_delete(owb_ds18b20_handle_t ds18b20_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __DS18B20_H__
