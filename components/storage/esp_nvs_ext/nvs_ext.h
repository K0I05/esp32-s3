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
/*
 Original Source Code and Author - SergeyKupavtsev
 https://github.com/SergeyKupavtsev/nvs_component
 https://github.com/SergeyKupavtsev/nvs-example
*/
/**
 * @file nvs_ext.h
 * @defgroup drivers nvs_ext
 * @{
 *
 * ESP-IDF nvs driver extension
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __NVS_EXT_H__
#define __NVS_EXT_H__

#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Initialization NVS storage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_init(void);
/**
 * @brief Writes float typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_float(const char *key, float write_value);
/**
 * @brief Reads float typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_float(const char *key, float *read_value);
/**
 * @brief Writes double typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_double(const char *key, double write_value);
/**
 * @brief Reads double typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_double(const char *key, double *read_value);
/**
 * @brief Writes uint8_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_u8(const char *key, uint8_t write_value);
/**
 * @brief Reads uint8_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_u8(const char *key, uint8_t *read_value);
/**
 * @brief Writes uint16_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_u16(const char *key, uint16_t write_value);
/**
 * @brief Reads uint16_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_u16(const char *key, uint16_t *read_value);
/**
 * @brief Writes uint32_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_u32(const char *key, uint32_t write_value);
/**
 * @brief Reads uint32_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_u32(const char *key, uint32_t *read_value);
/**
 * @brief Writes uint64_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_u64(const char *key, uint64_t write_value);
/**
 * @brief Reads uint64_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_u64(const char *key, uint64_t *read_value);
/**
 * @brief Writes int8_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_i8(const char *key, int8_t write_value);
/**
 * @brief Reads int8_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_i8(const char *key, int8_t *read_value);
/**
 * @brief Writes int16_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_i16(const char *key, int16_t write_value);
/**
 * @brief Reads int16_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_i16(const char *key, int16_t *read_value);
/**
 * @brief Writes int32_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_i32(const char *key, int32_t write_value);
/**
 * @brief Reads int32_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_i32(const char *key, int32_t *read_value);
/**
 * @brief Writes int64_t typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_value Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_i64(const char *key, int64_t write_value);
/**
 * @brief Reads int64_t typed value from NVS.
 * 
 * @param key Storage key.
 * @param read_value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_i64(const char *key, int64_t *read_value);
/**
 * @brief Writes string typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_str Write value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_str(const char *key, const char *write_str);
/**
 * @brief Reads string typed value from NVS.
 * 
 * @param key Storage key.
 * @param value Pointer to the read value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_str(const char *key, char **read_str);
/**
 * @brief Writes struct typed value to NVS.
 * 
 * @param key Storage key.
 * @param write_struct Write struct.
 * @param size Size of struct.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_write_struct(const char *key, void *write_struct, size_t size);
/**
 * @brief Reads struct type value from NVS.
 * 
 * @param key Storage key.
 * @param read_struct Pointer to the struct value.
 * @param size Size of struct.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_read_struct(const char *key, void **read_struct, size_t size);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __NVS_EXT_H__

