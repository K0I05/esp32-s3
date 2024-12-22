

#ifndef __NVS_EXT_H__
#define __NVS_EXT_H__

#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 Original Source Code and Author - SergeyKupavtsev
 https://github.com/SergeyKupavtsev/nvs_component
 https://github.com/SergeyKupavtsev/nvs-example

 Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com): added float and double read-write functions

*/


/**
 * @brief   Initialization NVS storage
 * @return esp_err_t result
 */
esp_err_t nvs_init(void);

/**
 * @brief 
 * 
 * @param key 
 * @param write_value 
 * @return esp_err_t 
 */
esp_err_t nvs_write_float(const char *key, float write_value);

/**
 * @brief 
 * 
 * @param key 
 * @param read_value 
 * @return esp_err_t 
 */
esp_err_t nvs_read_float(const char *key, float *read_value);

/**
 * @brief 
 * 
 * @param key 
 * @param write_value 
 * @return esp_err_t 
 */
esp_err_t nvs_write_double(const char *key, double write_value);

/**
 * @brief 
 * 
 * @param key 
 * @param read_value 
 * @return esp_err_t 
 */
esp_err_t nvs_read_double(const char *key, double *read_value);

/**
 * @brief   Write value uint8_t in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_u8(const char *key, uint8_t write_value);
/**
 * @brief   Чтение значения uint8_t in NVS
 * 
 * @param key   Storage key
 * @param value pointer to the write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_u8(const char *key, uint8_t *read_value);
/**
 * @brief   Write value uint16_t in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_u16(const char *key, uint16_t write_value);
/**
 * @brief   Чтение значения uint16_t in NVS
 * 
 * @param key   Storage key
 * @param value pointer to the write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_u16(const char *key, uint16_t *read_value);
/**
 * @brief   Write value int8_t in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_i8(const char *key, int8_t write_value);
/**
 * @brief   Чтение значения int8_t in NVS
 * 
 * @param key   Storage key
 * @param value pointer to the write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_i8(const char *key, int8_t *read_value);
/**
 * @brief   Write value int16_t in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_i16(const char *key, int16_t write_value);
/**
 * @brief   Чтение значения int16_t in NVS
 * 
 * @param key   Storage key
 * @param value pointer to the write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_i16(const char *key, int16_t *read_str);
/**
 * @brief   Write value char* in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_str(const char *key, const char *write_str);
/**
 * @brief   Чтение значения char* in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_str(const char *key, char **read_value);
/**
 * @brief   Запись структуры in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_write_struct(const char *key, void *write_struct, size_t size);
/**
 * @brief   Чтение структуры in NVS
 * 
 * @param key   Storage key
 * @param value write value
 * @return esp_err_t result
 */
esp_err_t nvs_read_struct(const char *key, void **read_struct, size_t size);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __NVS_EXT_H__

