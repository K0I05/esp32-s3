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
 * @file nvs_ext.c
 *
 * ESP-IDF nvs driver extension
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "nvs_ext.h"
#include <string.h>
#include <esp_log.h>
#include <esp_check.h>

/*
 * NVS EXT definitions
*/
#define NVS_EXT_FLOAT_MAX_STRING_LENGTH     (32)    // Maximum length of the float to string representation
#define NVS_EXT_DOUBLE_MAX_STRING_LENGTH    (64)    // Maximum length of the double to string representation
#define NVS_EXT_NAMESPACE                   "nvs_ext_data"

/*
* static constant declerations
*/
static const char *TAG = "nvs_ext";


esp_err_t nvs_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "unable to erase flash, nvs init failed");
        ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "unable to initialize flash, nvs init failed");
    } else {
        ESP_RETURN_ON_ERROR(ret, TAG, "unable to initialize flash, nvs init failed");
    }
    return ESP_OK;
}

esp_err_t nvs_write_float(const char *key, float write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    char *data = malloc(NVS_EXT_FLOAT_MAX_STRING_LENGTH);
    if (data == NULL) {
        ESP_LOGD(TAG, "Error allocating memory!");
        return ESP_ERR_NO_MEM;
    }
    int32_t result = snprintf(data, NVS_EXT_FLOAT_MAX_STRING_LENGTH, "%f", write_value);
    if (result >= 0 && result <= strlen(data)) {
        err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
        ESP_LOGD(TAG, "Save %s = %s", key, data);
        err = nvs_set_str(handle, key, data);
        err = nvs_commit(handle);
        nvs_close(handle);
    } else {
        err = ESP_FAIL;
    }
    free(data);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %f Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %f Done", key, write_value);
    return err;
}

esp_err_t nvs_read_float(const char *key, float *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    size_t required_size;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Error (%s) getting required size!", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    char *data = malloc(required_size);
    if (data == NULL) {
        ESP_LOGD(TAG, "Error allocating memory!");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }
    err = nvs_get_str(handle, key, data, &required_size);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %s", key, data);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        free(data);
        nvs_close(handle);
        return err;
    }
    *read_value = strtof(data, NULL);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_double(const char *key, double write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    char *data = malloc(NVS_EXT_DOUBLE_MAX_STRING_LENGTH);
    if (data == NULL) {
        ESP_LOGD(TAG, "Error allocating memory!");
        return ESP_ERR_NO_MEM;
    }
    int32_t result = snprintf(data, NVS_EXT_DOUBLE_MAX_STRING_LENGTH, "%lf", write_value);
    if (result >= 0 && result <= strlen(data)) {
        err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
        ESP_LOGD(TAG, "Save %s = %s", key, data);
        err = nvs_set_str(handle, key, data);
        err = nvs_commit(handle);
        nvs_close(handle);
    } else {
        err = ESP_FAIL;
    }
    free(data);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %f Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %f Done", key, write_value);
    return err;
}

esp_err_t nvs_read_double(const char *key, double *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    size_t required_size;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Error (%s) getting required size!", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    char *data = malloc(required_size);
    if (data == NULL) {
        ESP_LOGD(TAG, "Error allocating memory!");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }
    err = nvs_get_str(handle, key, data, &required_size);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %s", key, data);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        free(data);
        nvs_close(handle);
        return err;
    }
    *read_value = strtod(data, NULL);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_str(const char *key, const char *write_str) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    ESP_LOGD(TAG, "Save %s = %s", key, write_str);
    err = nvs_set_str(handle, key, write_str);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %s Failed!", key, write_str);
    else
        ESP_LOGD(TAG, "Write %s = %s Done", key, write_str);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_str(const char *key, char **read_str) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    size_t required_size;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Error (%s) getting required size!", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    char *data = malloc(required_size);
    if (data == NULL) {
        ESP_LOGD(TAG, "Error allocating memory!");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }
    err = nvs_get_str(handle, key, data, &required_size);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %s", key, *read_str);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        free(data);
        nvs_close(handle);
        return err;
    }
    *read_str = data;
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_u8(const char *key, uint8_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u8(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %u Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %u Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u8(const char *key, uint8_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u8(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %u", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_u16(const char *key, uint16_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u16(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %u Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %u Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u16(const char *key, uint16_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u16(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %u", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_u32(const char *key, uint32_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u32(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %lu Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %lu Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u32(const char *key, uint32_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u32(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %lu", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_u64(const char *key, uint64_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u64(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %llu Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %llu Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u64(const char *key, uint64_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u64(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %llu", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i8(const char *key, int8_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i8(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %i Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %i Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i8(const char *key, int8_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i8(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %i", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i16(const char *key, int16_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i16(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %i Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %i Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i16(const char *key, int16_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i16(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %i", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i32(const char *key, int32_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i32(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %li Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %li Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i32(const char *key, int32_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i32(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %li", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i64(const char *key, int64_t write_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i64(handle, key, write_value);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s = %lli Failed!", key, write_value);
    else
        ESP_LOGD(TAG, "Write %s = %lli Done", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i64(const char *key, int64_t *read_value) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i64(handle, key, read_value);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s = %lli", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_struct(const char *key, void *write_struct, size_t size) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_blob(handle, key, write_struct, size);
    err = nvs_commit(handle);
    if(err != ESP_OK) 
        ESP_LOGD(TAG, "Write %s Failed!", key);
    else
        ESP_LOGD(TAG, "Write %s Done", key);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_struct(const char *key, void **read_struct, size_t size) {
    esp_err_t err = ESP_OK;
    nvs_handle_t handle;
    err = nvs_open(NVS_EXT_NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_blob(handle, key, *read_struct, &size);
    switch (err) {
    case ESP_OK:
        ESP_LOGD(TAG, "Read %s", key);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGD(TAG, "The value %s is not initialized yet!", key);
        break;
    default:
        ESP_LOGD(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

