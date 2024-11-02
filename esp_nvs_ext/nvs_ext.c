#include "nvs_ext.h"
#include <string.h>

#define FLOAT_MAX_STRING_LENGTH 32  // Maximum length of the float to string representation
#define DOUBLE_MAX_STRING_LENGTH 64  // Maximum length of the double to string representation
#define NAMESPACE "storage"

#define DEBUGGING_NVS 1
#if DEBUGGING_NVS == 0
#define DEBUG_NVS(format, ...)
#else
#define DEBUG_NVS(message, ...) printf(message, ##__VA_ARGS__)
#endif

//static const char *TAG = "nvs_ext";

esp_err_t nvs_write_float(const char *key, float write_value) {
    nvs_handle_t handle;
    esp_err_t err;
    char *data = malloc(FLOAT_MAX_STRING_LENGTH);
    if (data == NULL) {
        DEBUG_NVS("[NVS] Error allocating memory!\n");
        return ESP_ERR_NO_MEM;
    }
    int32_t result = snprintf(data, FLOAT_MAX_STRING_LENGTH, "%f", write_value);
    if (result >= 0 && result <= strlen(data)) {
        err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
        DEBUG_NVS("[NVS] Save %s = %s \n", key, data);
        err = nvs_set_str(handle, key, data);
        err = nvs_commit(handle);
        nvs_close(handle);
    } else {
        err = ESP_FAIL;
    }
    free(data);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %f  Failed!\n" : "[NVS] Write %s = %f Done\n", key, write_value);
    return err;
}

esp_err_t nvs_read_float(const char *key, float *read_value) {
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    size_t required_size;

    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) getting required size!\n", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    char *data = malloc(required_size);
    if (data == NULL) {
        DEBUG_NVS("[NVS] Error allocating memory!\n");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_str(handle, key, data, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
        free(data);
        nvs_close(handle);
        return err;
    }

    *read_value = strtof(data, NULL);
    DEBUG_NVS("[NVS] Read %s = %f\n", key, *read_value);

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_write_double(const char *key, double write_value) {
    nvs_handle_t handle;
    esp_err_t err;
    char *data = malloc(DOUBLE_MAX_STRING_LENGTH);
    if (data == NULL) {
        DEBUG_NVS("[NVS] Error allocating memory!\n");
        return ESP_ERR_NO_MEM;
    }
    int32_t result = snprintf(data, DOUBLE_MAX_STRING_LENGTH, "%lf", write_value);
    if (result >= 0 && result <= strlen(data)) {
        err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
        DEBUG_NVS("[NVS] Save %s = %s \n", key, data);
        err = nvs_set_str(handle, key, data);
        err = nvs_commit(handle);
        nvs_close(handle);
    } else {
        err = ESP_FAIL;
    }
    free(data);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %f  Failed!\n" : "[NVS] Write %s = %f Done\n", key, write_value);
    return err;
}

esp_err_t nvs_read_double(const char *key, double *read_value) {
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    size_t required_size;

    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) getting required size!\n", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    char *data = malloc(required_size);
    if (data == NULL) {
        DEBUG_NVS("[NVS] Error allocating memory!\n");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_str(handle, key, data, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
        free(data);
        nvs_close(handle);
        return err;
    }

    *read_value = strtod(data, NULL);
    DEBUG_NVS("[NVS] Read %s = %lf\n", key, *read_value);

    nvs_close(handle);
    return ESP_OK;
}


esp_err_t nvs_write_str(const char *key, const char *write_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    DEBUG_NVS("[NVS] Save %s = %s \n", key, write_value);
    err = nvs_set_str(handle, key, write_value);
    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %s  Failed!\n" : "[NVS] Write %s = %s Done\n", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_str(const char *key, char **read_value)
{
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    size_t required_size;

    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) getting required size!\n", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    char *data = malloc(required_size);
    if (data == NULL) {
        DEBUG_NVS("[NVS] Error allocating memory!\n");
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_str(handle, key, data, &required_size);
    if (err != ESP_OK) {
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
        free(data);
        nvs_close(handle);
        return err;
    }

    *read_value = data;
    DEBUG_NVS("[NVS] Read %s = %s\n", key, *read_value);

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_write_u8(const char *key, uint8_t write_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u8(handle, key, write_value);

    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %i Failed!\n" : "[NVS] Write %s = %i Done\n", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u8(const char *key, uint8_t *read_value)
{
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u8(handle, key, read_value);
    switch (err)
    {
    case ESP_OK:
        DEBUG_NVS("[NVS] Read %s = %d\n", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        DEBUG_NVS("[NVS] The value %s is not initialized yet!\n", key);
        break;
    default:
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_u16(const char *key, uint16_t write_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_u16(handle, key, write_value);

    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %i Failed!\n" : "[NVS] Write %s = %i Done\n", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_u16(const char *key, uint16_t *read_value)
{
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_u16(handle, key, read_value);
    switch (err)
    {
    case ESP_OK:
        DEBUG_NVS("[NVS] Read %s = %d\n", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        DEBUG_NVS("[NVS] The value %s is not initialized yet!\n", key);
        break;
    default:
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i8(const char *key, int8_t write_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i8(handle, key, write_value);

    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %i Failed!\n" : "[NVS] Write %s = %i Done\n", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i8(const char *key, int8_t *read_value)
{
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i8(handle, key, read_value);
    switch (err)
    {
    case ESP_OK:
        DEBUG_NVS("[NVS] Read %s = %d\n", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        DEBUG_NVS("[NVS] The value %s is not initialized yet!\n", key);
        break;
    default:
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_i16(const char *key, int16_t write_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_i16(handle, key, write_value);
    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s = %i Failed!\n" : "[NVS] Write %s = %i Done\n", key, write_value);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_i16(const char *key, int16_t *read_value)
{
    esp_err_t err;
    nvs_handle_t handle;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_i16(handle, key, read_value);
    switch (err)
    {
    case ESP_OK:
        DEBUG_NVS("[NVS] Read %s = %d\n", key, *read_value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        DEBUG_NVS("[NVS] The value %s is not initialized yet!\n", key);
        break;
    default:
        DEBUG_NVS("[NVS] Error (%s) reading!\n", esp_err_to_name(err));
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_write_struct(const char *key, void *write_struct, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;
    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_set_blob(handle, key, write_struct, size);
    err = nvs_commit(handle);
    DEBUG_NVS((err != ESP_OK) ? "[NVS] Write %s struct = Failed!\n" : "[NVS] Write %s struct  = Done\n", key);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_read_struct(const char *key, void **read_struct, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    err = nvs_get_blob(handle, key, *read_struct, &size);

    DEBUG_NVS((err != ESP_OK) ? "[NVS] Read %s struct = Failed!\n" : "[NVS] Read %s struct  = Done\n", key);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    return ret;
}
