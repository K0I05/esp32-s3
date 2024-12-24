# NVS Extension
The NVS extension simplifies the process of reading and writing information to non-volatile storage.  The NVS extension supports common data-types such as `uint8_t` to `uint64_t`, `int8_t` to `int64_t`, `float` and `double`, `string`, and `struct`.  The library can be used as is but you can implement wrappers if the information will be stored to a common key to reference the key-value pairs.  

A wrapper example for a `struct` data-type is presented below.  The example uses a `struct` to preserve system state information that is updated on system boot-up.
```
/* System state structure */
typedef struct system_state_tag {
    uint16_t    reboot_counter;         /*!< number of times system has restarted */
    uint64_t    reboot_timestamp;       /*!< system restart unix epoch timestamp (UTC) in seconds */
    uint64_t    system_uptime;          /*!< up-time in seconds since system restart */
} system_state_t;

/* global variables */
static system_state_t  *s_system_state = NULL;
```
The `struct` wrapper functions are declared below using a common key named `system_state`.
```
static inline esp_err_t nvs_write_system_state(system_state_t *system_state) {
    return nvs_write_struct("system_state", system_state, sizeof(system_state_t));
}

static inline esp_err_t nvs_read_system_state(system_state_t **system_state) {
    return nvs_read_struct("system_state", (void **)system_state, sizeof(system_state_t));
}
```
The system state `struct` is initialized and saved to NVS on system boot-up as shown below. 
```
static inline void init_system_state(void) {
    s_system_state = (system_state_t*)calloc(1, sizeof(system_state_t));

    /* attempt to read system state structure from nvs */
    if(nvs_read_system_state(&s_system_state) != ESP_OK) {
        /* assume system state structure doesn't exist - initialize attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp();
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    } else {
        /* assume system state structure exists - update attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp() - s_system_state->reboot_timestamp;
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    }

    /* attempt to write system state structure to nvs */
    nvs_write_system_state(s_system_state);

    /* print system state structure */
    ESP_LOGW(TAG, "Boot Count: %u", s_system_state->reboot_counter);
    ESP_LOGW(TAG, "Up-Time:    %llu", s_system_state->system_uptime);
    ESP_LOGW(TAG, "Timestamp:  %llu", s_system_state->reboot_timestamp);
}
```
If any problems arise please feel free to log an issue and if you would to contribute please contact me.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)