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
 * @file time_into_interval.c
 *
 * ESP-IDF FreeRTOS task extension
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "time_into_interval.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TIME_INTO_INTERVAL_NAME_MAX_SIZE         (25)        //!< 25-characters for user-defined time-into-interval name

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "time_into_interval";


/**
 * @brief Gets a 16-bit hash-code utilizing epoch timestamp as the seed.
 * 
 * @return uint16_t 16-bit hash-code.
 */
static inline uint16_t time_into_interval_get_hash_code(void) {
    uint16_t seed_hash = (uint16_t)time_into_interval_get_epoch_timestamp();
    return ((seed_hash>>16) ^ (seed_hash)) & 0xFFFF;
}

uint64_t time_into_interval_normalize_interval_to_sec(const time_into_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_sec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            interval_sec = interval;
            break;
        case TIME_INTO_INTERVAL_MIN:
            interval_sec = (interval * 60U);  // 1-minute has 60-seconds
            break;
        case TIME_INTO_INTERVAL_HR:
            interval_sec = (interval * (60U * 60U)); // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_sec;
}

uint64_t time_into_interval_normalize_interval_to_msec(const time_into_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_msec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            interval_msec = interval * 1000U;
            break;
        case TIME_INTO_INTERVAL_MIN:
            interval_msec = (interval * 60U) * 1000U; // 1-minute has 60-seconds
            break;
        case TIME_INTO_INTERVAL_HR:
            interval_msec = ((interval * 60U) * 60U) * 1000U; // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_msec;
}

uint64_t time_into_interval_get_epoch_timestamp(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;
 
    // extract unix epoch utc timestamp and convert to seconds
    return (uint64_t)tv_utc_timestamp.tv_sec;
}

uint64_t time_into_interval_get_epoch_timestamp_msec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to milli-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000U + (uint64_t)tv_utc_timestamp.tv_usec / 1000U;
}

uint64_t time_into_interval_get_epoch_timestamp_usec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to micro-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000000U + (uint64_t)tv_utc_timestamp.tv_usec;
}

esp_err_t time_into_interval_set_epoch_timestamp_event(const time_into_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, uint64_t *epoch_timestamp) {
    struct timeval  now_tv;
    struct tm       now_tm;
    struct tm       next_tm;

    /* validate interval period argument */
    ESP_RETURN_ON_FALSE( (interval_period > 0), ESP_ERR_INVALID_ARG, TAG, "interval period cannot be 0, time-into-interval set epoch time event failed" );

    /* normalize interval period and offset to milli-seconds */
    uint64_t interval_period_msec = time_into_interval_normalize_interval_to_msec(interval_type, interval_period);
    uint64_t interval_offset_msec = time_into_interval_normalize_interval_to_msec(interval_type, interval_offset);

    /* validate interval period argument on total days */
    ESP_RETURN_ON_FALSE( (interval_period_msec <= (28U * 24U * 60U * 60U * 1000U)), ESP_ERR_INVALID_ARG, TAG, "interval period cannot be greater than 28-days, time-into-interval set epoch time event failed" );

    /* validate period and offset intervals */
    ESP_RETURN_ON_FALSE( ((interval_period_msec - interval_offset_msec) > 0), ESP_ERR_INVALID_ARG, TAG, "interval period must be larger than the interval offset, time-into-interval set epoch time event failed" );

    // get system unix epoch time (gmt)
    gettimeofday(&now_tv, NULL);

    // extract system unix time (seconds and milli-seconds)
    time_t now_unix_time        = now_tv.tv_sec;
    uint64_t now_unix_time_msec = (uint64_t)now_tv.tv_sec * 1000U + (uint64_t)now_tv.tv_usec / 1000U;

    // convert now tm to time-parts localtime from unix time
    localtime_r(&now_unix_time, &now_tm);

    // initialize next tm structure time-parts localtime based on interval-type
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = now_tm.tm_hour;
            next_tm.tm_min  = now_tm.tm_min;
            next_tm.tm_sec  = 0;
            break;
        case TIME_INTO_INTERVAL_MIN:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = now_tm.tm_hour;
            next_tm.tm_min  = 0;
            next_tm.tm_sec  = 0;
            break;
        case TIME_INTO_INTERVAL_HR:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = 0;
            next_tm.tm_min  = 0;
            next_tm.tm_sec  = 0;
            break;
    }

    /* handle interval period by tm structure time-part timespan exceedance */
    if(interval_period_msec > (60U * 1000U)) {
        /* over 60-seconds, set minute time-part to 0 */
        next_tm.tm_min  = 0;
        next_tm.tm_sec  = 0;
    } else if(interval_period_msec > (60U * 60U * 1000U)) {
        /* over 60-minutes, set hour time-part to 0 */
        next_tm.tm_hour = 0;
        next_tm.tm_min  = 0;
        next_tm.tm_sec  = 0;
    } else if(interval_period_msec > (24U * 60U * 60U * 1000U)) {
        /* over 24-hours, set day time-part to 0 */
        next_tm.tm_mday = 0;
        next_tm.tm_hour = 0;
        next_tm.tm_min  = 0;
        next_tm.tm_sec  = 0;
    }
    
    // validate if the next task event was computed
    if(*epoch_timestamp != 0) {
        // add task interval to next task event epoch to compute next task event epoch
        *epoch_timestamp = *epoch_timestamp + interval_period_msec;
    } else {
        // convert to unix time (seconds)
        time_t next_unix_time = mktime(&next_tm);

        // convert unix time to milli-seconds
        uint64_t next_unix_time_msec = next_unix_time * 1000U;

        // initialize next unix time by adding the task event interval period and offset
        next_unix_time_msec = next_unix_time_msec + interval_period_msec + interval_offset_msec;

        // compute the delta between now and next unix times
        int64_t delta_time_msec = next_unix_time_msec - now_unix_time_msec;

        // ensure next task event is ahead in time
        if(delta_time_msec <= 0) {
            // next task event is not ahead in time
            do {
                // keep adding task event intervals until next task event is ahead in time
                next_unix_time_msec = next_unix_time_msec + interval_period_msec;
                
                // compute the delta between now and next unix times
                delta_time_msec = next_unix_time_msec - now_unix_time_msec;
            } while(delta_time_msec <= 0);
        }

        // set next task event epoch time
        *epoch_timestamp = next_unix_time_msec;
    }

    return ESP_OK;
}

esp_err_t time_into_interval_init(const time_into_interval_config_t *time_into_interval_config, 
                                 time_into_interval_handle_t *time_into_interval_handle) {
    esp_err_t                   ret = ESP_OK;
    time_into_interval_handle_t out_handle;
    
    /* validate task-schedule arguments */
    ESP_GOTO_ON_FALSE( (strlen(time_into_interval_config->name) <= TIME_INTO_INTERVAL_NAME_MAX_SIZE), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval name cannot exceed 20-characters, time-into-interval handle initialization failed" );
    ESP_GOTO_ON_FALSE( (time_into_interval_config->interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period cannot be 0, time-into-interval handle initialization failed" );

    /* validate period and offset intervals */
    int64_t interval_delta = time_into_interval_normalize_interval_to_sec(time_into_interval_config->interval_type, time_into_interval_config->interval_period) - 
                             time_into_interval_normalize_interval_to_sec(time_into_interval_config->interval_type, time_into_interval_config->interval_offset); 
    ESP_GOTO_ON_FALSE( (interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period must be larger than the interval offset, time-into-interval handle initialization failed" );
    
    /* validate memory availability for time into interval handle */
    out_handle = (time_into_interval_handle_t)calloc(1, sizeof(time_into_interval_t)); 
    ESP_GOTO_ON_FALSE( out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for time-into-interval handle, time-into-interval handle initialization failed" );

    /* initialize task schedule state object parameters */
    out_handle->name            = time_into_interval_config->name;
    out_handle->epoch_timestamp = 0;
    out_handle->interval_type   = time_into_interval_config->interval_type;
    out_handle->interval_period = time_into_interval_config->interval_period;
    out_handle->interval_offset = time_into_interval_config->interval_offset;
    out_handle->hash_code       = time_into_interval_get_hash_code();
    out_handle->mutex_handle    = xSemaphoreCreateMutex();

    /* set epoch timestamp of the next scheduled time-into-interval event */
    ESP_GOTO_ON_ERROR( time_into_interval_set_epoch_timestamp_event(out_handle->interval_type, 
                                                            out_handle->interval_period, 
                                                            out_handle->interval_offset, 
                                                            &out_handle->epoch_timestamp), 
                                                            err_out_handle, TAG, "unable to set epoch timestamp, time-into-interval handle initialization failed" );

    /* set output handle */
    *time_into_interval_handle = out_handle;

    return ESP_OK;

    err_out_handle:
        free(out_handle);
    err:
        return ret;
}

bool time_into_interval(time_into_interval_handle_t time_into_interval_handle) {
    bool state = false;

    /* validate arguments */
    if(!(time_into_interval_handle)) {
        return state;
    }

    /* lock the mutex */
    xSemaphoreTake(time_into_interval_handle->mutex_handle, portMAX_DELAY);

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = time_into_interval_get_epoch_timestamp_msec();

    // compute time delta until next time into interval condition
    int64_t delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;

    // validate time delta, when delta is <= 0, time has elapsed
    if(delta_msec <= 0) {
        // set time-into-interval state to true - intervale has lapsed
        state = true;

        /* set next event timestamp (UTC) */
        time_into_interval_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                                    time_into_interval_handle->interval_period, 
                                                    time_into_interval_handle->interval_offset, 
                                                    &time_into_interval_handle->epoch_timestamp);
    }

    /* unlock the mutex */
    xSemaphoreGive(time_into_interval_handle->mutex_handle);
    
    return state;
}

esp_err_t time_into_interval_delay(time_into_interval_handle_t time_into_interval_handle) {
    // validate arguments
    ESP_ARG_CHECK( time_into_interval_handle );

    /* lock the mutex */
    xSemaphoreTake(time_into_interval_handle->mutex_handle, portMAX_DELAY);

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = time_into_interval_get_epoch_timestamp_msec();

    // compute time delta until next scan event
    int64_t delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;

    // validate time is into the future, otherwise, reset next epoch time
    if(delta_msec < 0) {
        // reset epoch time of the next schedule task
        time_into_interval_handle->epoch_timestamp = 0;

        // set epoch timestamp of the next scheduled task
        time_into_interval_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                                    time_into_interval_handle->interval_period, 
                                                    time_into_interval_handle->interval_offset, 
                                                    &time_into_interval_handle->epoch_timestamp);

        // compute time delta for next event
        delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;
    }

    // compute ticks delay from time delta
    TickType_t delay = (delta_msec / portTICK_PERIOD_MS);

    /* unlock the mutex */
    xSemaphoreGive(time_into_interval_handle->mutex_handle);

    // delay the task per ticks delay
    vTaskDelay( delay );

    /* lock the mutex */
    xSemaphoreTake(time_into_interval_handle->mutex_handle, portMAX_DELAY);

    // set epoch timestamp of the next scheduled task
    time_into_interval_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                        time_into_interval_handle->interval_period, 
                                        time_into_interval_handle->interval_offset, 
                                        &time_into_interval_handle->epoch_timestamp);
                                        
    /* unlock the mutex */
    xSemaphoreGive(time_into_interval_handle->mutex_handle);

    return ESP_OK;
}

esp_err_t time_into_interval_get_last_event(time_into_interval_handle_t time_into_interval_handle, uint64_t *epoch_timestamp) {
    // validate arguments
    ESP_ARG_CHECK( time_into_interval_handle );

    /* lock the mutex */
    xSemaphoreTake(time_into_interval_handle->mutex_handle, portMAX_DELAY);

    /* convert interval into msec */
    uint64_t interval_msec = time_into_interval_normalize_interval_to_msec(time_into_interval_handle->interval_type, time_into_interval_handle->interval_period);

    /* set last event epoch timestamp */
    *epoch_timestamp = time_into_interval_handle->epoch_timestamp - interval_msec;

    /* unlock the mutex */
    xSemaphoreGive(time_into_interval_handle->mutex_handle);

    return ESP_OK;
}

esp_err_t time_into_interval_delete(time_into_interval_handle_t time_into_interval_handle) {
    /* free resource */
    if(time_into_interval_handle) {
        free(time_into_interval_handle);
    }

    return ESP_OK;
}