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
 * @file time_into_interval.h
 * @defgroup FreeRTOS task extension
 * @{
 *
 * ESP-IDF FreeRTOS task extension
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TIME_INTO_INTERVAL_H__
#define __TIME_INTO_INTERVAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <esp_err.h>
#include <datalogger_common.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Time into interval definition.
 */
typedef struct time_into_interval_t time_into_interval_t;

/**
 * @brief Time into interval handle definition.
 */
typedef struct time_into_interval_t *time_into_interval_handle_t;

/**
 * @brief Time into interval handle parameters structure.
 */
typedef struct {
    time_t                           epoch_time;         /*!< time into interval unix epoch time in seconds */
    datalogger_time_interval_types_t interval_type;      /*!< time into interval, interval type setting */
    uint16_t                         interval_period;    /*!< time into interval, a non-zero interval period setting */
    uint16_t                         interval_offset;    /*!< time into interval, interval period setting */
} time_into_interval_params_t;

/**
 * @brief Time into interval handle state object structure.
 */
struct time_into_interval_t {
    time_into_interval_params_t *params;            /*!< time into interval parameters */
};

/**
 * @brief Creates a new time into interval handle.  A time into interval is used 
 * within a FreeRTOS task subroutine and returns true based on the configured
 * interval type and interval period that is synchronized to the system clock.
 * 
 * As an example, if a 5-second interval is configured, the will return true
 * every 5-seconds based on the system clock i.e. 12:00:00, 12:00:05, 12:00:10, etc.
 * 
 * The interval offset is used to offset the start of the interval period. As an
 * example, if a 5-minute interval with 1-minute offset is configured, the function
 * will return true every 5-minutes at 1-minute into the interval based on the 
 * system clock i.e. 12:01:00, 12:06:00, 12:11:00, etc. 
 * 
 * @param interval_type Time into interval, interval type setting.
 * @param interval_period Time into interval, interval period setting per configured interval type.
 * @param interval_offset Time into interval, interval offset setting per configured interval type.
 * @param time_into_interval_handle Time into interval handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t time_into_interval_new(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, time_into_interval_handle_t *time_into_interval_handle);

/**
 * @brief Validates when time into interval condition based on the configured interval
 * type and interval period that is synchronized to the system clock.
 * 
 * @param[in] time_into_interval_handle Time into interval handle.
 * @return true when time into interval condition is valid.
 * @return false when time into interval condition is not valid.
 */
bool time_into_interval(time_into_interval_handle_t time_into_interval_handle);

/**
 * @brief Deletes the time into interval handle and frees up resources.
 * 
 * @param time_into_interval_handle Time into interval handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t time_into_interval_del(time_into_interval_handle_t time_into_interval_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TIME_INTO_INTERVAL_H__