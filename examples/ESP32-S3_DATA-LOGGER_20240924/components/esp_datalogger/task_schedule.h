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
 * @file task_ext.h
 * @defgroup FreeRTOS task extension
 * @{
 *
 * ESP-IDF FreeRTOS task schedule
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TASK_SCHEDULE_H__
#define __TASK_SCHEDULE_H__

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
 * @brief Task schedule definition.
 */
typedef struct task_schedule_t task_schedule_t;

/**
 * @brief Task schedule handle definition.  A task schedule is used within
 * a FreeRTOS task function and delays the task based on the configured 
 * interval type and interval period that is synchronized to the system clock.
 */
typedef struct task_schedule_t *task_schedule_handle_t;

// TODO? - add multi-thread synch (i.e. mutex)

/**
 * @brief Task schedule handle parameters structure.
 */
typedef struct {
    uint64_t                         epoch_time;         /*!< task schuedule, next event unix epoch time in milli-seconds */
    datalogger_time_interval_types_t interval_type;      /*!< task schuedule, interval type setting */
    uint16_t                         interval_period;    /*!< task schuedule, a non-zero interval period setting per interval type setting  */
    uint16_t                         interval_offset;    /*!< task schuedule, interval offset setting, per interval type setting, that must be less than the interval period */
} task_schedule_params_t;

/**
 * @brief Task schedule handle state object structure.
 */
struct task_schedule_t {
    task_schedule_params_t      *params;            /*!< task schedule parameters */
};

/**
 * @brief Creates a new task schedule handle.  A task schedule is used within
 * a FreeRTOS task subroutine and delays the task based on the configured interval
 * type and interval period that is synchronized to the system clock.
 * 
 * As an example, if a 5-second interval is configured, the task will execute every
 * 5-seconds based on the system clock i.e. 12:00:00, 12:00:05, 12:00:10, etc.
 * 
 * The interval offset is used to offset the start of the interval period. As an
 * example, if a 5-minute interval with 1-minute offset is configured, the task
 * will execute every 5-minutes at 1-minute into the interval based on the system 
 * clock i.e. 12:01:00, 12:06:00, 12:11:00, etc.
 * 
 * The task execution time is compensated when the delay interval is computed.  
 * However, if the task execution time exceeds the configured interval, a skipped 
 * task will be triggerred.  
 * 
 * @param[in] interval_type Task schedule, interval type setting.
 * @param[in] interval_period Task schedule, a non-zero interval period setting per interval type setting.
 * @param[in] interval_offset Task schedule, interval offset setting, per interval type setting, that must be less than the interval period.
 * @param[out] task_schedule_handle Task schedule handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t task_schedule_new(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, task_schedule_handle_t *task_schedule_handle);

/**
 * @brief Delays the task until the next scheduled task event.  This function should
 * be placed after the `for (;;) {` syntax to delay the task based on the configured
 * interval type, period, and offset parameters.
 * 
 * @param task_schedule_handle Task schedule handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t task_schedule_delay(task_schedule_handle_t task_schedule_handle);

/**
 * @brief Deletes the task schedule handle and frees up resources.
 * 
 * @param task_schedule_handle Task schedule handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t task_schedule_del(task_schedule_handle_t task_schedule_handle);







#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TASK_SCHEDULE_H__
