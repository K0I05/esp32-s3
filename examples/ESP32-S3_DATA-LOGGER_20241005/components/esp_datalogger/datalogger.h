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
 * @file datalogger.h
 * @defgroup datalogger
 * @{
 *
 * ESP-IDF datalogger
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DATALOGGER_H__
#define __DATALOGGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <esp_err.h>

#include <systemtable.h>
#include <datatable.h>
#include <time_into_interval.h>

#ifdef __cplusplus
extern "C" {
#endif


/*
 * ESP DATA-LOGGER definitions
 */
#define DATALOGGER_NAME_SIZE                    (15)    //!< a maximum of 15-characters for user-defined name of data-logger handle
#define DATALOGGER_DT_HANDLES_MAXIMUM           (5)     //!< a maximum of 5 user-defined data-table handles for a data-logger handle
#define DATALOGGER_TTI_HANDLES_MAXIMUM          (5)     //!< a maximum of 5 user-defined time-into-interval handles for a data-logger handle





typedef struct {
  datatable_handle_t            datatable_handle;
  uint8_t                       datatable_fill_rate;
} datalogger_datatable_handle_t;

/**
 * @brief Data-logger state object structure definition.  Do not modify these fields once the
 * data-logger handle is created, these are read-only, and represent a state machine.
 */
struct datalogger_t {
    char                            name[DATALOGGER_NAME_SIZE];       /*!< data-logger textual name, 15-characters maximum */
    systemtable_handle_t            systemtable_handle;               /*!< data-logger system data-table handle, initialized when data-logger handle is created */
    uint8_t                         datatable_handles_count;          /*!< data-logger data-tables handles count, initialized to 0 when data-logger handle is created */
    datalogger_datatable_handle_t  *datatable_handles;              /*!< data-logger array of referenced data-table handles, initialized when data-logger handle is created */
    //uint8_t                         time_into_interval_handles_count; /*!< data-logger time-into-interval handles count, initialized to 0 when data-logger handle is created */
    //time_into_interval_handle_t    *time_into_interval_handles;       /*!< data-logger array of referenced time-into-interval handles, initialized when data-logger handle is created */
    //datalogger_event_cb_t           system_event_cb;
    datalogger_event                event_handler;
};

/**
 * @brief Data-logger structure.
 */
typedef struct datalogger_t datalogger_t;

 /**
  * @brief Data-logger handle structure.
  */
typedef struct datalogger_t *datalogger_handle_t;





/**
 * @brief Initializes a data-logger handle and only one global handle should be created.  The
 * data-logger instance monitors referenced data-table or time-into-interval handles for skipped
 * task execution events. A skipped task execution event is generated when the duration of the
 * task exceeds the time-into-interval user-defined interval. In addition, memory statistics
 * are recorded along with data-table fill status.
 * 
 * @param name Data-logger textual name, 15-character maximum.
 * @param datalogger_handle Data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_init(const char *name, datalogger_handle_t *datalogger_handle);


/**
 * @brief Creates a new data-table handle that is referenced to the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle to reference the data-table handle.
 * @param[in] datatable_config Data-logger data-table configuration.
 * @param[out] datatable_handle Data-loger data-table referenced to the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new_datatable(datalogger_handle_t datalogger_handle, const datatable_config_t *datatable_config, datatable_handle_t *datatable_handle);

/**
 * @brief Gets a count of data-table handles referenced by the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the data-table handles of interest.
 * @param[out] count Number of data-table handles created and referenced in the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_datatable_count(datalogger_handle_t datalogger_handle, uint8_t *count);

//esp_err_t datalogger_rm_datatable(datalogger_handle_t datalogger_handle, const uint8_t index);


/**
 * @brief Creates a new time-into-interval handle that is referenced to the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle to reference the time-inter-interval handle.
 * @param[in] time_into_interval_config Data-logger time-into-interval configuration.
 * @param[out] time_into_interval_handle Data-logger time-into-interval handle referenced to the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new_time_into_interval(datalogger_handle_t datalogger_handle, const time_into_interval_config_t *time_into_interval_config, time_into_interval_handle_t *time_into_interval_handle);


/**
 * @brief Gets a count of time-into-interval handles referenced by the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the time-into-interval handles of interest.
 * @param[out] count Number of time-into-interval handles created and referenced in the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_time_into_intervals_count(datalogger_handle_t datalogger_handle, uint8_t *count);

//esp_err_t datalogger_rm_time_into_interval(datalogger_handle_t datalogger_handle, const uint8_t index);



/**
 * @brief Deletes the data-logger handle and frees up resources.
 * 
 * @param datalogger_handle Data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_del(datalogger_handle_t datalogger_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DATALOGGER_H__
