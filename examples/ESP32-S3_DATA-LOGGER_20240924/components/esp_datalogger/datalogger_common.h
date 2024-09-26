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
 * @file datalogger_common.h
 * @defgroup datalogger
 * @{
 *
 * ESP-IDF datalogger
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DATALOGGER_COMMON_H__
#define __DATALOGGER_COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <esp_log.h>
#include <esp_err.h>


#ifdef __cplusplus
extern "C" {
#endif


static const char *TAG_DL_COMMON = "datalogger_common";


/**
 * @brief Data-logger time interval types enumerator.
 */
typedef enum {
    DATALOGGER_TIME_INTERVAL_SEC, /*!< Data-logger time interval in seconds. */
    DATALOGGER_TIME_INTERVAL_MIN, /*!< Data-logger time interval in minutes. */
    DATALOGGER_TIME_INTERVAL_HR   /*!< Data-logger time interval in hours. */
} datalogger_time_interval_types_t;

/**
 * @brief Data-logger time interval structure.
 */
typedef struct {
    datalogger_time_interval_types_t    type;   /*!< Data-logger time interval type (seconds, minutes, hours). */
    uint16_t                            period; /*!< Data-logger time interval period (1 to 65535), 0 is not an acceptable period. */
    uint16_t                            offset; /*!< Data-logger time interval offset (0 to 65535, 0 by default), offset cannot be larger than period. */
} datalogger_time_interval_t;

/**
 * @brief Normalizes data-logger time interval period to seconds.
 * 
 * @param[in] interval_type Data-logger time interval type.
 * @param[in] interval_period Data-logger time interval period for interval type.
 * @return uint64_t Normalized time interval period in seconds.
 */
static inline uint64_t datalogger_normalize_period_to_sec(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period) {
    uint64_t interval_sec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            interval_sec = interval_period;
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            interval_sec = (interval_period * 60);  // 1-minute has 60-seconds
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            interval_sec = (interval_period * (60 * 60)); // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_sec;
}

/**
 * @brief Normalizes data-logger time interval offset to seconds.
 * 
 * @param[in] interval_type Data-logger time interval type.
 * @param[in] interval_offset Data-logger time interval offset for interval type.
 * @return uint64_t Normalized time interval offset in seconds.
 */
static inline uint64_t datalogger_normalize_offset_to_sec(const datalogger_time_interval_types_t interval_type, const uint16_t interval_offset) {
    uint64_t interval_sec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            interval_sec = interval_offset;
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            interval_sec = (interval_offset * 60); // 1-minute has 60-seconds
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            interval_sec = ((interval_offset * 60) * 60); // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_sec;
}

/**
 * @brief Sets the next epoch event time from system clock based on the time interval type and period. 
 * 
 * The task interval should be divisible by 60 i.e. no remainder if the task interval type and 
 * period is every 10-seconds, the event will trigger on-time with the system clock i.e. 09:00:00, 09:00:10, 09:00:20, etc.
 * 
 * @param[in] interval_type Data-logger time interval type.
 * @param[in] interval_period Data-logger time interval period for interval type.
 * @param[out] epoch_time Unix epoch time of next event in seconds.
 */
static inline void datalogger_set_epoch_time_event(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, time_t *epoch_time) {
    struct timeval  now_tv;
    struct tm       now_tm;
    time_t          now_unix_time;
    struct tm       next_tm;
    uint64_t        interval_period_sec;

    /* normalize interval period to seconds */
    interval_period_sec = datalogger_normalize_period_to_sec(interval_type, interval_period);

    // get system time
    gettimeofday(&now_tv, NULL);

    // extract unix time
    now_unix_time = now_tv.tv_sec;

    // convert now tm to time-parts
    localtime_r(&now_unix_time, &now_tm);

    // initialize next tm structure time-parts based on interval-type
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = now_tm.tm_hour;
            next_tm.tm_min  = now_tm.tm_min;
            next_tm.tm_sec  = 0;
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = now_tm.tm_hour;
            next_tm.tm_min  = 0;
            next_tm.tm_sec  = 0;
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            next_tm.tm_year = now_tm.tm_year;
            next_tm.tm_mon  = now_tm.tm_mon;
            next_tm.tm_mday = now_tm.tm_mday;
            next_tm.tm_hour = 0;
            next_tm.tm_min  = 0;
            next_tm.tm_sec  = 0;
            break;
    }
    
    // validate if the next task event was computed
    if(*epoch_time != 0) {
        // add task interval to next task event epoch to compute next task event epoch
        *epoch_time = *epoch_time + interval_period_sec;

        // convert next unix time to tm components
        localtime_r(epoch_time, &next_tm);
    } else {
        // convert to unix time (seconds)
        time_t next_unix_time = mktime(&next_tm);

        // initialize next unix time by adding the task event interval
        next_unix_time = next_unix_time + interval_period_sec;

        // compute the delta between now and next unix times
        double delta_time_sec = difftime(next_unix_time, now_unix_time);

        // ensure next task event is ahead in time
        if(delta_time_sec <= 0) {
            // next task event is not ahead in time
            do {
                // keep adding task event intervals until next task event is ahead in time
                next_unix_time = next_unix_time + interval_period_sec;
                delta_time_sec = difftime(next_unix_time, now_unix_time);
            } while(delta_time_sec <= 0);
        }

        // convert next unix time to tm components
        localtime_r(&next_unix_time, &next_tm);

        // set next task event epoch time
        *epoch_time = next_unix_time;
    }

    //printf("Time Into Interval System Time:          %s", asctime(&now_tm));
    //printf("Time Into Interval Next Scan Event Time: %s", asctime(&next_tm)); 

    char ctime_str[70];

    // log time now
    strftime(ctime_str, sizeof(ctime_str), "%A %c", &now_tm);
    ESP_LOGW(TAG_DL_COMMON, "Time-Into-Interval System Time:          %s", ctime_str);

    // log time next
    strftime(ctime_str, sizeof(ctime_str), "%A %c", &next_tm);
    ESP_LOGW(TAG_DL_COMMON, "Time-Into-Interval Next Event Time:      %s", ctime_str);
}

/**
 * @brief Gets unix epoch GMT time from system clock.
 * 
 * @return time_t Unix epoch timestamp (GMT)
 */
static inline time_t datalogger_get_epoch_time(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval ts_timeval;
    gettimeofday(&ts_timeval, NULL); // TODO system clock error check
    //int err = gettimeofday(&ts_timeval, NULL);
    //assert(err == 0); // 
    // extract unix utc time for timestamp value
    return ts_timeval.tv_sec; // unix epoch timestamp
}

/**
 * @brief Converts degrees to radians.
 * 
 * @param degree degrees to convert.
 * @return double converted degrees to radians.
 */
static inline double datalogger_convert_degrees_to_radians(double degree) {
    return degree * (M_PI / 180);
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param radian radian to convert.
 * @return double converted radians to degrees.
 */
static inline double datalogger_convert_radians_to_degrees(double radian) {
    return radian * (180 / M_PI);
}









//
// https://andreyor.st/posts/2022-03-15-generic-tuples-in-c/

struct Tuple {
    float x;
    float y;
    float z;
};

/*<! axis tuple (x, y, z) */
#define tuple(...) (struct Tuple){__VA_ARGS__}

// struct Tuple axis = tuple(42.544, 34.433, 23.44);

//esp_err_t get_axis(datatable_handle_t datatable_handle, struct Tuple *axis);

enum TupleItemType {
    LONG,
    DOUBLE,
    STRING,
};

struct TupleItem {
    enum TupleItemType type;
    char               pad[4];
    union {
        long   i32;
        double f64;
        char * string;
    } value;
};

struct TupleItem make_long(long);
struct TupleItem make_double(double);
struct TupleItem make_string(char *);

/*

// move to .c file

struct TupleItem make_long(long x) {
    return (struct TupleItem){.value.i32 = x, .type = LONG};
}

struct TupleItem make_double(double x) {
    return (struct TupleItem){.value.f64 = x, .type = DOUBLE};
}

struct TupleItem make_string(char * x) {
    return (struct TupleItem){.value.string = x, .type = STRING};
}

struct Tuple {
    unsigned int       size;
    char               pad[4];
    struct TupleItem * tuple;
};
*/














#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DATALOGGER_COMMON_H__
