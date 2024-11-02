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
 * @file common.h
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
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>


#ifdef __cplusplus
extern "C" {
#endif

/*
 * ESP DATA-LOGGER COMMON macro definitions
 */

#define SEC_TO_USEC(sec)    { return (1000000U * sec); }						/*!< converts seconds to micro-seconds */
#define SEC_TO_MSEC(sec)    { return (1000U * sec); }							/*!< converts seconds to milli-seconds */
#define SEC_TO_TICKS(sec)   { return (SEC_TO_MS(sec) / portTICK_PERIOD_MS); }	/*!< converts seconds to ticks */
#define MSEC_TO_TICKS(msec) { return (msec / portTICK_PERIOD_MS); }				/*!< converts milli-seconds to ticks */

/*
* static constant declerations
*/
static const char *TAG_DL_COMMON = "datalogger_common";


/**
 * @brief Data-logger event types enumerator.
 */
typedef enum datalogger_event_types_tag {
    DATALOGGER_EVENT_DL_INIT,
    DATALOGGER_EVENT_DT_INIT,
    DATALOGGER_EVENT_DT_RESET,
    DATALOGGER_EVENT_DT_FIFO,
    DATALOGGER_EVENT_DT_FULL,
    DATALOGGER_EVENT_DT_ROW,
    DATALOGGER_EVENT_DT_COLUMN,
    DATALOGGER_EVENT_DT_PRCS,  // process samples
    DATALOGGER_EVENT_TII_INIT,
    DATALOGGER_EVENT_TII_RESET,
    DATALOGGER_EVENT_TII_ELAPSED,
    DATALOGGER_EVENT_TII_SKIPPED,
} datalogger_event_types_t;

/**
 * @brief Data-logger event sources enumerator.
 */
typedef enum datalogger_event_sources_tag {
    DATALOGGER_EVENT_DL,
    DATALOGGER_EVENT_DT,
    DATALOGGER_EVENT_TII
} datalogger_event_sources_t;


typedef struct datalogger_event_tag {
    datalogger_event_sources_t  source;
    datalogger_event_types_t    type;
    const char*                 message;
} datalogger_event_t;


/**
 * @brief Data-logger event.
 * 
 */
typedef void (*datalogger_event)(void *handle, datalogger_event_t);




// https://lloydrochester.com/post/c/c-timestamp-epoch/

/**
 * @brief Data-logger time interval types enumerator.
 */
typedef enum datalogger_time_interval_types_tag {
    DATALOGGER_TIME_INTERVAL_SEC, /*!< Data-logger time interval in seconds. */
    DATALOGGER_TIME_INTERVAL_MIN, /*!< Data-logger time interval in minutes. */
    DATALOGGER_TIME_INTERVAL_HR   /*!< Data-logger time interval in hours. */
} datalogger_time_interval_types_t;

/**
 * @brief Data-logger time interval structure.
 */
typedef struct datalogger_time_interval_tag {
    datalogger_time_interval_types_t    type;   /*!< Data-logger time interval type (seconds, minutes, hours). */
    uint16_t                            period; /*!< Data-logger time interval period (1 to 65535), 0 is not an acceptable period. */
    uint16_t                            offset; /*!< Data-logger time interval offset (0 to 65535, 0 by default), offset cannot be larger than period. */
} datalogger_time_interval_t;

/**
 * @brief Normalizes data-logger time interval period or offset to seconds.
 * 
 * @param[in] interval_type Data-logger time interval type of interval period or offset.
 * @param[in] interval Data-logger time interval period or offset for interval type.
 * @return uint64_t Normalized time interval period or offset in seconds.
 */
static inline uint64_t datalogger_normalize_interval_to_sec(const datalogger_time_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_sec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            interval_sec = interval;
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            interval_sec = (interval * 60);  // 1-minute has 60-seconds
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            interval_sec = (interval * (60 * 60)); // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_sec;
}

/**
 * @brief Normalizes data-logger time interval period or offset to milli-seconds.
 * 
 * @param[in] interval_type Data-logger time interval type of interval period or offset.
 * @param[in] interval Data-logger time interval period or offset for interval type.
 * @return uint64_t Normalized time interval period or offset in milli-seconds.
 */
static inline uint64_t datalogger_normalize_interval_to_msec(const datalogger_time_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_msec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            interval_msec = interval * 1000U;
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            interval_msec = (interval * 60) * 1000U; // 1-minute has 60-seconds
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            interval_msec = ((interval * 60) * 60) * 1000U; // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_msec;
}

/**
 * @brief Gets unix epoch timestamp (UTC) in seconds from system clock.
 * 
 * @return uint64_t Unix epoch timestamp (UTC) in seconds or it will return 0-seconds 
 * when there is an issue accessing the system clock.
 */
static inline uint64_t datalogger_get_epoch_timestamp(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;
 
    // extract unix epoch utc timestamp and convert to seconds
    return (uint64_t)tv_utc_timestamp.tv_sec;
}

/**
 * @brief Gets unix epoch timestamp (UTC) in milli-seconds from system clock.
 * 
 * @return uint64_t Unix epoch timestamp (UTC) in milli-seconds or it will return 0-milli-seconds 
 * when there is an issue accessing the system clock.
 */
static inline uint64_t datalogger_get_epoch_timestamp_msec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to milli-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000U + (uint64_t)tv_utc_timestamp.tv_usec / 1000U;
}

/**
 * @brief Gets unix epoch timestamp (UTC) in micro-seconds from system clock.
 * 
 * @return uint64_t Unix epoch timestamp (UTC) in micro-seconds or it will return 0-micro-seconds 
 * when there is an issue accessing the system clock.
 */
static inline uint64_t datalogger_get_epoch_timestamp_usec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to micro-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000000U + (uint64_t)tv_utc_timestamp.tv_usec;
}

/**
 * @brief Sets the next epoch event timestamp in milli-seconds from system clock based on 
 * the time interval type, period, and offset. 
 * 
 * The interval should be divisible by 60 i.e. no remainder if the interval type and period
 * is every 10-seconds, the event will trigger on-time with the system clock i.e. 09:00:00, 
 * 09:00:10, 09:00:20, etc.
 * 
 * The interval offset is used to offset the start of the interval period.  If the interval type
 * and period is every 5-minutes with a 1-minute offset, the event will trigger on-time with the
 * system clock i.e. 09:01:00, 09:06:00, 09:11:00, etc.
 * 
 * @param[in] interval_type Data-logger time interval type.
 * @param[in] interval_period Data-logger time interval period for interval type.
 * @param[in] interval_offset Data-logger time interval offset for interval type.
 * @param[out] epoch_timestamp Unix epoch timestamp (UTC) of next event in milli-seconds.
 */
static inline esp_err_t datalogger_set_epoch_timestamp_event(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, uint64_t *epoch_timestamp) {
    struct timeval  now_tv;
    struct tm       now_tm;
    struct tm       next_tm;

    /* validate interval period argument */
    ESP_RETURN_ON_FALSE( (interval_period > 0), ESP_ERR_INVALID_ARG, TAG_DL_COMMON, "interval period cannot be 0, data-logger set epoch time event failed" );

    /* normalize interval period and offset to milli-seconds */
    uint64_t interval_period_msec = datalogger_normalize_interval_to_msec(interval_type, interval_period);
    uint64_t interval_offset_msec = datalogger_normalize_interval_to_msec(interval_type, interval_offset);

    /* validate period and offset intervals */
    ESP_RETURN_ON_FALSE( ((interval_period_msec - interval_offset_msec) > 0), ESP_ERR_INVALID_ARG, TAG_DL_COMMON, "interval period must be larger than the interval offset, data-logger set epoch time event failed" );

    // get system unix epoch time (gmt)
    gettimeofday(&now_tv, NULL);

    // extract system unix time (seconds and milli-seconds)
    time_t now_unix_time        = now_tv.tv_sec;
    uint64_t now_unix_time_msec = (uint64_t)now_tv.tv_sec * 1000U + (uint64_t)now_tv.tv_usec / 1000U;

    // convert now tm to time-parts localtime from unix time
    localtime_r(&now_unix_time, &now_tm);

    // initialize next tm structure time-parts localtime based on interval-type
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

/**
 * @brief Converts degrees to radians.
 * 
 * @param degree degrees to convert.
 * @return double converted degrees to radians.
 */
static inline double datalogger_degrees_to_radians(double degree) {
    return degree * (M_PI / 180);
}

/**
 * @brief Converts radians to degrees.
 * 
 * @param radian radian to convert.
 * @return double converted radians to degrees.
 */
static inline double datalogger_radians_to_degrees(double radian) {
    return radian * (180 / M_PI);
}

/**
 * @brief Gets a 16-bit hash-code utilizing epoch timestamp as the seed.
 * 
 * @return uint16_t 16-bit hash-code.
 */
static inline uint16_t datalogger_get_hash_code(void) {
    uint16_t seed_hash = (uint16_t)datalogger_get_epoch_timestamp();
    return ((seed_hash>>16) ^ (seed_hash)) & 0xFFFF;
}


//https://github.com/skeeto/scratch/blob/master/misc/float16.c
/**
 * @brief Encodes a double to an int16_t data-type.
 * 
 * @param value double value to encode.
 * @return uint16_t encoded double value.
 */
static inline uint16_t datalogger_encode_double(double f) {
    uint64_t b;
    memcpy(&b, &f, 8);
    int s = (b>>48 & 0x8000);
    int e = (b>>52 & 0x07ff) - 1023;
    int m = (b>>42 & 0x03ff);
    int t = !!(b && 0xffffffffffff);

    if (e == -1023) {
        // input is denormal, round to zero
        e = m = 0;
    } else if (e < -14) {
        // convert to denormal
        if (-14 - e > 10) {
            m = 0;
        } else {
            m |= 0x400;
            m >>= -14 - e - 1;
            m = (m>>1) + (m&1);  // round
        }
        e = 0;
    } else if (e > +16) {
        // NaN / overflow to infinity
        m &= t << 9;  // canonicalize to quiet NaN
        e = 31;
    } else {
        e += 15;
    }

    return s | e<<10 | m;
}

/**
 * @brief Decodes a uint16_t to a double data-type.
 * 
 * @param value uint16_t value to decode.
 * @return double decoded uint16_t value.
 */
static inline double datalogger_decode_uint16(uint16_t x) {
    int s = (x     & 0x8000);
    int e = (x>>10 & 0x001f) - 15;
    int m = (x     & 0x03ff);

    switch (e) {
    case -15: if (!m) {
                  e = 0;
              } else {
                  // convert from denormal
                  e += 1023 + 1;
                  while (!(m&0x400)) {
                      e--;
                      m <<= 1;
                  }
                  m &= 0x3ff;
              }
              break;
    case +16: m = !!m << 9;  // canonicalize to quiet NaN
              e = 2047;
              break;
    default:  e += 1023;
    }

    uint64_t b = (uint64_t)s<<48 |
                 (uint64_t)e<<52 |
                 (uint64_t)m<<42;
    double f;
    memcpy(&f, &b, 8);
    return f;
}

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humidity in percent.
 * @return float calculated dewpoint temperature in degrees Celsius, otherwise, NAN.
 */
static inline float datalogger_dewpoint(const float temperature, const float humidity) {
    float dewpoint = NAN;

    // validate parameters
    if(temperature > 80 || temperature < -40) return NAN;
    if(humidity > 100 || humidity < 0) return NAN;
    
    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    return dewpoint = 243.12*H/(17.62-H);
}

/**
 * @brief Concatenates the `append` string to the `base` string.
 * 
 * @param base String base.
 * @param append String to append to the base.
 * @return const char* `append` string concatenated to the `base` string.
 */
static inline const char* datalogger_concat(const char* base, const char* append) {
    char *res = malloc(strlen(base) + strlen(append) + 1);
    strcpy(res, base);
    strcat(res, append);
    return res;
}















#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DATALOGGER_COMMON_H__
