/**
 * @file main.c
 * @brief ESP Data-Logger Component
 *
 * i2c sensors: AHT20 + BMP280 
 * 
 * This example takes the parameters 
 *
 *  Sensor Board AHT20 + BMP280 Wiring
 *  VIN -> MCU VDD 3.3V Pin
 *  GNG -> MCU GND Pin
 *  SCL -> MCU SCL Pin
 *  SDA -> MCU SDA Pin
 *  
 *  BMP280 I2C Address: 0x77
 *  AHT20  I2C Address: 0x38
 *  
 * 
 * CTRL + SHIFT + P
 * pio run -t menufconfig
 * k & l keys for up or down
 * OR
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * 
 * `pio system prune` to save disk space
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <datalogger.h>
#include <bmp280.h>
#include <ahtxx.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "AHT20+BMP280 [APP]"

// macros
#define CONFIG_I2C_0_MASTER_DEFAULT {                               \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = CONFIG_I2C_0_PORT,        \
        .scl_io_num                     = CONFIG_I2C_0_SCL_IO,      \
        .sda_io_num                     = CONFIG_I2C_0_SDA_IO,      \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

//
// data-table variables
static datatable_handle_t dt_sample_hdl;
static datatable_config_t dt_sample_cfg = {
        .columns_size               = 9, 
        .rows_size                  = 10, 
        .sampling_interval_type     = DATALOGGER_TIME_INTERVAL_SEC, 
        .sampling_interval_period   = 10, 
        .processing_interval_type   = DATALOGGER_TIME_INTERVAL_MIN, 
        .processing_interval_period = 1, 
        .data_storage_type          = DATATABLE_DATA_STORAGE_MEMORY_RING
    };
static uint8_t dt_pa_avg_column_index;
static uint8_t dt_ta_avg_column_index;
static uint8_t dt_ta_max_column_index;
static uint8_t dt_ta_min_column_index;
static uint8_t dt_td_avg_column_index;
static uint8_t dt_rh_avg_column_index;
static uint8_t dt_rh_max_ts_column_index;
static uint8_t dt_rh_min_ts_column_index;
static uint8_t dt_wsd_avg_column_index;


static inline void datatable_print_columns(void) {
    if(dt_sample_hdl == NULL || dt_sample_hdl->columns_size == 0) {
        ESP_LOGW(CONFIG_APP_TAG, "data-table columns error: intialize data-table");
        return;
    } else {
        ESP_LOGW(CONFIG_APP_TAG, "data-table (%s) columns:", dt_sample_hdl->name);
    }
    
    for(uint8_t i = 0; i <= dt_sample_hdl->columns_index; i++) {
        datatable_column_t col = dt_sample_hdl->columns[i];
        ESP_LOGW(CONFIG_APP_TAG, "->name (%d): %s", i, col.names[0].name);
    }

    /*
    // print recorded records
    //
    // data-table columns - header
    
    ESP_LOGI(CONFIG_APP_TAG, "ID   %s %s %s %s %s %s",
            dt_sample_hdl->columns[dt_pa_column_index].name, 
            dt_sample_hdl->columns[dt_ta_column_index].name,
            dt_sample_hdl->columns[dt_ta_min_column_index].name,
            dt_sample_hdl->columns[dt_ta_max_column_index].name,
            dt_sample_hdl->columns[dt_td_column_index].name,
            dt_sample_hdl->columns[dt_rh_column_index].name);

    for(uint16_t row_index = 0; row_index < dt_sample_hdl->rows_count; row_index++) {
        float dt_pa     = dt_sample_hdl->rows[row_index].data_columns[dt_pa_column_index].data.float_data.value;
        float dt_ta     = dt_sample_hdl->rows[row_index].data_columns[dt_ta_column_index].data.float_data.value;
        float dt_ta_min = dt_sample_hdl->rows[row_index].data_columns[dt_ta_min_column_index].data.float_data.value;
        float dt_ta_max = dt_sample_hdl->rows[row_index].data_columns[dt_ta_max_column_index].data.float_data.value;
        float dt_td     = dt_sample_hdl->rows[row_index].data_columns[dt_td_column_index].data.float_data.value;
        float dt_rh     = dt_sample_hdl->rows[row_index].data_columns[dt_rh_column_index].data.float_data.value;
        //
        ESP_LOGI(CONFIG_APP_TAG, "[%d]    %.2f      %.2f      %.2f      %.2f      %.2f      %.2f",
            dt_sample_hdl->rows[row_index].data_columns[0].data.id_data.value,
            dt_pa, dt_ta, dt_ta_min, dt_ta_max, dt_td, dt_rh);
    }
    */
}

/*
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    uint64_t time_ms = (uint64_t)tv_now.tv_sec * 1000U + (uint64_t)tv_now.tv_usec / 1000U;
    uint64_t time_us = (uint64_t)tv_now.tv_sec * 1000000U + (uint64_t)tv_now.tv_usec;

    ESP_LOGI(TAG, "UNIX time in mseconds: %lld", time_ms);
    ESP_LOGI(TAG, "UNIX time in useconds: %lld", time_us);
*/

static inline void set_time( void ) {
    // Prepare the broken-down time.
    // See https://en.cppreference.com/w/c/chrono/tm
    struct tm initial_time = {
        .tm_year    = 2024,
        .tm_mon     = 8,
        .tm_mday    = 13,
        .tm_hour    = 14,
        .tm_min     = 00,
        .tm_sec     = 50
    };
    // Convert to Unix time.
    // See https://en.cppreference.com/w/c/chrono/mktime
    time_t initial_unix_time = mktime(&initial_time);
    // Convert to 'struct timeval'
    struct timeval initial_timeval = {
        .tv_sec     = initial_unix_time,
        .tv_usec    = 0
    };
    // Set system time.
    // See https://linux.die.net/man/2/settimeofday
    int err = settimeofday(&initial_timeval, NULL);
    assert(err == 0);
    printf("Time set to:          %s", asctime(&initial_time));
}

static inline void get_time( void ) {
    // Get current time as 'struct timeval'.
    // See https://linux.die.net/man/2/gettimeofday
    struct timeval new_timeval;
    int err = gettimeofday(&new_timeval, NULL);
    assert(err == 0);
    // Extract Unix time
    time_t new_unix_time = new_timeval.tv_sec;
    // Convert to broken-down time
    // See https://en.cppreference.com/w/c/chrono/localtime
    struct tm new_time;
    localtime_r(&new_unix_time, &new_time);
    // 'new_time' now contains the current time components
    printf("Current time:         %s", asctime(&new_time)); 
}

static void i2c_0_task( void *pvParameters ) {
    task_schedule_handle_t      task_schedule_hdl;
    time_into_interval_handle_t time_into_interval_hdl;
    float                       ta; 
    float                       td; 
    float                       rh;
    float                       pa;
    float                       ta_max          = NAN;
    float                       ta_min          = NAN;
    float                       pa_max          = NAN;
    float                       pa_min          = NAN;
    char                        deg_char        = 176;
    uint64_t                    start_time      = 0;
    uint64_t                    end_time        = 0;
    int64_t                     delta_time      = 0;
    //
    //
    // instantiate a new time into interval handle - task system clock synchronization
    time_into_interval_new(DATALOGGER_TIME_INTERVAL_MIN, 5, 0, &time_into_interval_hdl);
    if (time_into_interval_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 time_into_new_interval new time into interval handle failed");
    // 
    // instantiate a new task schedule handle - task system clock synchronization
    task_schedule_new(DATALOGGER_TIME_INTERVAL_SEC, 10, 0, &task_schedule_hdl);
    if (task_schedule_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 task_new_schedule new task schedule handle failed");
    //
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize bmp280 i2c device configuration
    i2c_bmp280_config_t         bmp280_dev_cfg = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t         bmp280_dev_hdl;
    //
    // initialize aht20 i2c device configuration
    i2c_ahtxx_config_t          aht20_dev_cfg = I2C_AHT2X_CONFIG_DEFAULT;
    i2c_ahtxx_handle_t          aht20_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create new master bus handle failed");
    //
    // init i2c devices
    //
    // bmp280 init device
    i2c_bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create bmp280 handle init failed");
    //
    // aht20 init device
    i2c_ahtxx_init(i2c0_bus_hdl, &aht20_dev_cfg, &aht20_dev_hdl);
    if (aht20_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create aht2x handle init failed");
    //
    // 
    //
    // task loop entry point
    for ( ;; ) {
        // set start timer
        start_time = esp_timer_get_time(); 
        //
        // delay task until task schedule condition is valid
        task_schedule_delay(task_schedule_hdl);
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## AHT20+BMP280 - START #########################");
        //
        ESP_LOGI(CONFIG_APP_TAG, "free memory:  %lu bytes", esp_get_free_heap_size());
        //
        // handle bmp280 sensor
        if(i2c_bmp280_get_pressure(bmp280_dev_hdl, &pa) != 0) {
            ESP_LOGE(CONFIG_APP_TAG, "i2c_bmp280_get_pressure failed");
        } else {
            // convert pressure to hPa
            pa = pa / 100;
            //
            // process min and max pa since reboot
            if(isnan(pa_max) && isnan(pa_min)) {
                // initialize
                pa_max = pa;
                pa_min = pa;
            }
            //
            // set min and max values
            if(pa > pa_max) pa_max = pa;
            if(pa < pa_min) pa_min = pa;
            //
            ESP_LOGI(CONFIG_APP_TAG, "pressure:     %.1f hPa", pa);
            ESP_LOGI(CONFIG_APP_TAG, "pressure-min: %.1f hPa", pa_min);
            ESP_LOGI(CONFIG_APP_TAG, "pressure-max: %.1f hPa", pa_max);
        }
        //
        // handle aht2x sensor
        if(i2c_ahtxx_get_measurements(aht20_dev_hdl, &ta, &rh, &td) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_ahtxx_get_measurements failed");
        } else {
            //
            // process min and max ta since reboot
            if(isnan(ta_max) && isnan(ta_min)) {
                // initialize
                ta_max = ta;
                ta_min = ta;
            }
            //
            // set min and max values
            if(ta > ta_max) ta_max = ta;
            if(ta < ta_min) ta_min = ta;
            //
            ESP_LOGI(CONFIG_APP_TAG, "air:          %.2f%cC", ta, deg_char);
			ESP_LOGI(CONFIG_APP_TAG, "air-min:      %.2f%cC", ta_min, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "air-max:      %.2f%cC", ta_max, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "dew-point:    %.2f%cC", td, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "humidity:     %.2f %%", rh);
        }
        //
        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_sample_hdl, dt_pa_avg_column_index, pa);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_avg_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_min_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_max_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_td_avg_column_index, td);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_avg_column_index, rh);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_min_ts_column_index, rh);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_max_ts_column_index, rh);
        //
        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_sample_hdl);
        //
        // print data-table in json format at specified interval
        
        if(time_into_interval(time_into_interval_hdl)) {
            char *dt_json = "";
            datatable_to_json(dt_sample_hdl, dt_json);
	        ESP_LOGI(CONFIG_APP_TAG, "JSON Data-Table:\n%s",dt_json);
        }
    
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## AHT20+BMP280 - END ###########################");
        //
        // set end timer
        end_time = esp_timer_get_time();
        //
        // compute delta time 
        delta_time = end_time - start_time;
        //
        ESP_LOGI(CONFIG_APP_TAG, "Task Duration: %llu us / %llu ms", delta_time, delta_time / 1000);
    }
    //
    // free up task resources and remove task from stack
    i2c_bmp280_rm( bmp280_dev_hdl );    // remove bmp280 device from master i2c bus
    i2c_ahtxx_rm( aht20_dev_hdl );      // remove aht20 device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl ); // delete master i2c bus
    task_schedule_del( task_schedule_hdl );  // delete task schedule
    time_into_interval_del( time_into_interval_hdl ); //delete time into interval
    vTaskDelete( NULL );
}

void app_main( void ) {
    ESP_LOGI(CONFIG_APP_TAG, "Startup..");
    ESP_LOGI(CONFIG_APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(CONFIG_APP_TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(CONFIG_APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // initialize system time - manually populated time structure
    set_time();
    get_time();

    // data-table testing
    //
    // create new data-table handle
    datatable_new("Tbl_1-Min", &dt_sample_cfg, &dt_sample_hdl);
    //
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Pa_1-Min", &dt_pa_avg_column_index);                 // column index 2
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_avg_column_index);                 // column index 3
    // add float minimum column to data-table
    datatable_add_float_min_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_min_column_index);                 // column index 4
    // add float maximum column to data-table
    datatable_add_float_max_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_max_column_index);                 // column index 5
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Td_1-Min", &dt_td_avg_column_index);                 // column index 6
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_avg_column_index);                 // column index 7
    // add float minimum timestamp column to data-table
    datatable_add_float_min_ts_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_min_ts_column_index);           // column index 8
    // add float maximum timestamp column to data-table
    datatable_add_float_max_ts_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_max_ts_column_index);           // column index 9
    // add vector average column to data-table
    datatable_add_vector_avg_column(dt_sample_hdl, "Wd_1-Min", "Ws_1-Min", &dt_wsd_avg_column_index);   // column index 10
    //
    ESP_LOGW(CONFIG_APP_TAG, "data-table id column index:        %d", 0);
    ESP_LOGW(CONFIG_APP_TAG, "data-table ts column index:        %d", 1);
    ESP_LOGW(CONFIG_APP_TAG, "data-table pa column index:        %d", dt_pa_avg_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table ta column index:        %d", dt_ta_avg_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table ta-min column index:    %d", dt_ta_min_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table ta-max column index:    %d", dt_ta_max_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table td column index:        %d", dt_td_avg_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table rh column index:        %d", dt_rh_avg_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table rh-min-ts column index: %d", dt_rh_min_ts_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table rh-max-ts column index: %d", dt_rh_max_ts_column_index);
    ESP_LOGW(CONFIG_APP_TAG, "data-table ws-wd column index:     %d", dt_wsd_avg_column_index);
    //
    // print data-table columns
    //datatable_print_columns();

    char *dt_json = "";
    datatable_to_json(dt_sample_hdl, dt_json);
	ESP_LOGI(CONFIG_APP_TAG, "JSON Data-Table:\n%s",dt_json);

    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    
}