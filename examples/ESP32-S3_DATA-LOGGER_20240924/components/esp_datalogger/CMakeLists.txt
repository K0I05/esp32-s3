idf_component_register(
    SRCS datalogger.c task_schedule.c time_into_interval.c datatable.c
    INCLUDE_DIRS .
    REQUIRES json esp_common esp_timer driver log
)
