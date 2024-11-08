idf_component_register(
    SRCS datatable.c
    INCLUDE_DIRS .
    REQUIRES json esp_common esp_timer driver log esp_time_into_interval
)
