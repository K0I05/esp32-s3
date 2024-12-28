idf_component_register(
    SRCS ds18b20.c
    INCLUDE_DIRS .
    REQUIRES onewire_bus esp_common esp_timer log
)
