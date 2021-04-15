idf_component_register(
    SRCS ds18x20.c
    INCLUDE_DIRS .
    REQUIRES onewire freertos log esp_idf_lib_helpers
)
