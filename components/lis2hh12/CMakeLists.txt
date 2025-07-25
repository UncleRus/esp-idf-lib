idf_component_register(
    SRCS lis2hh12.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
