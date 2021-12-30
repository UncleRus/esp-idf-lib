idf_component_register(
    SRCS htu21d.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
