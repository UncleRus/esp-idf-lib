idf_component_register(
    SRCS tps63101x.c
    INCLUDE_DIRS .
    REQUIRES i2cdev esp_idf_lib_helpers
)
