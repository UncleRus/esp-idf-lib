idf_component_register(
    SRCS ds3502.c
    INCLUDE_DIRS .
    REQUIRES i2cdev esp_idf_lib_helpers
)
