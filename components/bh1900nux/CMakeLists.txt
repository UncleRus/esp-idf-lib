idf_component_register(
    SRCS bh1900nux.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
