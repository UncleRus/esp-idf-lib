idf_component_register(
    SRCS hts221.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers esp_timer
)
