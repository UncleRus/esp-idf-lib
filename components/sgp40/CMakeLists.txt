idf_component_register(
    SRCS sgp40.c sensirion_voc_algorithm.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
