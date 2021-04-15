idf_component_register(
    SRCS led_strip.c
    INCLUDE_DIRS .
    REQUIRES driver log color esp_idf_lib_helpers
)
