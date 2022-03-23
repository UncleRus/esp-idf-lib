if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 log color esp_idf_lib_helpers)
else()
    set(req driver log color esp_idf_lib_helpers)
endif()

idf_component_register(
    SRCS led_strip_spi.c led_strip_spi_sk9822.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror=override-init)
