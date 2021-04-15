if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 log esp_idf_lib_helpers)
else()
    set(req driver log esp_idf_lib_helpers)
endif()

idf_component_register(
    SRCS wiegand.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
