if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 log)
else()
    set(req driver log)
endif()

idf_component_register(
    SRCS hts221.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
