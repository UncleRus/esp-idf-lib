if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos log)
else()
    set(req driver freertos log)
endif()

idf_component_register(
    SRCS encoder.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
