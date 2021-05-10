if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266)
else()
    set(req driver)
endif()

idf_component_register(
    SRCS button.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
