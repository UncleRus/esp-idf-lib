if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 esp_timer)
else()
    set(req driver esp_timer)
endif()

idf_component_register(
    SRCS button.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
