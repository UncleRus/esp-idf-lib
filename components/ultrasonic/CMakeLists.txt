if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos esp_idf_lib_helpers esp_timer)
elseif(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
    set(req driver freertos esp_idf_lib_helpers)
else()
    set(req driver freertos esp_idf_lib_helpers esp_timer)
endif()

idf_component_register(
    SRCS ultrasonic.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
