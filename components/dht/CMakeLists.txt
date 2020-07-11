if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos esp_idf_lib_helpers)
else()
    set(req driver esp32 freertos esp_idf_lib_helpers)
endif()

idf_component_register(
    SRCS "dht.c"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
