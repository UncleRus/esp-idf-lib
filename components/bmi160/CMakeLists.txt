if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos i2cdev log esp_idf_lib_helpers)
elseif(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
    set(req driver freertos i2cdev log esp_idf_lib_helpers)
else()
    set(req driver freertos i2cdev log esp_idf_lib_helpers)
endif()

idf_component_register(
    SRCS "bmi160.c"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
