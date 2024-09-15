if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos esp_idf_lib_helpers)
else()
    set(req driver freertos esp_idf_lib_helpers)
endif()

set(srcs "i2cdev.c")
if(CONFIG_I2CDEV_USING_LEGACY_I2C)
    list(APPEND srcs "i2cdev_legacy.c")
else()
    list(APPEND srcs "i2cdev_master.c")
endif()

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
