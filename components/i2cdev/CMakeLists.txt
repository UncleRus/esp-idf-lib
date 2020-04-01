set(COMPONENT_SRCDIRS .)
set(COMPONENT_ADD_INCLUDEDIRS .)

if(${IDF_TARGET} STREQUAL esp8266)
    set(COMPONENT_REQUIRES esp8266 freertos esp_idf_lib_helpers)
else()
    set(COMPONENT_REQUIRES driver freertos esp_idf_lib_helpers)
endif()

register_component()
