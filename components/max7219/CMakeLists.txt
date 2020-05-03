set(COMPONENT_SRCDIRS .)
set(COMPONENT_ADD_INCLUDEDIRS .)

if(${IDF_TARGET} STREQUAL esp8266)
    set(COMPONENT_REQUIRES esp8266 log)
else()
    set(COMPONENT_REQUIRES driver log)
endif()

register_component()
