set(COMPONENT_SRCDIRS .)
set(COMPONENT_ADD_INCLUDEDIRS .)

if(CONFIG_IDF_TARGET_ESP8266)
    set(COMPONENT_REQUIRES esp8266 freertos)
else()
    set(COMPONENT_REQUIRES driver esp32 freertos)
endif()

register_component()
