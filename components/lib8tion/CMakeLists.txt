if(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
	set(req)	
else()
    set(req esp_timer)
endif()

idf_component_register(
    INCLUDE_DIRS .
    SRCS lib8tion.c
    REQUIRES ${req}
)
