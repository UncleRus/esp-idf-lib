if(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
	message("Not including esp_timer in color")
    	set(req lib8tion)	
else()
    message("Including esp_timer in color")
    set(req lib8tion)	
endif()

idf_component_register(
    SRCS color.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
