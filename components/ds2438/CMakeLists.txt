idf_component_register(
    SRCS ds2438.c
    INCLUDE_DIRS .
    REQUIRES onewire log esp_idf_lib_helpers utils
)
