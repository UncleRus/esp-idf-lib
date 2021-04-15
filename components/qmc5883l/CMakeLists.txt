idf_component_register(
    SRCS qmc5883l.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
