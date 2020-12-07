idf_component_register(
    SRCS "hmc5883l.c"
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
