idf_component_register(
    SRCS "mpu6050.c" "i2c_dev_bridge.c"
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers utils
)
