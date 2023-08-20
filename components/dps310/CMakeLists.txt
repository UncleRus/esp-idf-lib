idf_component_register(

    # sources to compile
    SRCS src/dps310.c src/helper_i2c.c

    # public headers
    INCLUDE_DIRS include

    # private headers
    PRIV_INCLUDE_DIRS priv_include

    # Component Requirements
    # https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#component-requirements

    # components whose header files are #included from the public header files
    # of this component.
    REQUIRES log i2cdev

    # components whose header files are #included from any source files in
    # this component, unless already listed in REQUIRES
    PRIV_REQUIRES esp_idf_lib_helpers i2cdev
)
