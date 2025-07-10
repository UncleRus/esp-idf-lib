COMPONENT_ADD_INCLUDEDIRS = .

ifdef CONFIG_IDF_TARGET_ESP8266
COMPONENT_DEPENDS = esp8266 freertos esp_idf_lib_helpers
# ESP8266 RTOS SDK auto-detects all .c files, so use COMPONENT_OBJS to override
# This prevents both i2cdev.c and i2cdev_legacy.c from being compiled
COMPONENT_OBJS := i2cdev_legacy.o
COMPONENT_SRCDIRS := .
else
COMPONENT_DEPENDS = driver freertos esp_idf_lib_helpers
# For ESP32 family, check for manual override first
ifdef CONFIG_I2CDEV_USE_LEGACY_DRIVER
COMPONENT_SRCS = i2cdev_legacy.c
else
# Check if version variables are available, fallback to legacy if not
ifdef IDF_VERSION_MAJOR
ifeq ($(shell test $(IDF_VERSION_MAJOR) -lt 5 && echo 1),1)
COMPONENT_SRCS = i2cdev_legacy.c
else ifeq ($(shell test $(IDF_VERSION_MAJOR) -eq 5 -a $(IDF_VERSION_MINOR) -lt 3 && echo 1),1)
COMPONENT_SRCS = i2cdev_legacy.c
else
COMPONENT_SRCS = i2cdev.c
endif
else
# Version variables not available - fallback to legacy driver for safety
COMPONENT_SRCS = i2cdev_legacy.c
endif
endif
endif
