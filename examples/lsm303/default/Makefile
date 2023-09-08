#V := 1
PROJECT_NAME := example-lsm303

EXTRA_COMPONENT_DIRS := $(CURDIR)/../../../components

#ifdef CONFIG_IDF_TARGET_ESP8266
EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip max31865 ls7366r max31855 ads130e08
#endif

include $(IDF_PATH)/make/project.mk
