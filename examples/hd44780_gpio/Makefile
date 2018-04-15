#V := 1
PROJECT_NAME := example-hd44780-gpio

EXTRA_COMPONENT_DIRS := $(CURDIR)/../../components

EXTRA_COMPONENTS := hd44780

DEFAULT_COMPONENTS := \
    esptool_py \
	heap \
	soc \
	esp32 \
	driver \
	pthread \
	partition_table \
	log \
	bootloader_support \
	micro-ecc \
	mbedtls \
	lwip \
	tcpip_adapter \
	ethernet \
	app_update \
	app_trace \
	xtensa-debug-module \
	nvs_flash \
	wpa_supplicant \
	main \
	newlib \
	vfs \
	spi_flash \
	console \
	freertos


COMPONENTS := $(DEFAULT_COMPONENTS) $(EXTRA_COMPONENTS)

include $(IDF_PATH)/make/project.mk

