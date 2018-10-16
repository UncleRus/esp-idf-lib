#V := 1
PROJECT_NAME := esp-iot-node

EXTRA_COMPONENT_DIRS := $(CURDIR)/components/esp-idf-lib/components

EXTRA_COMPONENTS := json i2cdev bmp280 onewire ds18x20 dht ds3231 sdmmc wear_levelling fatfs esp-mqtt

INTERNAL_COMPONENTS := iot

DEFAULT_COMPONENTS := \
	esptool_py \
	heap \
	soc \
	esp32 \
	driver \
	pthread \
	partition_table \
	log \
	bootloader \
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
	smartconfig_ack \
	main \
	newlib \
	vfs \
	spi_flash \
	console \
	freertos


COMPONENTS := $(DEFAULT_COMPONENTS) $(EXTRA_COMPONENTS) $(INTERNAL_COMPONENTS)

include $(IDF_PATH)/make/project.mk

