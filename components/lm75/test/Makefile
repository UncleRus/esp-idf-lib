# Makefile to build a test application
#
# this Makefile injects esp-idf-lib components into esp-idf's component path
# and builds a unit test application of ESP_IDF_LIB_TEST_CONPONENT_NAME.
#
# XXX as of Aug 2019, "Unit Testing in ESP32" omits an important assumption:
# it is written for esp-idf development, not for users. it does not work
# because users usually want to test their own components, and the component
# is not in component path.
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/unit-tests.html
#
# XXX also, the example in esp-idf is unnecessarily complicated, and
# confusing.
# https://github.com/espressif/esp-idf/blob/master/examples/system/unit_test/README.md
#
ESP_IDF_LIB_TEST_CONPONENT_NAME := lm75
PYTHON_BIN := python2

MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CURRENT_DIR := $(dir $(MAKEFILE_PATH))
ESP_IDF_LIB_COMPONENT_DIR := $(CURRENT_DIR)/../../../components
IDF_PY_EXTRA_ARGS := -DEXTRA_COMPONENT_DIRS=$(ESP_IDF_LIB_COMPONENT_DIR) -DTEST_COMPONENTS=$(ESP_IDF_LIB_TEST_CONPONENT_NAME)


build:
	(cd $(IDF_PATH)/tools/unit-test-app && \
		$(PYTHON_BIN) $(IDF_PATH)/tools/idf.py build $(IDF_PY_EXTRA_ARGS))
flash:
	(cd $(IDF_PATH)/tools/unit-test-app && \
		$(PYTHON_BIN) $(IDF_PATH)/tools/idf.py flash $(IDF_PY_EXTRA_ARGS))
clean:
	(cd $(IDF_PATH)/tools/unit-test-app && \
		$(PYTHON_BIN) $(IDF_PATH)/tools/idf.py clean)
