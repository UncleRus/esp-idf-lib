cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../../components)
set(TEST_COMPONENTS "example" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unit_test_test)
