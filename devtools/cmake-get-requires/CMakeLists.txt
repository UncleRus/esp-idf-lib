# a script to parse CMakeLists.txt of a component, print REQUIRES.
#
# usage:
# cmake -DCOMPONENT_NAME:STRING=framebuffer .  >/dev/null
#
# output in stderr:
# REQUIRES:log;color

cmake_minimum_required(VERSION 3.5)

# mock idf_component_register()
function(idf_component_register)
    set(multiValueArgs REQUIRES)
    cmake_parse_arguments(MY "" "" "${multiValueArgs}" ${ARGN})

    # print REQUIRES argument to stderr. MY_REQUIRES is a semicolon separated
    # string, such as `foo;bar`
    message(NOTICE "REQUIRES:${MY_REQUIRES}")
endfunction()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../components/${COMPONENT_NAME}/CMakeLists.txt)

project(ProjectName)
