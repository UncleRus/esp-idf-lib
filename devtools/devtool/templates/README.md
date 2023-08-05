# ESP-IDF Components library

[![Main CI process](https://github.com/UncleRus/esp-idf-lib/actions/workflows/ci.yml/badge.svg)](https://github.com/UncleRus/esp-idf-lib/actions/workflows/ci.yml)
[![Docs Status](https://readthedocs.org/projects/esp-idf-lib/badge/?version=latest&style=flat)](https://esp-idf-lib.readthedocs.io/en/latest/)

Components for Espressif ESP32 [ESP-IDF framework](https://github.com/espressif/esp-idf)
and [ESP8266 RTOS SDK](https://github.com/espressif/ESP8266_RTOS_SDK).

Part of them ported from [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos).

## Supported versions of frameworks and devices

| Chip     | Framework        | Versions                                                                                                                                         |
|----------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| ESP32-xx | ESP-IDF          | All officially supported versions (see [Support Period Policy](https://github.com/espressif/esp-idf/blob/master/SUPPORT_POLICY.md)) and `master` |
| ESP8266  | ESP8266 RTOS SDK | `master`, v3.4                                                                                                                                   |

*See "Supported on" column for each of the components.*

## How to use

### ESP32-xx

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/UncleRus/esp-idf-lib.git
```

Add path to components in your [CMakeLists.txt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html):
e.g:

```CMake
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS /home/user/myprojects/esp/esp-idf-lib/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

or with CMake [FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html)

```CMake
cmake_minimum_required(VERSION 3.11)
include(FetchContent)
FetchContent_Declare(
  espidflib
  GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
)
FetchContent_MakeAvailable(espidflib)
set(EXTRA_COMPONENT_DIRS ${espidflib_SOURCE_DIR}/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my-esp-project)
```

### ESP8266 RTOS SDK

Clone this repository somewhere, e.g.:

```Shell
cd ~/myprojects/esp
git clone https://github.com/UncleRus/esp-idf-lib.git
```

Add path to components in your [project makefile](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-guides/build-system.html),
e.g:

```Makefile
PROJECT_NAME := my-esp-project
EXTRA_COMPONENT_DIRS := /home/user/myprojects/esp/esp-idf-lib/components
EXCLUDE_COMPONENTS := max7219 mcp23x17 led_strip max31865 ls7366r max31855
include $(IDF_PATH)/make/project.mk
```

See [GitHub examples](https://github.com/UncleRus/esp-idf-lib/tree/master/examples)
or [GitLab examples](https://gitlab.com/UncleRus/esp-idf-lib/tree/master/examples).

## Documentation

- [Documentation](https://esp-idf-lib.readthedocs.io/en/latest/)
- [Frequently asked questions](FAQ.md)

## Components
{% for g_name, components in groups|dictsort(False) %}
### {{ g_name }}

| Component                | Description                                                                      | License | Supported on       | Thread safety |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|---------------|
{%- for c in components %}
| {{ '%-24s' % ('**%s**' % c.name) }} | {{ '%-80s' % c.description }} | {{ '%-7s' % c.license.value }} | {{ '%-18s' % c.targets|join(', ') }} | {{ '%-13s' % c.thread_safe.value }} | 
{%- endfor %}

{% endfor -%} 

## Library maintainers

- [Ruslan V. Uss](https://github.com/UncleRus)
- [Tomoyuki Sakurai](https://github.com/trombik)

## Credits
{% for full_name, copyrights in authors|dictsort() -%}

{%- if copyrights.person.gh_id -%}
    {% set person_name = '[%s](https://github.com/%s)' % (full_name, copyrights.person.gh_id) %}
{%- elif copyrights.person.url -%}
    {% set person_name = '[%s](%s)' % (full_name, copyrights.person.url) %}
{%- else -%}
    {% set person_name = full_name %}
{%- endif %}
- {{ person_name }}: {% for c in copyrights.components %}`{{ c.name }}` {% endfor %} 

{%- endfor %}
