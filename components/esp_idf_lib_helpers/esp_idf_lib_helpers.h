#if !defined(__ESP_IDF_LIB_HELPERS__H__)
#define __ESP_IDF_LIB_HELPERS__H__

/* XXX this header file does not need to include freertos/FreeRTOS.h.
 * but without it, ESP8266 RTOS SDK does not include `sdkconfig.h` in correct
 * order. as this header depends on sdkconfig.h, sdkconfig.h must be included
 * first. however, the SDK includes this header first, then includes
 * `sdkconfig.h` when freertos/FreeRTOS.h is not explicitly included. an
 * evidence can be found in `build/${COMPONENT}/${COMPONENT}.d` in a failed
 * build.
 */
#include <freertos/FreeRTOS.h>

/* {{{ pre-tests */
#if defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_IDF_TARGET_ESP8266) && defined(CONFIG_IDF_TARGET_ESP32S2)
#error BUG: defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_IDF_TARGET_ESP8266) && defined(CONFIG_IDF_TARGET_ESP32S2)
#endif
/* }}} */

#if (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)) && !defined(ESP_IDF_VERSION_MAJOR)
#error Unsupported ESP-IDF version (<= v3.5). Please update!
#endif

/* HELPER_TARGET_IS_ESP32
 *
 * 1 when the target is esp32
 */
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
#define HELPER_TARGET_IS_ESP32     (1)
#define HELPER_TARGET_IS_ESP8266   (0)

/* HELPER_TARGET_IS_ESP8266
 * 1 when the target is esp8266
 */
#elif defined(CONFIG_IDF_TARGET_ESP8266)
#define HELPER_TARGET_IS_ESP32     (0)
#define HELPER_TARGET_IS_ESP8266   (1)
#else
#error BUG: cannot determine the target
#endif

/* show the actual values for debugging */
#if DEBUG
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32S2))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP8266))
#pragma message(VAR_NAME_VALUE(ESP_IDF_VERSION_MAJOR))
#endif

#endif
