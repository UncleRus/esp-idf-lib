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
#if defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_IDF_TARGET_ESP8266)
#error BUG: defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_IDF_TARGET_ESP8266)
#endif
/* }}} */

/* Constant macros for TARGET and SDK version
 */
#define HELPER_TARGET_VERSION_ESP32_V0        (32000000)
#define HELPER_TARGET_VERSION_ESP32_V3_2      (32030200)
#define HELPER_TARGET_VERSION_ESP32_V4        (32040000)
#define HELPER_TARGET_VERSION_ESP32_V_MAX     (32999999)
#define HELPER_TARGET_VERSION_ESP8266_V0    (8266000000)
#define HELPER_TARGET_VERSION_ESP8266_V3_2  (8266030200)
#define HELPER_TARGET_VERSION_ESP8266_V_MAX (8266999999)

/* Target and SDK version guestimation.
 *
 * ESP_IDF_VERSION_VAL macro is provided in esp-idf 4.x, or the current master
 * branch. Other esp-idf versions and ESP8266 RTOS SDK do not provide macro to
 * branch code flow.
 */

#if defined(CONFIG_IDF_TARGET_ESP32)
/* esp32 and esp-idf 4.x */
#define HELPER_TARGET_VERSION HELPER_TARGET_VERSION_ESP32_V4

#elif defined(CONFIG_IDF_TARGET_ESP8266)

/* ESP8266 RTOS SDK 3.2 */
#define HELPER_TARGET_VERSION HELPER_TARGET_VERSION_ESP8266_V3_2

#elif !defined(CONFIG_IDF_TARGET_ESP32) && !defined(IDF_VERSION_MAJOR)
/* esp-idf 3.2 does not define CONFIG_IDF_TARGET_*, nor IDF_VERSION_MAJOR.
 *
 * use this guestimation until ESP_IDF_VERSION_VAL is ported to all SDKs
 */
#define HELPER_TARGET_VERSION HELPER_TARGET_VERSION_ESP32_V3_2
#else
#error BUG: Cannot guess target version
#endif

/* HELPER_TARGET_IS_ESP32
 *
 * 1 when the target is esp32
 */

#if (HELPER_TARGET_VERSION >= HELPER_TARGET_VERSION_ESP32_V0) && (HELPER_TARGET_VERSION <= HELPER_TARGET_VERSION_ESP32_V_MAX)
#define HELPER_TARGET_IS_ESP32     (1)
#define HELPER_TARGET_IS_ESP8266   (0)

/* HELPER_TARGET_IS_ESP8266
 * 1 when the target is esp8266
 */
#elif HELPER_TARGET_VERSION >= HELPER_TARGET_VERSION_ESP8266_V0 && HELPER_TARGET_VERSION <= HELPER_TARGET_VERSION_ESP8266_V_MAX
#define HELPER_TARGET_IS_ESP32     (0)
#define HELPER_TARGET_IS_ESP8266   (1)
#else
#error BUG: cannot determine the target
#endif

/*  {{{ post-tests  */
#if HELPER_TARGET_IS_ESP32 == 1 && HELPER_TARGET_IS_ESP8266 == 1
#error BUG: HELPER_TARGET_IS_ESP32 == 1 && HELPER_TARGET_IS_ESP8266 == 1
#endif

#if HELPER_TARGET_IS_ESP32 && HELPER_TARGET_IS_ESP8266
#error BUG: HELPER_TARGET_IS_ESP32 && HELPER_TARGET_IS_ESP8266
#endif

#if HELPER_TARGET_IS_ESP32 == 0 && HELPER_TARGET_IS_ESP8266 == 0
#error BUG: HELPER_TARGET_IS_ESP32 == 0 && HELPER_TARGET_IS_ESP8266 == 0
#endif

#if !HELPER_TARGET_IS_ESP32 && !HELPER_TARGET_IS_ESP8266
#error BUG: !HELPER_TARGET_IS_ESP32 && !HELPER_TARGET_IS_ESP8266
#endif

#if HELPER_TARGET_VERSION > HELPER_TARGET_VERSION_ESP8266_V_MAX || HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP32_V0
#error BUG: HELPER_TARGET_VERSION range under/overflow
#endif

#if HELPER_TARGET_IS_ESP32 && HELPER_TARGET_VERSION >= HELPER_TARGET_VERSION_ESP8266_V0
#error HELPER_TARGET_IS_ESP32 is true but HELPER_TARGET_VERSION  >= HELPER_TARGET_VERSION_ESP8266_V0
#endif

#if HELPER_TARGET_IS_ESP8266 && HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP8266_V0
#error HELPER_TARGET_IS_ESP8266 is true but HELPER_TARGET_VERSION < HELPER_TARGET_VERSION_ESP8266_V0
#endif
/* }}} */

/* show the actual values for debugging */
#if DEBUG
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP32))
#pragma message(VAR_NAME_VALUE(CONFIG_IDF_TARGET_ESP8266))
#pragma message(VAR_NAME_VALUE(HELPER_TARGET_VERSION))
#endif

#endif
