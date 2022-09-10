// Copyright 2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdint.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_MEM_ALLOCATION_SPIRAM
#define MALLOC_CAP_INDICATE MALLOC_CAP_SPIRAM
#else
#define MALLOC_CAP_INDICATE MALLOC_CAP_DEFAULT
#endif

#ifndef __FILENAME__
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)
#endif /* __FILENAME__ */

/**
 * @brief  Malloc memory
 *
 * @param[in]  size  memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_MALLOC(size)                                                                                               \
    ({                                                                                                                 \
        void *ptr = heap_caps_malloc(size, MALLOC_CAP_INDICATE);                                                       \
        if (!ptr)                                                                                                      \
        {                                                                                                              \
            ESP_LOGE(TAG, "[%s, %d] <ESP_ERR_NO_MEM> Malloc size: %d, ptr: %p, heap free: %d", __func__, __LINE__,     \
                (int)size, ptr, esp_get_free_heap_size());                                                             \
            return ESP_ERR_NO_MEM;                                                                                     \
        }                                                                                                              \
        ptr;                                                                                                           \
    })

/**
 * @brief  Calloc memory
 *
 * @param[in]  n     number of block
 * @param[in]  size  block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_CALLOC(n, size)                                                                                            \
    ({                                                                                                                 \
        void *ptr = heap_caps_calloc(n, size, MALLOC_CAP_INDICATE);                                                    \
        if (!ptr)                                                                                                      \
        {                                                                                                              \
            ESP_LOGW(TAG, "[%s, %d] <ESP_ERR_NO_MEM> Calloc size: %d, ptr: %p, heap free: %d", __func__, __LINE__,     \
                (int)(n) * (size), ptr, esp_get_free_heap_size());                                                     \
            return ESP_ERR_NO_MEM;                                                                                     \
        }                                                                                                              \
        ptr;                                                                                                           \
    })

/**
 * @brief  Reallocate memory
 *
 * @param[in]  ptr   memory pointer
 * @param[in]  size  block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_REALLOC(ptr, size)                                                                                         \
    ({                                                                                                                 \
        void *new_ptr = heap_caps_realloc(ptr, size, MALLOC_CAP_INDICATE);                                             \
        if (!new_ptr)                                                                                                  \
        {                                                                                                              \
            ESP_LOGW(TAG, "[%s, %d] <ESP_ERR_NO_MEM> Realloc size: %d, new_ptr: %p, heap free: %d", __func__,          \
                __LINE__, (int)size, new_ptr, esp_get_free_heap_size());                                               \
            return ESP_ERR_NO_MEM;                                                                                     \
        }                                                                                                              \
        new_ptr;                                                                                                       \
    })

/**
 * @brief  Reallocate memory, If it fails, it will retry until it succeeds
 *
 * @param[in]  ptr   memory pointer
 * @param[in]  size  block memory size
 *
 * @return
 *     - valid pointer on success
 *     - NULL when any errors
 */
#define ESP_REALLOC_RETRY(ptr, size)                                                                                   \
    ({                                                                                                                 \
        void *new_ptr = NULL;                                                                                          \
        while (size > 0 && !(new_ptr = heap_caps_realloc(ptr, size, MALLOC_CAP_INDICATE)))                             \
        {                                                                                                              \
            ESP_LOGW(TAG, "[%s, %d] <ESP_ERR_NO_MEM> Realloc size: %d, new_ptr: %p, heap free: %d", __func__,          \
                __LINE__, (int)size, new_ptr, esp_get_free_heap_size());                                               \
            vTaskDelay(pdMS_TO_TICKS(100));                                                                            \
        }                                                                                                              \
        new_ptr;                                                                                                       \
    })

/**
 * @brief  Free memory
 *
 * @param[in]  ptr  memory pointer
 */
#define ESP_FREE(ptr)                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
        if (ptr)                                                                                                       \
        {                                                                                                              \
            heap_caps_free(ptr);                                                                                       \
            ptr = NULL;                                                                                                \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * Macro which can be used to check the error code,
 * and terminate the program in case the code is not ESP_OK.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 * Disabled if assertions are disabled.
 */
#define ESP_ERROR_RETURN(con, err, format, ...)                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        if (con)                                                                                                       \
        {                                                                                                              \
            if (*format != '\0')                                                                                       \
                ESP_LOGW(TAG, "[%s, %d] <%s> " format, __func__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__);       \
            return err;                                                                                                \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * @brief Macro serves similar purpose as ``assert``, except that it checks `esp_err_t`
 *        value rather than a `bool` condition. If the argument of `ESP_ERROR_ASSERT`
 *        is not equal `ESP_OK`, then an error message is printed on the console,
 *         and `abort()` is called.
 *
 * @note If `IDF monitor` is used, addresses in the backtrace will be converted
 *       to file names and line numbers.
 *
 * @param[in]  err [description]
 * @return         [description]
 */
#define ESP_ERROR_ASSERT(err)                                                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __err_rc = (err);                                                                                    \
        if (__err_rc != ESP_OK)                                                                                        \
        {                                                                                                              \
            ESP_LOGW(TAG, "[%s, %d] <%s> ESP_ERROR_ASSERT failed, at 0x%08x, expression: %s", __func__, __LINE__,      \
                esp_err_to_name(__err_rc), (intptr_t)__builtin_return_address(0) - 3, __ASSERT_FUNC);                  \
            assert(0 && #err);                                                                                         \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * Macro which can be used to check the pointer,
 * and terminate the program in case the pointer is NULL.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_RETURN_ON_NULL(ptr)                                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(ptr))                                                                                                    \
        {                                                                                                              \
            ESP_LOGW(TAG, "[%s, %d] <NULL POINTER> heap free: %d", __func__, __LINE__, esp_get_free_heap_size());      \
            return ESP_ERR_NO_MEM;                                                                                     \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * Macro which can be used to print the error code,
 * and terminate the program in case the pointer is not ESP_OK.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_ERROR_PRINT(con, err, format, ...)                                                                         \
    if (con)                                                                                                           \
    {                                                                                                                  \
        if (*format != '\0')                                                                                           \
            ESP_LOGE(TAG, "[%s, %d] <%s> " format, __func__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__);           \
    }

/**
 * Macro which can be used to check the condition and ,
 * goto the label if condition satifed.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_ERROR_GOTO(con, lable, format, ...)                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        if (con)                                                                                                       \
        {                                                                                                              \
            if (*format != '\0')                                                                                       \
                ESP_LOGW(TAG, "[%s, %d]" format, __func__, __LINE__, ##__VA_ARGS__);                                   \
            goto lable;                                                                                                \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

/**
 * Macro which can be used to check the condition and ,
 * continue if condition satisfied.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_ERROR_CONTINUE(con, format, ...)                                                                           \
    {                                                                                                                  \
        if (con)                                                                                                       \
        {                                                                                                              \
            if (*format != '\0')                                                                                       \
                ESP_LOGW(TAG, "[%s, %d]: " format, __func__, __LINE__, ##__VA_ARGS__);                                 \
            continue;                                                                                                  \
        }                                                                                                              \
    }

/**
 * Macro which can be used to check the condition and ,
 * break from loop if condition satisfied.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_ERROR_BREAK(con, format, ...)                                                                              \
    {                                                                                                                  \
        if (con)                                                                                                       \
        {                                                                                                              \
            if (*format != '\0')                                                                                       \
                ESP_LOGW(TAG, "[%s, %d]: " format, __func__, __LINE__, ##__VA_ARGS__);                                 \
            break;                                                                                                     \
        }                                                                                                              \
    }

/**
 * Macro which can be used to check parameters.
 * Prints the error code, error location, and the failed statement to serial output.
 *
 */
#define ESP_PARAM_CHECK(con)                                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(con))                                                                                                    \
        {                                                                                                              \
            ESP_LOGE(TAG, "[%s, %d]: <ESP_ERR_INVALID_ARG> !(%s)", __func__, __LINE__, #con);                          \
            return ESP_ERR_INVALID_ARG;                                                                                \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#define ESP_NULL_CHECK(ptr)                                                                                            \
    {                                                                                                                  \
        if (ptr == NULL)                                                                                               \
        {                                                                                                              \
            size_t min_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INDICATE);                                    \
            ESP_LOGE(TAG, "[%s, %d] Failed to allocate. <Minimum free heap size: %u bytes> ",__func__, __LINE__,       \
                min_heap);                                                                                             \
            return ESP_ERR_NO_MEM;                                                                                     \
        }                                                                                                              \
    }

#ifdef __cplusplus
}
#endif
