/*
 * Copyright (c) YYYY YOUR NAME HERE <user@your.dom.ain>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file example.h
 * @defgroup example example
 * @{
 *
 * An example component.
 *
 */

#if !defined(__EXAMPLE__H__)
#define __EXAMPLE__H__

#ifdef __cplusplus

/* in most cases, break before an opening brace, but do not in case of `extern
 * "C"`. otherwise, all the code would have been indented.
 */
extern "C" {
#endif

#include <esp_err.h>

/**
 * @brief An example function
 *
 * This is an example function in `example` component.
 *
 * @param foo An integer of something.
 * @param p A pointer to an integer.
 * @return `ESP_OK` on success.
 */
esp_err_t example_do_something(int foo, int *p);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __EXAMPLE__H__
