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

/*
 * this file is supposed to conform the code style.
 */

/* inculde external headers. sort header files by name. if the order matters,
 * create another header block separated by an empty line. */
#include <freertos/FreeRTOS.h>

#include <esp_err.h>
#include <esp_log.h>

/* insert an empty line after external header files. use double-quotes for
 * local header file name */
#include "example.h"

#define USE_UPPER_CASE_FOR_MACRO (1)

/* do NOT use upper case for variables. use `snake_case` instead of
 * `CamelCase`. */
static char *tag = "example";

/* prefix public function names with the component name, `example_` in this
 * case. */
esp_err_t example_do_something(int foo, int *p)
{
    /* indent with four spaces, not a tab */

    /* declare variables at the beginning of the function, not in the middle
     * of code */
    esp_err_t err; // an inline comment should use `//`, not `/* *`/

    if (foo == 0)
    {
        ESP_LOGE(tag, "example(): argument must not be zero");
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if (foo > 0)
    {

        /* do something */
        *p = foo;
    }
    else
    {

        /* do other thing */
        p = NULL;
    }

    switch (foo)
    {
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }

    err = ESP_OK;
fail:
    return err;
}
