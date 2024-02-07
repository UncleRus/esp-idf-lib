/*
 * Copyright (c) 2022 Jaime Albuqueruqe <jaime.albq@gmail.com>
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

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>
#include <shift_reg.h>

void app_main(void)
{
	static shift_reg_config_t shifter = {
		.num_reg = 2,
		.mode = {
			.dir = SHIFT_DIR_OUTPUT,
			.bit_mode = SHIFT_BIT_MODE_MSB,
		},
		.pin = {
			.clk = GPIO_NUM_12,
			.data = GPIO_NUM_13,
			.latch = GPIO_NUM_15,
		},
	};


	ESP_ERROR_CHECK(shift_reg_init(&shifter));

	while (true) {
		uint8_t value [2];
		for (uint16_t i = 0; i < 0xFFFF; i++) {
			value[0] = (uint8_t) (i & 0xFF);
			value[1] = (uint8_t) ((i >> 8) & 0xFF);
			ESP_ERROR_CHECK(shift_reg_send(&shifter, value, 2));
			ESP_ERROR_CHECK(shift_reg_latch(&shifter));
			ESP_LOGI(__func__, "reg_value: 0x%02X 0x%02X", shifter.reg_value[0], shifter.reg_value[1]);
			vTaskDelay(2 / pdMS_TO_TICKS);
		}
	}
}
