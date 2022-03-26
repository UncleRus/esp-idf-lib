#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <mhz19b.h>

#define TAG "MHZ19BDEMO"

void app_main(void)
{
	int16_t co2;
	mhz19b_dev_t dev;
	char version[6];
	uint16_t range;
	bool autocal;

	ESP_ERROR_CHECK(mhz19b_init(&dev, UART_NUM_1, CONFIG_EXAMPLE_UART_TX, CONFIG_EXAMPLE_UART_RX));

	while (!mhz19b_detect(&dev))
	{
		ESP_LOGI(TAG, "MHZ-19B not detected, waiting...");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	mhz19b_get_version(&dev, version);
	ESP_LOGI(TAG, "MHZ-19B firmware version: %s", version);
	ESP_LOGI(TAG, "MHZ-19B set range and autocal");

	mhz19b_set_range(&dev, MHZ19B_RANGE_5000);
	mhz19b_set_auto_calibration(&dev, false);

	mhz19b_get_range(&dev, &range);
	ESP_LOGI(TAG, "  range: %d", range);

	mhz19b_get_auto_calibration(&dev, &autocal);
	ESP_LOGI(TAG, "  autocal: %s", autocal ? "ON" : "OFF");

	while (mhz19b_is_warming_up(&dev, true))  // use smart warming up detection
	{
		ESP_LOGI(TAG, "MHZ-19B is warming up");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

    while (1) {
		mhz19b_read_co2(&dev, &co2);
		ESP_LOGI(TAG, "CO2: %d", co2);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
