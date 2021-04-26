# Driver for the MH-Z19B NDIR CO2 sensor

This driver is heavily inspired from [Erriez MH-Z19B CO2 sensor library for
Arduino](https://github.com/Erriez/ErriezMHZ19B).

It uses an UART serial port to communicate with the sensor. Since the UART must
configured a specific way (9600 8N1), the `mhz19b_init()` takes care of configuring it.
It will however not start detecting/reading from the sensor. This must be done. Typical
usage would be:


```C
[...]
#include "esp_log.h"
#include "mhz19b.h"

#define MHZ19B_TX 12
#define MHZ19B_RX 13

void app_main(void)
{
	int16_t co2;
	mhz19b_dev_t dev;
	char version[6];
	uint16_t range;
	bool autocal;

	mhz19b_init(&dev, UART_NUM_1, MHZ19B_TX, MHZ19B_RX);

	while (!mhz19b_detect(&dev))
	{
		ESP_LOGI(TAG, "MHZ-19B not detected, waiting...");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

	mhz19b_get_version(&dev, version, 5);
	ESP_LOGI(TAG, "MHZ-19B firmware version: %s", version);
	ESP_LOGI(TAG, "MHZ-19B set range and autocal");

	mhz19b_set_range(&dev, MHZ19B_RANGE_5000);
	mhz19b_set_auto_calibration(&dev, false);

	mhz19b_get_range(&dev, &range);
	ESP_LOGI(TAG, "  range: %d", range);

	mhz19b_get_auto_calibration(&dev, &autocal);
	ESP_LOGI(TAG, "  autocal: %s", autocal ? "ON" : "OFF");

	while (mhz19b_is_warming_up(&dev))
	{
		ESP_LOGI(TAG, "MHZ-19B is warming up");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

    while (1) {
		mhz19b_read_CO2(&dev, &co2);
		ESP_LOGI(TAG, "CO2: %d", co2);
		vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

```
