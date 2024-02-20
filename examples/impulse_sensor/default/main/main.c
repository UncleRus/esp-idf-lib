/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <driver/gpio.h>
#include <impulse_sensor.h>

static const char *TAG = "APP";

static imp_sensor_t anemometer;

static imp_sensor_config_t config = {
    .input_pin = CONFIG_EXAMPLE_DATA_GPIO,
    .scale_factor = (1.75 / 20) //!< 1.75 m/s = 20 pps,
};

void app_main()
{
    float val;
    ESP_LOGI(TAG, "Anemometer test");
    ESP_ERROR_CHECK(imp_sensor_init(&config, &anemometer));

    while (1)
    {
        imp_sensor_get_value(anemometer, &val);
        ESP_LOGI(TAG, "Wind speed = %.2f m/s", val);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
