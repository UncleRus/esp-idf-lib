
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_utils.h>
#include <ds2438.h>
#include <sdkconfig.h>

#define TAG "ds2438_test"

#define FARENHEIT(c) ((float)(c * 9.0 / 5.0 + 32.0))
/* ------------------------------------------------*/
void ds2438_humdity_test(void *pvParameters)
{
    ds2438_dev_t *dev = (ds2438_dev_t *)pvParameters;
    esp_err_t ret = ESP_OK;
    double temperature;
    float heatindex;
    float dewpoint;
    float humidity;
    float voltA, voltB;
    uint8_t id[8];
    ds2438_get_uniqueID((uint8_t *)id);
    ESP_LOG_BUFFER_HEXDUMP("uniqueID", (uint8_t *)id, sizeof(id), ESP_LOG_INFO);

    ds2438_data_t data = {0};
    ret = ds2438_update(dev, &(data));
    ESP_ERROR_GOTO(ret != ESP_OK, EXIT, "failed to read the ds2438 device.");

    ESP_LOGI(TAG, " temp: %f volt: %f", data.temperatue, data.voltage);

    while (true)
    {
        ds2438_get_temperature(dev, &(temperature));
        heatindex = temperature;
        ds2438_get_voltage(dev, &(voltA));

        dev->mode = DS2438_MODE_CHB;
        ds2438_get_voltage(dev, &(voltB));

        float rh = (voltA / voltB - 0.16) / 0.0062;
        humidity = (float)(rh / (1.0546 - 0.00216 * temperature));
        if (humidity < 0.0)
        {
            humidity = 0.0;
        }
        else if (humidity > 100.0)
        {
            humidity = 100.0;
        }
        float tempK = temperature + 273.15;
        dewpoint = tempK / ((-0.0001846 * log(humidity / 100.0) * tempK) + 1.0) - 273.15;
        if ((temperature >= 26.7) && (humidity >= 40.0))
        {
            float t = FARENHEIT(temperature); // heat index formula assumes degF
            rh = humidity;
            float heatindexF = -42.38 + 2.049 * t + 10.14 * rh + -0.2248 * t * rh + -0.006838 * t * t + -0.05482 * rh * rh + 0.001228 * t * t * rh + 0.0008528 * t * rh * rh + -0.00000199 * t * t * rh * rh;
            heatindex = (heatindexF - 32.0) * 5.0 / 9.0;
        }
        if (heatindex < temperature)
        {
            heatindex = temperature;
        }

        ESP_LOGI(TAG, "Temperature %f C", temperature);
        ESP_LOGI(TAG, "Temperature %f F", FARENHEIT(temperature));
        ESP_LOGI(TAG, "Heat Index %f C", heatindex);
        ESP_LOGI(TAG, "Heat Index %f F", FARENHEIT(heatindex));
        ESP_LOGI(TAG, "Dewpoint %f C", dewpoint);
        ESP_LOGI(TAG, "Dewpoint %f F", FARENHEIT(dewpoint));
        ESP_LOGI(TAG, "Relative Humidity %f %% C", humidity);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

EXIT:
    vTaskDelete(NULL);
}

/* ------------------------------------------------*/
void ds2438_test(void *pvParameters)
{
    esp_err_t ret = ESP_OK;
    ds2438_dev_t *dev = (ds2438_dev_t *)pvParameters;

    ds2438_data_t data = {0};

    ret = ds2438_update(dev, &(data));
    ESP_ERROR_GOTO(ret != ESP_OK, EXIT, "failed to update the ds2438 device.");
    uint8_t id[8];
    ds2438_get_uniqueID((uint8_t *)id);
    ESP_LOG_BUFFER_HEXDUMP("uniqueID", (uint8_t *)id, sizeof(id), ESP_LOG_INFO);

    while (true)
    {
        ESP_LOGI(TAG, "Temperature %lf", data.temperatue);
        ESP_LOGI(TAG, "voltage A %f", data.voltage);

        if (dev->mode == DS2438_MODE_CHB)
        {
            ESP_LOGI(TAG, "voltage B %f", data.voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

EXIT:
    vTaskDelete(NULL);
}

/* ------------------------------------------------*/
void app_main()
{
    esp_err_t ret = ESP_OK;
    ds2438_dev_t ds2438_config = {
        .mode = DS2438_MODE_CHA,
        .onewire_pin = CONFIG_EXAMPLE_DS2438_ONEWIRE_PIN,
    };

    do
    {
        ret = ds2438_init(&(ds2438_config));
        ESP_ERROR_PRINT(ret != ESP_OK, ret, "Please check the connection...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (ret != ESP_OK);

    xTaskCreate(ds2438_test, "ds2438_test", 1024 * 4, &(ds2438_config), 5, NULL);
    // xTaskCreate(ds2438_humdity_test, "ds2438_humdity_test", 1024 * 4, &(ds2438_config), 5, NULL);
}

/* ------------------------------------------------*/