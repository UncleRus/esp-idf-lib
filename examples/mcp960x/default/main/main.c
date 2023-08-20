#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp960x.h>
#include <string.h>

#define I2C_PORT 0
#define I2C_ADDR MCP960X_ADDR_DEFAULT
#define SENSORS_COUNT 4

/* 5 pins for 4 sensors */
static struct { gpio_num_t sda, scl; } pins[SENSORS_COUNT] = {
#if defined(CONFIG_IDF_TARGET_ESP8266)
        { 4, 5 },
        { 5, 2 },
        { 2, 14 },
        { 14, 12 }
#else
        { 23, 22 },
        { 22, 21 },
        { 21, 18 },
        { 18, 17 }
#endif
};

static const char * const snames[] = {
        [MCP960X_OK] = "OK",
        [MCP960X_OPEN_CIRCUIT] = "Open circuit",
        [MCP960X_SHORT_CIRCUIT] = "Short circuit",
};

static mcp960x_t sensors[SENSORS_COUNT];

void task(void *pvParameters)
{
    memset(sensors, 0, sizeof(sensors));
    for (size_t i = 0; i < SENSORS_COUNT; i++)
    {
        ESP_ERROR_CHECK(mcp960x_init_desc(sensors + i, I2C_ADDR, I2C_PORT, pins[i].sda, pins[i].scl));
        ESP_ERROR_CHECK(mcp960x_init(sensors + i));
        ESP_ERROR_CHECK(mcp960x_set_device_config(sensors + i, MCP960X_MODE_NORMAL, MCP960X_SAMPLES_1, MCP960X_ADC_RES_18, MCP960X_TC_RES_0_0625));
        ESP_ERROR_CHECK(mcp960x_set_sensor_config(sensors + i, MCP960X_TYPE_K, MCP960X_FILTER_MAX));
        printf("Sensor %u: SDA=%u, SCL=%u, HW ID 0x%02x, HW Rev. 0x%02x\n", i, pins[i].sda, pins[i].scl, sensors[i].id, sensors[i].revision);
    }

    bool ready;
    mcp960x_status_t status;
    float t;
    while (1)
    {
        for (size_t i = 0; i < SENSORS_COUNT; i++)
        {
            ESP_ERROR_CHECK(mcp960x_get_status(sensors + i, &ready, NULL, &status, NULL, NULL, NULL, NULL));
            if (!ready)
            {
                printf("%u: Not ready\n", i);
                continue;
            }

            ESP_ERROR_CHECK(mcp960x_get_thermocouple_temp(sensors + i, &t));
            printf("%u: %s, T: %.02f\n", i, snames[i], t);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

