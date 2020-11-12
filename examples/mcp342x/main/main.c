#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp342x.h>
#include <string.h>

#define I2C_PORT 0
#define ADDR MCP342X_ADDR_MIN

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define GAIN  MCP342X_GAIN1       // +-2.048
#define CHANNEL MCP342X_CHANNEL1
#define RESOLUTION MCP342X_RES_16 // 16-bit, 15 sps

static mcp342x_t adc;

static void task(void *arg)
{
    // Clear device descriptor
    memset(&adc, 0, sizeof(adc));

    ESP_ERROR_CHECK(mcp342x_init_desc(&adc, I2C_PORT, ADDR, SDA_GPIO, SCL_GPIO));

    adc.channel = CHANNEL;
    adc.gain = GAIN;
    adc.resolution = RESOLUTION;
    adc.mode = MCP342X_CONTINUOUS;

    uint32_t wait_time;
    ESP_ERROR_CHECK(mcp342x_get_sample_time_us(&adc, &wait_time)); // microseconds
    wait_time = wait_time / 1000 + 1; // milliseconds

    // start first conversion
    ESP_ERROR_CHECK(mcp342x_start_conversion(&adc));

    while (1)
    {
        // Wait for conversion
        vTaskDelay(pdMS_TO_TICKS(wait_time));

        // Read data
        float volts;
        ESP_ERROR_CHECK(mcp342x_get_voltage(&adc, &volts, NULL));
        printf("Channel: %d, voltage: %0.4f\n", adc.channel, volts);
    }
}

void app_main()
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Start task
    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

