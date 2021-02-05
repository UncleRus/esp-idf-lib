#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ads111x.h>
#include <string.h>

#define DEV_COUNT 2   // 2 ICs

#define I2C_PORT 0

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

#define GAIN ADS111X_GAIN_4V096 // +-4.096V

// I2C addresses
static const uint8_t addr[DEV_COUNT] = {
    ADS111X_ADDR_GND,
    ADS111X_ADDR_VCC
};

// Descriptors
static i2c_dev_t devices[DEV_COUNT];

// Gain value
static float gain_val;

static void measure(size_t n)
{
    // wait for conversion end
    bool busy;
    do
    {
        ads111x_is_busy(&devices[n], &busy);
    }
    while (busy);

    // Read result
    int16_t raw = 0;
    if (ads111x_get_value(&devices[n], &raw) == ESP_OK)
    {
        float voltage = gain_val / ADS111X_MAX_VALUE * raw;
        printf("[%u] Raw ADC value: %d, voltage: %.04f volts\n", n, raw, voltage);
    }
    else
        printf("[%u] Cannot read ADC value\n", n);
}

// Main task
void ads111x_test(void *pvParameters)
{
    gain_val = ads111x_gain_values[GAIN];

    // Setup ICs
    for (size_t i = 0; i < DEV_COUNT; i++)
    {
        ESP_ERROR_CHECK(ads111x_init_desc(&devices[i], addr[i], I2C_PORT, SDA_GPIO, SCL_GPIO));

        ESP_ERROR_CHECK(ads111x_set_mode(&devices[i], ADS111X_MODE_CONTINUOUS));    // Continuous conversion mode
        ESP_ERROR_CHECK(ads111x_set_data_rate(&devices[i], ADS111X_DATA_RATE_32)); // 32 samples per second
        ESP_ERROR_CHECK(ads111x_set_input_mux(&devices[i], ADS111X_MUX_0_GND));    // positive = AIN0, negative = GND
        ESP_ERROR_CHECK(ads111x_set_gain(&devices[i], GAIN));
    }

    while (1)
    {
        for (size_t i = 0; i < DEV_COUNT; i++)
            measure(i);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Clear device descriptors
    memset(devices, 0, sizeof(devices));

    // Start task
    xTaskCreatePinnedToCore(ads111x_test, "ads111x_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

