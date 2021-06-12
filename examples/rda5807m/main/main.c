#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rda5807m.h>
#include <string.h>
#include <esp_log.h>

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

static const char *TAG = "rda5807m-example";

static const char *states[] = {
    [RDA5807M_SEEK_NONE]     = "---",
    [RDA5807M_SEEK_STARTED]  = ">>>",
    [RDA5807M_SEEK_COMPLETE] = "***",
    [RDA5807M_SEEK_FAILED]   = "...",
};

static rda5807m_t dev = { 0 };

static void display(void *pvParameters)
{
    rda5807m_state_t state;
    while (1)
    {
        ESP_ERROR_CHECK(rda5807m_get_state(&dev, &state));
        ESP_LOGI(TAG, "[ %3d.%d MHz ] [ %s ] [ %c ] [ %6s ] [ %3s ] [ RSSI: %3d ]",
                 state.frequency / 1000, (state.frequency % 1000) / 100,
                 states[state.seek_status],
                 state.station ? 'S' : ' ',
                 state.stereo ? "Stereo" : "Mono",
                 state.rds_ready ? "RDS" : "",
                 state.rssi);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Main task
static void test(void *pvParameters)
{
    dev.i2c_dev.cfg.scl_pullup_en = true;
    dev.i2c_dev.cfg.sda_pullup_en = true;

    ESP_ERROR_CHECK(rda5807m_init_desc(&dev, I2C_PORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(rda5807m_init(&dev, RDA5807M_CLK_32768HZ));

    xTaskCreatePinnedToCore(display, "display", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL, APP_CPU_NUM);
    rda5807m_state_t state;

    while (1)
    {
        ESP_LOGI(TAG, "Start seeking...");

        ESP_ERROR_CHECK(rda5807m_seek_start(&dev, true, true, RDA5807M_SEEK_TH_DEF));
        memset(&state, 0, sizeof(state));

        while (state.seek_status != RDA5807M_SEEK_COMPLETE && !state.station)
        {
            rda5807m_get_state(&dev, &state);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_ERROR_CHECK(rda5807m_seek_stop(&dev));
        ESP_LOGI(TAG, "Found station");

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main()
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Start task
    xTaskCreatePinnedToCore(test, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

