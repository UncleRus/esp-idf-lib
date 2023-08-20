#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include <sgm58031.h>

#define I2C_PORT 0

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define GAIN SGM58031_GAIN_2V048 // +-2.048V

static const char *TAG = "single-shot-example";
// I2C addresses
static const uint8_t addr = CONFIG_EXAMPLE_I2C_DEVICE_ADDRESS;

// Descriptors
static i2c_dev_t device;

// Gain value
static float gain_val;

// Main task
void sgm58031_test(void *pvParameters)
{
    gain_val = sgm58031_gain_values[GAIN];

    // Setup ICs

    ESP_ERROR_CHECK(
        sgm58031_init_desc(&device, addr, I2C_PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    uint8_t id, version;
    ESP_ERROR_CHECK(sgm58031_get_chip_id(&device, &id, &version));
    ESP_LOGI(TAG, "Device - Addr 0x%02x\tID: %d\tVersion: %d", addr, id, version);

    ESP_ERROR_CHECK(sgm58031_set_data_rate(&device, SGM58031_DATA_RATE_800)); // 25 samples per second
    ESP_ERROR_CHECK(sgm58031_set_gain(&device, GAIN));
    ESP_ERROR_CHECK(sgm58031_set_conv_mode(&device, SGM58031_CONV_MODE_SINGLE_SHOT)); // Single-shot conversion mode

    while (1)
    {
        for (uint8_t i = 0; i < 4; ++i)
        {
            ESP_ERROR_CHECK(sgm58031_set_input_mux(&device, SGM58031_MUX_AIN0_GND + i)); // positive = AINx, negative = GND

            if (sgm58031_start_conversion(&device) == ESP_OK)
            {
                bool busy = true;

                while (busy)
                {
                    sgm58031_is_busy(&device, &busy);
                }

                // Read result
                int16_t raw = 0;
                if (sgm58031_get_value(&device, &raw) == ESP_OK)
                {
                    float voltage = gain_val / SGM58031_MAX_VALUE * raw;
                    ESP_LOGI(TAG, "[0x%02x] AIN%d Raw ADC value: %d, voltage: %.04f volts", addr, i, raw, voltage);
                }
                else
                {
                    ESP_LOGE(TAG, "[0x%02x] Cannot read ADC value on AIN%d", addr, i);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main()
{
    // Init library
    ESP_ERROR_CHECK(i2cdev_init());

    // Clear device descriptors
    memset(&device, 0, sizeof(device));

    // Start task
    xTaskCreatePinnedToCore(sgm58031_test, "sgm58031_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
