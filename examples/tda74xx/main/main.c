#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tda74xx.h>
#include <string.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 16
#define SCL_GPIO 17
#endif

#define INPUT 0 // 0..3
#define INPUT_GAIN 28 // 0dB..30dB
#define VOLUME 0 // -48dB..0dB
#define BASS 0 // -14dB..14dB, 2dB step
#define MIDDLE 0 // -14dB..14dB, 2dB step | comment this line out for TDA7440
#define TREBLE 0 // -14dB..14dB, 2dB step

void tda74xx_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    // init device descriptor
    ESP_ERROR_CHECK(tda74xx_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));

    // Disable attenuation
    ESP_ERROR_CHECK(tda74xx_set_speaker_attenuation(&dev, TDA74XX_CHANNEL_LEFT, 0));
    ESP_ERROR_CHECK(tda74xx_set_speaker_attenuation(&dev, TDA74XX_CHANNEL_RIGHT, 0));

    // Set input gain
    ESP_ERROR_CHECK(tda74xx_set_input_gain(&dev, INPUT_GAIN));

    // Switch input
    ESP_ERROR_CHECK(tda74xx_set_input(&dev, INPUT));

    // Set volume & eq
    ESP_ERROR_CHECK(tda74xx_set_volume(&dev, VOLUME));
    ESP_ERROR_CHECK(tda74xx_set_equalizer_gain(&dev, TDA74XX_BAND_BASS, BASS));
    ESP_ERROR_CHECK(tda74xx_set_equalizer_gain(&dev, TDA74XX_BAND_TREBLE, TREBLE));
#ifdef MIDDLE
    ESP_ERROR_CHECK(tda74xx_set_equalizer_gain(&dev, TDA74XX_BAND_MIDDLE, MIDDLE));
#endif

    int8_t vol = -48;
    while (1)
    {
        ESP_ERROR_CHECK(tda74xx_set_volume(&dev, vol));
        if (++vol > 0)
            vol = -48;
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(tda74xx_test, "tda74xx_test", 8192, NULL, 5, NULL);
}

