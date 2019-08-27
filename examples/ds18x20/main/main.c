#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds18x20.h>

#if defined(CONFIG_IDF_TARGET_ESP8266)
static const gpio_num_t SENSOR_GPIO = 4;
#else
static const gpio_num_t SENSOR_GPIO = 17;
#endif
static const uint32_t LOOP_DELAY_MS = 250;
static const int MAX_SENSORS = 8;
static const int RESCAN_INTERVAL = 8;

void ds18x20_test(void *pvParameter)
{
    ds18x20_addr_t addrs[MAX_SENSORS];
    float temps[MAX_SENSORS];
    int sensor_count;

    // There is no special initialization required before using the ds18x20
    // routines.  However, we make sure that the internal pull-up resistor is
    // enabled on the GPIO pin so that one can connect up a sensor without
    // needing an external pull-up (Note: The internal (~47k) pull-ups of the
    // ESP8266 do appear to work, at least for simple setups (one or two sensors
    // connected with short leads), but do not technically meet the pull-up
    // requirements from the ds18x20 datasheet and may not always be reliable.
    // For a real application, a proper 4.7k external pull-up resistor is
    // recommended instead!)

    while (1)
    {
        // Every RESCAN_INTERVAL samples, check to see if the sensors connected
        // to our bus have changed.
        sensor_count = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS);

        if (sensor_count < 1)
            printf("No sensors detected!\n");
        else
        {
            printf("%d sensors detected:\n", sensor_count);
            // If there were more sensors found than we have space to handle,
            // just report the first MAX_SENSORS..
            if (sensor_count > MAX_SENSORS)
                sensor_count = MAX_SENSORS;

            // Do a number of temperature samples, and print the results.
            for (int i = 0; i < RESCAN_INTERVAL; i++)
            {
                ds18x20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
                for (int j = 0; j < sensor_count; j++)
                {
                    // The ds18x20 address is a 64-bit integer, but newlib-nano
                    // printf does not support printing 64-bit values, so we
                    // split it up into two 32-bit integers and print them
                    // back-to-back to make it look like one big hex number.
                    uint32_t addr0 = addrs[j] >> 32;
                    uint32_t addr1 = addrs[j];
                    float temp_c = temps[j];
                    float temp_f = (temp_c * 1.8) + 32;
                    /* float is used in printf(). you need non-default configuration in
                     * sdkconfig for ESP8266, which is enabled by default for this
                     * example. see sdkconfig.defaults.esp8266
                     */
                    printf("  Sensor %08x%08x reports %f deg C (%f deg F)\n", addr0, addr1, temp_c, temp_f);
                }
                printf("\n");

                // Wait for a little bit between each sample (note that the
                // ds18x20_measure_and_read_multi operation already takes at
                // least 750ms to run, so this is on top of that delay).
                vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
            }
        }
    }
}

void app_main()
{
    xTaskCreate(ds18x20_test, "ds18x20_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
}

