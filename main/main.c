#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ds18x20.h>
#include <hmc5883l.h>
#include <dht.h>
#include <ds1307.h>
#include <ds3231.h>


void ds18x20_test(void *pvParameter)
{
    static const gpio_num_t SENSOR_GPIO = 17;
    static const uint32_t LOOP_DELAY_MS = 250;
    static const int MAX_SENSORS = 8;
    static const int RESCAN_INTERVAL = 8;

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

void hmc5883l_test(void *pvParameters)
{
    while (hmc5883l_i2c_init(0, 17, 16) != ESP_OK)
    {
        printf("Could not init I2C bus\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    while (hmc5883l_init(0) != ESP_OK)
    {
        printf("HMC5883L not found\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    hmc5883l_set_opmode(0, HMC5883L_MODE_CONTINUOUS);
    hmc5883l_set_samples_averaged(0, HMC5883L_SAMPLES_8);
    hmc5883l_set_data_rate(0, HMC5883L_DATA_RATE_07_50);
    hmc5883l_set_gain(0, HMC5883L_GAIN_1090);

    while (1)
    {
        hmc5883l_data_t data;
        if (hmc5883l_get_data(0, &data) == ESP_OK)
            printf("Magnetic data: X:%.2f mG, Y:%.2f mG, Z:%.2f mG\n", data.x, data.y, data.z);
        else
            printf("Could not read HMC5883L data\n");

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void dht_test(void *pvParameters)
{
    static const dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
    static const gpio_num_t dht_gpio = 17;

    int16_t temperature = 0;
    int16_t humidity = 0;

    // DHT sensors that come mounted on a PCB generally have
    // pull-up resistors on the data pin.  It is recommended
    // to provide an external pull-up resistor otherwise...

    while (1)
    {
        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10);
        else
            printf("Could not read data from sensor\n");

        // Three second delay...
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void ds1307_test(void *pvParameters)
{
    static const i2c_port_t bus = 0;

    while (ds1307_i2c_init(bus, 17, 16) != ESP_OK)
    {
        printf("Could not init I2C bus\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    // setup datetime: 2016-10-09 13:50:10
    struct tm time = {
        .tm_year = 2016,
        .tm_mon  = 9,  // 0-based
        .tm_mday = 9,
        .tm_hour = 13,
        .tm_min  = 50,
        .tm_sec  = 10
    };
    ds1307_set_time(bus, &time);

    while (1)
    {
        ds1307_get_time(bus, &time);

        printf("%04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void ds3231_test(void *pvParameters)
{
    static const i2c_port_t bus = 0;

    while (ds3231_i2c_init(bus, 17, 16) != ESP_OK)
    {
        printf("Could not init I2C bus\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    // setup datetime: 2016-10-09 13:50:10
    struct tm time = {
        .tm_year = 2016,
        .tm_mon  = 9,  // 0-based
        .tm_mday = 9,
        .tm_hour = 13,
        .tm_min  = 50,
        .tm_sec  = 10
    };
    ds3231_set_time(bus, &time);

    while (1)
    {
        float temp;

        ds3231_get_temp_float(bus, &temp);
        ds3231_get_time(bus, &time);

        printf("%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel\n", time.tm_year, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, temp);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    //xTaskCreate(ds18x20_test, "ds18x20_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    //xTaskCreate(hmc5883l_test, "hmc5883l_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    //xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    //xTaskCreate(ds1307_test, "ds1307_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(ds1307_test, "ds3231_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

