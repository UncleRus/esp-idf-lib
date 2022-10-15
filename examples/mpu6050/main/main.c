/* standard C library */
#include <stdio.h>

/* freeRTOS library */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu6050.h"
#include <esp_err.h>
#include <esp_log.h>


// #ifdef CONFIG_EXAMPLE_MPU6050_ADDRESS_LOW
// #define MPU6050_ADDR (MPU6050_ADDRESS_LOW)
// #endif
// #ifdef CONFIG_EXAMPLE_MPU6050_ADDRESS_HIGH
// #define MPU6050_ADDR (MPU6050_ADDRESS_HIGH)
// #endif
// #define MPU6050_SDA_PIN       (CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SDA)
// #define MPU6050_SCL_PIN       (CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SCL)
// #define MPU6050_I2C_CLK_FREQZ (CONFIG_EXAMPLE_MPU6050_I2C_CLOCK_HZ)

#define TAG "mpu6050_test"

void mpu6050_test(void *pvParameters)
{
    // esp_err_t ret = ESP_FAIL;
    // mpu6050_config_t mpu6050_config;
    // mpu6050_config.cfg.mode = I2C_NUM_0;
    // mpu6050_config.cfg.sda_io_num = MPU6050_SDA_PIN;
    // mpu6050_config.cfg.scl_io_num = MPU6050_SCL_PIN;
    // mpu6050_config.cfg.master.clk_speed = MPU6050_I2C_CLK_FREQZ;
    // mpu6050_config.addr = MPU6050_ADDR;

    // ESP_LOGI(TAG, "sda %d, scl %d clk freq %u addr 0x%x", MPU6050_SDA_PIN, MPU6050_SCL_PIN,
    //     mpu6050_config.cfg.master.clk_speed, mpu6050_config.addr);

    // while (1)
    // {
    //     ret = mpu6050_init(&(mpu6050_config));
    //     ESP_ERROR_BREAK(ret == ESP_OK, "mpu6050 initilized.");
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // ESP_LOGI(TAG, "Testing connection...");
    // ESP_LOGI(TAG, "%s",
    //     mpu6050_test_connection(&(mpu6050_config)) ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // // use the code below to change accel/gyro offset values

    // ESP_LOGI(TAG, "Reading internal sensor offsets...");
    // ESP_LOGI(TAG, "accel x offset: %d", mpu6050_get_x_accel_offset(&(mpu6050_config)));
    // ESP_LOGI(TAG, "accel y offset: %d", mpu6050_get_y_accel_offset(&(mpu6050_config)));
    // ESP_LOGI(TAG, "accel z offset: %d", mpu6050_get_z_accel_offset(&(mpu6050_config)));

    // ESP_LOGI(TAG, "gyro x offset: %d", mpu6050_get_x_gyro_offset(&(mpu6050_config)));
    // ESP_LOGI(TAG, "gyro y offset: %d", mpu6050_get_y_gyro_offset(&(mpu6050_config)));
    // ESP_LOGI(TAG, "gyro z offset: %d", mpu6050_get_z_gyro_offset(&(mpu6050_config)));
    // /*
    // ESP_LOGI(TAG, "Updating internal sensor offsets...");
    // mpu6050_set_x_gyro_offset(&(mpu6050_config), 220);
    // mpu6050_set_y_gyro_offset(&(mpu6050_config), 76);
    // mpu6050_set_z_gyro_offset(&(mpu6050_config), -85);
    // */

    // if(mpu6050_get_temp_sensor_enabled(&(mpu6050_config)) != ESP_OK)
    // {
    //     ESP_LOGI(TAG, "enable sensor temp");
    //     mpu6050_set_temp_sensor_enabled(&(mpu6050_config), true);
    // }
    // mpu6050_acceleration_t data_accel = { 0 };
    // mpu6050_rotation_t data_rotat = { 0 };
    // while (1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(5000));

    //     mpu6050_get_motion(&(mpu6050_config), &(data_accel), &(data_rotat));
    //     ESP_LOGI(TAG, "Acceleration: x: <%d> \t y: <%d> \t z: <%d>", data_accel.accel_x, data_accel.accel_y,
    //         data_accel.accel_z);
    //     ESP_LOGI(TAG, "    Rotation: x: <%d> \t y: <%d> \t z: <%d>", data_rotat.gyro_x, data_rotat.gyro_y,
    //     data_rotat.gyro_z); ESP_LOGI(TAG, "temperature %d", mpu6050_get_temperature(&(mpu6050_config)));
    // }
    // vTaskDelete(NULL);
    return;
}

void app_main()
{
    // xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
