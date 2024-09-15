#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>
#include <stdio.h>

void task(void *ignore)
{
    i2c_dev_t dev = { 0 };
#if CONFIG_I2CDEV_USING_LEGACY_I2C
    dev.cfg.sda_io_num = CONFIG_EXAMPLE_I2C_MASTER_SDA;
    dev.cfg.scl_io_num = CONFIG_EXAMPLE_I2C_MASTER_SCL;
#else
    dev.master_bus_config.sda_io_num = CONFIG_EXAMPLE_I2C_MASTER_SDA;
    dev.master_bus_config.scl_io_num = CONFIG_EXAMPLE_I2C_MASTER_SCL;
    dev.master_bus_config.i2c_port = -1;
    dev.master_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    dev.master_bus_config.intr_priority = 0;
    dev.master_bus_config.glitch_ignore_cnt = 7;
    dev.master_bus_config.trans_queue_depth = 0;
    dev.master_bus_config.flags.enable_internal_pullup = false;
#endif
#if HELPER_TARGET_IS_ESP32 && CONFIG_I2CDEV_USING_LEGACY_I2C
    dev.cfg.master.clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_HZ; // 100kHz
#endif
    while (1)
    {
        esp_err_t res;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (uint8_t addr = 3; addr < 0x78; addr++)
        {
            if (addr % 16 == 0)
                printf("\n%.2x:", addr);

            dev.addr = addr;
            res = i2c_dev_probe(&dev, I2C_DEV_WRITE);

            if (res == 0)
                printf(" %.2x", addr);
            else
                printf(" --");
        }
        printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());
    // Start task
    xTaskCreate(task, "i2c_scanner", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
