#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tps63101x.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *MAIN_LOG_TAG = "Main";

void read_control_1_register(i2c_dev_t* buckBoostConverter)
{
    ESP_LOGI(MAIN_LOG_TAG, "Reading control 1 register.");
    
    tps63101x_control_1_t control_1;

    esp_err_t err = tps63101x_get_control_1(buckBoostConverter, &control_1);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error reading control 1 register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, enable fast DVS: %d, enable SCP: %d, enable converter: %d", control_1.register_data.reg, control_1.data_fields.en_fast_dvs, control_1.data_fields.en_scp, control_1.data_fields.converter_en);
}

void write_control_1_register(i2c_dev_t* buckBoostConverter, tps63101x_control_1_t* control_1)
{
    ESP_LOGI(MAIN_LOG_TAG, "Writing control 1 register.");
    
    esp_err_t err = tps63101x_set_control_1(buckBoostConverter, control_1);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error writing control 1 register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }
}

void read_vout_register(i2c_dev_t* buckBoostConverter)
{
    ESP_LOGI(MAIN_LOG_TAG, "Reading vout register.");
    
    tps63101x_vout_t vout;

    esp_err_t err = tps63101x_get_vout(buckBoostConverter, &vout);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error reading vout register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(MAIN_LOG_TAG, "Result: %08x", vout.register_data.reg);
}

void write_vout_register(i2c_dev_t* buckBoostConverter, tps63101x_vout_t* vout)
{
    ESP_LOGI(MAIN_LOG_TAG, "Writing vout register.");
    
    esp_err_t err = tps63101x_set_vout(buckBoostConverter, vout);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error writing vout register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }
}

void read_control_2_register(i2c_dev_t* buckBoostConverter)
{
    ESP_LOGI(MAIN_LOG_TAG, "Reading control 2 register.");
    
    tps63101x_control_2_t control_2;

    esp_err_t err = tps63101x_get_control_2(buckBoostConverter, &control_2);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error reading control 2 register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(MAIN_LOG_TAG, "Result: %08x, force pwm: %d, fast ramp enable: %d, enable vout discharge: %d, cl ramp minimum: %d, ramp: %d", control_2.register_data.reg, control_2.data_fields.fpwm, control_2.data_fields.fast_ramp_en, control_2.data_fields.en_disch_vout, control_2.data_fields.cl_ramp_min, control_2.data_fields.td_ramp);
}

void write_control_2_register(i2c_dev_t* buckBoostConverter, tps63101x_control_2_t* control_2)
{
    ESP_LOGI(MAIN_LOG_TAG, "Writing control 2 register.");
    
    esp_err_t err = tps63101x_set_control_2(buckBoostConverter, control_2);

    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Error writing control 2 register: %s", esp_err_to_name(err));

        vTaskDelete(NULL);
        return;
    }
}

void task(void *pvParameters)
{
    i2c_dev_t buckBoostConverter;
    memset(&buckBoostConverter, 0, sizeof(i2c_dev_t));

    i2cdev_init();

    esp_err_t err = tps63101x_init_desc(&buckBoostConverter, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGI(MAIN_LOG_TAG, "Cannot init TPS63101X: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        
        return;
    }

    ESP_LOGI(MAIN_LOG_TAG, "TPS63101X initialized.");

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Reset to defaults
    ESP_LOGI(MAIN_LOG_TAG, "Reset to defaults.");

    tps63101x_reset(&buckBoostConverter);

    // Control 1 register
    read_control_1_register(&buckBoostConverter);

    vTaskDelay(pdMS_TO_TICKS(2000));
   
    // Vout register
    read_vout_register(&buckBoostConverter);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Control 2 register
    read_control_2_register(&buckBoostConverter);

    vTaskDelay(pdMS_TO_TICKS(2000));
   
    ESP_LOGI(MAIN_LOG_TAG, "Finished.");

    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
