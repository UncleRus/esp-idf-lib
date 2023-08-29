#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <l3gx.h>

#define L3Gx_GYRO_I2C_ADDR 0x69

void gyro_test(void *pvParameters)
{
	l3gx_t l3gx_gyro;
	l3gx_raw_data_t raw_data;
	l3gx_data_t gyro_data;
	bool gyro_ready;

	ESP_ERROR_CHECK(l3gx_init_desc(&l3gx_gyro,L3Gx_GYRO_I2C_ADDR,0,CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
	ESP_ERROR_CHECK(l3gx_init(&l3gx_gyro));
	
	/* OPTIONAL */
	ESP_ERROR_CHECK(l3gd20_set_scale(&l3gx_gyro,L3GX_SCALE_500));
	ESP_ERROR_CHECK(l3gd20_set_datarate_and_bandwith(&l3gx_gyro,L3GX_DRBW_800_30));

    while (1)
    {
		if (l3gx_data_ready(&l3gx_gyro,&gyro_ready) == ESP_OK){
			if (l3gx_get_raw_data(&l3gx_gyro, &raw_data) == ESP_OK){
				l3gd20_raw_to_dps(&l3gx_gyro, &raw_data, &gyro_data);
				
				printf("Gyro data: raw[x=%6d y=%6d z=%6d] x=%.2fdps y=%.2fdps z=%.2fdps\n", 
				    raw_data.x, raw_data.y, raw_data.z,
				    gyro_data.x, gyro_data.y, gyro_data.z);
			} else {
				printf("Could not read data from sensor\n");
			}
		}
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(gyro_test, "gyro_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

