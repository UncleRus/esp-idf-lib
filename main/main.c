#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_system.h"

void task(void *pvParameter)
{
    while(true)
    {
        printf("Hello world! Free heap: %d kB\n", xPortGetFreeHeapSize());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(task, "main_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}

