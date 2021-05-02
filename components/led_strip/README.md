# RMT-based ESP-IDF driver for WS2812B/SK6812/APA106 LED strips

**WARNING!** If you try to use this driver simultaneously with Wi-Fi, you may
encounter RMT transmission bugs. To avoid them, simply initialize device
descriptor from the task bound to the second processor core.

Interrupt handlers assigned during the initialization of the RMT driver are
bound to the core on which the initialization took place. 

For example:

```C
static SemaphoreHandle_t mutex;

// this task will run on core 1
static void led_strip_init_task(void *arg)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(led_strip_init((led_strip_t *)arg));
    xSemaphoreGive(mutex);
    vTaskDelete(NULL);
}

// this function runs on core 0
void app_main()
{
	...
	
	// this function does not init RMT, so it can be called on any core
	led_strip_install();

	// fill descriptor
	led_strip_t strip = {	
    	.length = STRIP_LENGTH,
    	.gpio = STRIP_GPIO,
    	.type = LED_STRIP_WS2812,
    	.channel = RMT_CHANNEL_0,
#ifdef LED_STRIP_BRIGNTNESS
		.brightness = STRIP_BRIGHTNESS,
#endif
	}

	// Create mutex
    mutex = xSemaphoreCreateMutex();
    if (!mutex)
    {
        ESP_LOGE(TAG, "Could not create surface init task");
        ...
    }
	
	// Create led_strip_init_task with higher priority
    if (xTaskCreatePinnedToCore(led_strip_init_task, "led_strip_init", 2048, &strip, uxTaskPriorityGet(NULL) + 1, NULL, APP_CPU_NUM) != pdPASS)
    {
        ESP_LOGE(TAG, "Could not create surface init task");
        ...
    }
    
    // Switch to created task
    portYIELD();
    
    // Wait for init task done
    xSemaphoreTake(mutex, portMAX_DELAY);
    xSemaphoreGive(mutex);
	vSemaphoreDelete(mutex);
	
	...
	// here bug-free LED strip usage
}
```
