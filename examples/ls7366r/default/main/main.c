#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <ls7366r.h>

#define TEST_PIN CONFIG_EXAMPLE_PIN_NUM_TEST
#define INTR_PIN CONFIG_EXAMPLE_PIN_NUM_INTR
#define GPIO_OUTPUT_PIN_SEL (1ULL << TEST_PIN)
#define GPIO_INPUT_PIN_SEL (1ULL << INTR_PIN)

#ifdef HSPI_HOST
#define LS7366R_HOST HSPI_HOST
#else
#define LS7366R_HOST SPI2_HOST
#endif

#define PIN_NUM_MISO CONFIG_EXAMPLE_PIN_NUM_MISO
#define PIN_NUM_MOSI CONFIG_EXAMPLE_PIN_NUM_MOSI
#define PIN_NUM_SCLK CONFIG_EXAMPLE_PIN_NUM_CLK
#define PIN_NUM_CS CONFIG_EXAMPLE_PIN_NUM_CS

static const char TAG[] = "main";

static QueueHandle_t gpio_evt_queue = NULL;

static ls7366r_config_t config = {
	.count_type =			LS7366R_NON_QUAD,
	.counter_bits =			LS7366R_32_BIT,
	.filter_clock_divider = LS7366R_FILTER_CLK_1,
	.counter_enable =		LS7366R_COUNTER_ENABLE,
	.index_mode =			LS7366R_INDEX_DISABLED,
	.index_sync =			LS7366R_INDEX_ASYNCHRONOUS,
	.flag_mode = {
		.borrow = LS7366R_FLAG_BORROW_DISABLE,
		.carry = LS7366R_FLAG_CARRY_DISABLE,
		.compare = LS7366R_FLAG_COMPARE_ENABLE,
		.index = LS7366R_FLAG_INDEX_DISABLE
	}
};


static void gpio_toggle_task(void* arg)
{
	for (;;)
	{
		gpio_set_level(TEST_PIN, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(TEST_PIN, 0);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		vTaskDelay(1);
	}
}


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


// Receives event from queue, prints the interrupt event
static void gpio_recv_task(void* arg)
{
	uint32_t io_num;
	for (;;)
	{
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
		{
			printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
		}

	}
}

void app_main(void)
{
	spi_bus_config_t cfg = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_SCLK,
		.quadhd_io_num = -1,
		.quadwp_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};

	ESP_ERROR_CHECK(spi_bus_initialize(LS7366R_HOST, &cfg, 0));
	ls7366r_t dev;
	ESP_ERROR_CHECK(ls7366r_init_desc(&dev, LS7366R_HOST, LS7366R_MAX_CLOCK_SPEED_HZ, PIN_NUM_CS));
	ESP_ERROR_CHECK(ls7366r_set_config(&dev, &config));

	// setup for gpio output (gpio_toggle_task)
	gpio_config_t io_conf = { };
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	// for interrupt input
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	xTaskCreate(gpio_recv_task, "gpio_recv_task", 2048, NULL, 10, NULL);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(INTR_PIN, gpio_isr_handler, (void*)INTR_PIN);

	xTaskCreate(gpio_toggle_task, "gpio_toggle_task", 4096, NULL, 5, NULL);

	ls7366r_set_compare_val(&dev, 250); // set the value for the compare flag to trigger as 250

	// While in regular loop print the count every 500ms
	while (1)
	{
		int32_t count;
		ls7366r_get_count(&dev, &count);
		ESP_LOGI(TAG, "Count: %" PRIi32, count);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
