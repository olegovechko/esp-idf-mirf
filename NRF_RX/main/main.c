/*	Mirf Example

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "mirf.h"

static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t gpio_trigger_queue = NULL;

#define GPIO_INPUT_PIN_SEL (1ULL<<CONFIG_IRQ_GPIO)
#define ESP_INTR_FLAG_DEFAULT 0

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR gpio_trigger_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_trigger_queue, &gpio_num, NULL);
}

/*
void gpio(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint32_t io_num;
	while (1) {
		if(xQueuePeek(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			ESP_LOGW(pcTaskGetName(NULL), "GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
		} else {
			ESP_LOGE(pcTaskGetName(NULL), "xQueuePeek fail");
			break;
		}
	}
	vTaskDelete( NULL );
}
*/

#if CONFIG_ADVANCED
void AdvancedSettings(NRF24_t * dev)
{
#if CONFIG_RF_RATIO_2M
	ESP_LOGW(pcTaskGetName(NULL), "Set RF Data Ratio to 2MBps");
	Nrf24_SetSpeedDataRates(dev, 1);
#endif // CONFIG_RF_RATIO_2M

#if CONFIG_RF_RATIO_1M
	ESP_LOGW(pcTaskGetName(NULL), "Set RF Data Ratio to 1MBps");
	Nrf24_SetSpeedDataRates(dev, 0);
#endif // CONFIG_RF_RATIO_2M

#if CONFIG_RF_RATIO_250K
	ESP_LOGW(pcTaskGetName(NULL), "Set RF Data Ratio to 250KBps");
	Nrf24_SetSpeedDataRates(dev, 2);
#endif // CONFIG_RF_RATIO_2M

	ESP_LOGW(pcTaskGetName(NULL), "CONFIG_RETRANSMIT_DELAY=%d", CONFIG_RETRANSMIT_DELAY);
	Nrf24_setRetransmitDelay(dev, CONFIG_RETRANSMIT_DELAY);
}
#endif // CONFIG_ADVANCED

#if CONFIG_RECEIVER

bool triggerPressed = false;
uint8_t checkBuff[4][2] = {"1\0", "2\0", "3\0", "4\0"};
int checkID = 0;

void trigger(void *pvParameters)
{
    gpio_reset_pin(CONFIG_TRIGGER_GPIO);

	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = (1ULL << CONFIG_TRIGGER_GPIO);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//create a queue to handle gpio event from isr
	gpio_trigger_queue = xQueueCreate(10, sizeof(uint32_t));

	//install gpio isr service
	//gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(CONFIG_TRIGGER_GPIO, gpio_trigger_handler, (void*) CONFIG_TRIGGER_GPIO);

	uint32_t io_num;

	while(1) {
		// Wait for assertion of RX receive complete(RX_DR)
		if(xQueueReceive(gpio_trigger_queue, &io_num, 0)) 
		{
			while (xQueueReceive(gpio_trigger_queue, &io_num, 0))
				;
			vTaskDelay(15);	// debounce
			bool pressed = gpio_get_level(CONFIG_TRIGGER_GPIO) == 0;
			if (pressed != triggerPressed)
			{
				triggerPressed = pressed;
				if (pressed)
				{
					checkID = (checkID + 1) % 4;
					ESP_LOGI(pcTaskGetName(NULL), "Next weapon: %c", checkBuff[checkID][0]);
				}
			}
		} else
		{
			vTaskDelay(1);
		}
	}
}


void receiver(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	NRF24_t dev;
	Nrf24_init(&dev);
	Nrf24_enableNoAckFeature(&dev);

	uint8_t payload = 2;
	uint8_t channel = CONFIG_RADIO_CHANNEL;
	Nrf24_config(&dev, channel, payload);
	Nrf24_configRegister(&dev, EN_AA, 0);
	Nrf24_setRetransmitCount(&dev, 0);

	uint8_t addrLen = 0b11; 
	if (mirf_ADDR_LEN == 3)
	{
		addrLen = 0b01; 
	} else
	if (mirf_ADDR_LEN == 4)
	{
		addrLen = 0b10; 
	}
    Nrf24_configRegister(&dev, SETUP_AW, addrLen); // 3 byte addr


	// Set my own address using 5 characters
	esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"FGHIJ");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ADVANCED
	AdvancedSettings(&dev);
#endif // CONFIG_ADVANCED

	// Print settings
	Nrf24_printDetails(&dev);
	ESP_LOGI(pcTaskGetName(NULL), "Listening...");

	uint8_t buf[32];

	// Clear RX FiFo
	while(1) {
		if (Nrf24_dataReady(&dev) == false) break;
		Nrf24_getData(&dev, buf);
	}

	uint32_t io_num;

	TickType_t lastOnTick = 0;
	TickType_t onTime = 3;

	bool powerPing = false;
	TickType_t lastPowerPing = 0;
	TickType_t PowerDelayOff = 6000;
	TickType_t PowerDelayOn = 200;


	while(1) {
		// Wait for assertion of RX receive complete(RX_DR)
		if(xQueueReceive(gpio_evt_queue, &io_num, 1)) {
			//ESP_LOGD(pcTaskGetName(NULL), "GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
			Nrf24_getData(&dev, buf);
     		bool bOn = buf[0] == checkBuff[checkID][0];
			gpio_set_level(CONFIG_LED_GPIO, !bOn);
			gpio_set_level(CONFIG_LASER_GPIO, bOn);
			//gpio_set_level(CONFIG_CAMERA_GPIO, true);
			lastOnTick = xTaskGetTickCount();
			//ESP_LOGI(pcTaskGetName(NULL), "Got data: %s", buf);
		}

		TickType_t nowTick = xTaskGetTickCount();
		if (lastOnTick > 0 && nowTick - lastOnTick >= onTime )
		{
			lastOnTick = 0;
			gpio_set_level(CONFIG_LED_GPIO, true);
			//gpio_set_level(CONFIG_CAMERA_GPIO, false);
			gpio_set_level(CONFIG_LASER_GPIO, false);
		}

        if (powerPing)
        {
            if (nowTick - lastPowerPing >= PowerDelayOn)
            {
                powerPing = false;
                lastPowerPing = nowTick;
				gpio_set_level(CONFIG_POWERKEY_GPIO, 1);
            }
        } else
        {
            if (nowTick - lastPowerPing >= PowerDelayOff)
            {
                powerPing = true;
                lastPowerPing = nowTick;
                gpio_set_level(CONFIG_POWERKEY_GPIO, 0);
            }
        }
	}
}
#endif // CONFIG_RECEIVER


#if CONFIG_SENDER
void sender(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	NRF24_t dev;
	Nrf24_init(&dev);
	Nrf24_enableNoAckFeature(&dev);

	uint8_t payload = 2;
	uint8_t channel = CONFIG_RADIO_CHANNEL;
	Nrf24_config(&dev, channel, payload);
	Nrf24_configRegister(&dev, EN_AA, 0);
	Nrf24_setRetransmitCount(&dev, 0);
	uint8_t addrLen = 0b11; 
	if (mirf_ADDR_LEN == 3)
	{
		addrLen = 0b01; 
	} else
	if (mirf_ADDR_LEN == 4)
	{
		addrLen = 0b10; 
	}
    Nrf24_configRegister(&dev, SETUP_AW, addrLen); // 3 byte addr


	// Set destination address using 5 characters
	esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"FGHIJ");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ADVANCED
	AdvancedSettings(&dev);
#endif // CONFIG_ADVANCED

	//Print settings
	Nrf24_printDetails(&dev);

	uint8_t buf[32];
	uint32_t io_num;
	while(1) {
		TickType_t nowTick = xTaskGetTickCount();
		sprintf((char *)buf, "Hello World %"PRIu32, nowTick);
		gpio_set_level(CONFIG_LED_GPIO, 0);
		Nrf24_sendNoAck(&dev, buf);
		//ESP_LOGI(pcTaskGetName(NULL), "Wait for sending.....");
		if(xQueueReceive(gpio_evt_queue, &io_num, 1000/portTICK_PERIOD_MS)) {
			gpio_set_level(CONFIG_LED_GPIO, 1);
			ESP_LOGD(pcTaskGetName(NULL), "GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
			ESP_LOGI(pcTaskGetName(NULL),"Send success:%s", buf);
		} else
		{
			gpio_set_level(CONFIG_LED_GPIO, 1);
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
#endif // CONFIG_SENDER


void app_main(void)
{
	//Initialize gpio
	// led
    gpio_reset_pin(CONFIG_LED_GPIO);
	gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LED_GPIO, 1);

    gpio_reset_pin(CONFIG_LASER_GPIO);
	gpio_set_direction(CONFIG_LASER_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LASER_GPIO, 0);

    gpio_reset_pin(CONFIG_POWERKEY_GPIO);
	gpio_set_direction(CONFIG_POWERKEY_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_POWERKEY_GPIO, 1);

	//zero-initialize the config structure.
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(CONFIG_IRQ_GPIO, gpio_isr_handler, (void*) CONFIG_IRQ_GPIO);

	//xTaskCreate(&gpio, "GPIO", 1024*2, NULL, 5, NULL);
#if CONFIG_RECEIVER
	xTaskCreate(&receiver, "RECEIVER", 1024*3, NULL, 5, NULL);
	xTaskCreate(&trigger, "TRIGGER", 1024*3, NULL, 5, NULL);
#endif

#if CONFIG_SENDER
	xTaskCreate(&sender, "SENDER", 1024*3, NULL, 5, NULL);
#endif

}
