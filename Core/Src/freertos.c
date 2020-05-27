/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <driver/serial.h>
#include <console.h>
#include <shell.h>
#include <lib/nrf24l01.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void frtos_delay_ms(unsigned int delay_ms)
{
	TickType_t delay_ticks = delay_ms * configTICK_RATE_HZ / 1000;

	vTaskDelay(delay_ticks);
}
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

int __io_putchar(int ch)
{
#if 0
    ITM_SendChar(ch);
    return(ch);
#else
    LL_USART_TransmitData8(USART1, ch);
    while (!LL_USART_IsActiveFlag_TXE(USART1)) {};
    return(ch);
#endif
}

void nrf24l01_irq_callback(void)
{
	//uint8_t data;

	// read the data from the radio chip
	//(void)nrf24l01_read_rx_payload((uint8_t *)&data, 1U);

	// acknowledge the interrupt on the radio chip
	//nrf24l01_irq_clear_all();

	LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void frtos_task_rf(void *arg);
void frtos_task_bt(void *arg);

TaskHandle_t h_task_rf;
TaskHandle_t h_task_bt;

int frtos_start(void)
{
	int retval = 0;

	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

	static struct serial_desc s_serial;
	uint32_t config = SERIAL_BR_TO_CONFIG(115200) |
			          SERIAL_CONFIG_TO_WL(SERIAL_WORD_LENGTH__8) |
					  SERIAL_SB_TO_CONFIG(SERIAL_STOP_BITS__1) |
					  SERIAL_PB_TO_CONFIG(SERIAL_PARITY__NONE);

	dm_serial_init(&s_serial, SERIAL_PORT__TTYS0, DM_DRIVER_ID__STM32F10X_UART);
	dm_serial_setconfig(&s_serial, config);
	console_init();
	console_attach(CONSOLE_ATTACH_DM_DEVICE(&s_serial));
	console_enable();

#if 1
	if (xTaskCreate((TaskFunction_t)frtos_task_rf, "RF", 1024, NULL, 21, &h_task_rf) != pdPASS) {

	}
#endif
#if 1
	if (xTaskCreate((TaskFunction_t)frtos_task_bt, "BT", 1024, NULL, 20, &h_task_bt) != pdPASS) {

	}
#endif
	return retval;
}

void frtos_task_rf(void *arg)
{
	uint8_t data = 0x55;

	// Power on reset delay >100ms
	//frtos_delay_ms(100);

	// Set into default state
	//nrf24l01_power_down();

	// Set as transmitter
	nrf24l01_initialize_debug(false, 1, false);

	for(;;) {
		frtos_delay_ms(1000);

		nrf24l01_write_tx_payload(&data, 1U, true); // transmit a byte over rf

		for (uint8_t tries = 0U; tries < 50U; tries++)
		{
			  // check to see if the data has been sent
			  if (nrf24l01_irq_pin_active() && nrf24l01_irq_tx_ds_active())
			  {
					// data sent, no more tries
					break;
			  }
		}

		nrf24l01_flush_tx();					// make sure everything is sent by rf module
		nrf24l01_irq_clear_all(); 				// clear rf module interrupts
	}
}

void frtos_task_bt(void *arg)
{
	shell();
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
