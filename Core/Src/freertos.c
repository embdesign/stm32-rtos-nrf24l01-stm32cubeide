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

TIM_HandleTypeDef htim4;
extern volatile unsigned long ulHighFrequencyTimerTicks;

void SetupRunTimeStatsTimer(void)
{
	RCC_ClkInitTypeDef    clkconfig;
	uint32_t              pFLatency;
	uint32_t              uwTimclock;
	uint32_t              uwPrescalerValue;

	/* Configure the TIM4 IRQ priority */
	HAL_NVIC_SetPriority(TIM4_IRQn, 10, 0);

	/* Enable the TIM4 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

	/* Enable TIM4 clock */
	__HAL_RCC_TIM4_CLK_ENABLE();

	/* Get clock configuration */
	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

	/* Compute TIM4 clock */
	uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

	/* Compute the prescaler value to have TIM4 counter clock equal to 1MHz */
	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);

	/* Initialize TIM4 */
	htim4.Instance = TIM4;

	/* Initialize TIMx peripheral as follow:
	+ Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
	+ Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	htim4.Init.Period = (1000000 / 1000) - 1;
	htim4.Init.Prescaler = uwPrescalerValue;
	htim4.Init.ClockDivision = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&htim4) == HAL_OK)
	{
		/* Start the TIM time Base generation in interrupt mode */
		HAL_TIM_Base_Start_IT(&htim4);
	}
}

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	SetupRunTimeStatsTimer();
}

__weak unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN VPORT_SUPPORT_TICKS_AND_SLEEP */
__weak void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
  // Generated when configUSE_TICKLESS_IDLE == 2.
  // Function called in tasks.c (in portTASK_FUNCTION).
  // TO BE COMPLETED or TO BE REPLACED by a user one, overriding that weak one.
}
/* USER CODE END VPORT_SUPPORT_TICKS_AND_SLEEP */

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
void frtos_task_rt(void *arg);

TaskHandle_t h_task_rf;
TaskHandle_t h_task_bt;
TaskHandle_t h_task_rt;

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
	if (xTaskCreate((TaskFunction_t)frtos_task_rf, "RF", 256, NULL, 15, &h_task_rf) != pdPASS) {

	}
#endif
#if 0
	if (xTaskCreate((TaskFunction_t)frtos_task_bt, "BT", 256, NULL, 14, &h_task_bt) != pdPASS) {

	}
#endif
#if 1
	if (xTaskCreate((TaskFunction_t)frtos_task_rt, "RT", 256, NULL, 16, &h_task_rt) != pdPASS) {

	}
#endif

	return retval;
}

void frtos_task_rf(void *arg)
{
	uint8_t data = 0x55;

	// Power on delay
	nrf24l01_port_delay_us(NRF24L01_PORT_DELAY_POWER_ON_RESET_US);

	// Set into default state
	nrf24l01_power_down();

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

/* This example demonstrates how a human readable table of run time stats
information is generated from raw data provided by uxTaskGetSystemState().
The human readable table is written to pcWriteBuffer.  (see the vTaskList()
API function which actually does just this). */
void frtos_print_run_time_stats(void)
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime, ulStatsAsPercentage;

	/* Take a snapshot of the number of tasks in case it changes while this
	function is executing. */
	uxArraySize = uxTaskGetNumberOfTasks();

	/* Allocate a TaskStatus_t structure for each task.  An array could be
	allocated statically at compile time. */
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		/* Generate raw status information about each task. */
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
								 uxArraySize,
								 &ulTotalRunTime );

		/* For percentage calculations. */
		ulTotalRunTime /= 100UL;

		console_puts("Name:\t\tState:\tPrio:\tStack:\tRun:\t%:\t" CONFIG_CONSOLE_LINE_ENDING_STRING);

		/* Avoid divide by zero errors. */
		if( ulTotalRunTime > 0 )
		{
			/* For each populated position in the pxTaskStatusArray array,
			format the raw data as human readable ASCII data. */
			for( x = 0; x < uxArraySize; x++ )
			{
				/* Name */
				console_puts(pxTaskStatusArray[ x ].pcTaskName);
				console_puts("\t\t");

				/* State */
				switch( pxTaskStatusArray[ x ].eCurrentState )
				{
					case eRunning:		console_putc('X');
										break;

					case eReady:		console_putc('R');
										break;

					case eBlocked:		console_putc('B');
										break;

					case eSuspended:	console_putc('S');
										break;

					case eDeleted:		console_putc('D');
										break;

					default:			/* Should not get here, but it is included
										to prevent static checking errors. */
										console_putc(' ');
										break;
				}
				console_puts("\t(");

				/* Prio */
				console_puti(pxTaskStatusArray[ x ].uxCurrentPriority, 10);
				console_putc(' ');
				console_puti(pxTaskStatusArray[ x ].uxBasePriority, 10);
				console_puts(")\t");

				/* Stack */
				console_puti(pxTaskStatusArray[ x ].usStackHighWaterMark, 10);
				console_puts("\t");

				/* What percentage of the total run time has the task used?
				This will always be rounded down to the nearest integer.
				ulTotalRunTimeDiv100 has already been divided by 100. */
				ulStatsAsPercentage =
					  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

				if( ulStatsAsPercentage > 0UL )
				{
					console_puti(pxTaskStatusArray[ x ].ulRunTimeCounter, 10);
					console_puts("\t");
					console_puti(ulStatsAsPercentage, 10);
					console_puts("%" CONFIG_CONSOLE_LINE_ENDING_STRING);
				}
				else
				{
					/* If the percentage is zero here then the task has
					consumed less than 1% of the total run time. */
					console_puti(pxTaskStatusArray[ x ].ulRunTimeCounter, 10);
					console_puts("\t<1%" CONFIG_CONSOLE_LINE_ENDING_STRING);
				}
			}
		}

		/* The array is no longer needed, free the memory it consumes. */
		vPortFree( pxTaskStatusArray );
	}
}

void frtos_task_rt(void *arg)
{
	while (1) {
		frtos_delay_ms(5000);
		frtos_print_run_time_stats();
	}
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
