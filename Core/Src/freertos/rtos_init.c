// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <freertos/rtos_init.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  FreeRTOS timer based on systick.
 *  This is copy of the <Midlewares/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c>
 */

#include "stm32f1xx_hal.h"

void rtos_init(void)
{
	/*
	 * If you are using an STM32 with the STM32 driver library then ensure all
	 * the priority bits are assigned to be preempt priority bits by calling
	 * NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is
	 * started.
	 */
	//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// Stop TIM3 during debugging
	__HAL_DBGMCU_FREEZE_TIM3();
}
