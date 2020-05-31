// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <freertos/rtos_timer_tim3.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  FreeRTOS timer based on tim3
 *
 *  Maximum idle time is Couter_MAX/(CK_PSC/Prescaler) = 0xffff/2000 ~ 32s
 *  Because this is 16-bit timer it do not provide superior functionality in
 *  compare with 24-bit Systick timer. But due integrated Prescaler the larger
 *  idle times are possible at the cost of timing precision.
 */
#if 1
#define configPRE_SLEEP_PROCESSING(m_a)	    rtos_timer_tim3_pre_sleep_processing(m_a)
#define configPOST_SLEEP_PROCESSING(m_a)    rtos_timer_tim3_post_sleep_processing(m_a)

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"

/* Enable debug gpio
 * PC13 - WFI time
 * PC14 - vPortSuppressTicksAndSleep time
 * PC15 - TIM3_IRQHandler time
 **/
//#define PORT_TIMER_TIM_DBG_GPIO

/* The TIM3 is a 16-bit counter. */
#define portMAX_16_BIT_NUMBER				( 0xffffUL )

/*
 * The number of TIM3 increments that make up one tick period.
 */
#if( configUSE_TICKLESS_IDLE == 2 )
static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 16 bit resolution of the TIM3 timer.
 */
#if( configUSE_TICKLESS_IDLE == 2 )
static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

static void rtos_timer_tim3_pre_sleep_processing(TickType_t * ticks)
{
	HAL_SuspendTick();
#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
#endif /* PORT_TIMER_TIM_DBG_GPIO */
}

static void rtos_timer_tim3_post_sleep_processing(TickType_t * ticks)
{
#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
#endif /* PORT_TIMER_TIM_DBG_GPIO */
	HAL_ResumeTick();
}

#if( configUSE_TICKLESS_IDLE == 2 )
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
	uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedTimDecrements;
	TickType_t xModifiableIdleTime;

#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_14);
#endif /* PORT_TIMER_TIM_DBG_GPIO */

	/* Make sure the TIMx reload value does not overflow the counter. */
	if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
	{
		xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
	}

	/* Stop the TIMx momentarily.  The time the TIMx is stopped for
	is accounted for as best it can be, but using the tickless mode will
	inevitably result in some tiny drift of the time maintained by the
	kernel with respect to calendar time. */
	TIM3->CR1 |= (TIM_CR1_UDIS);

	/* Calculate the reload value required to wait xExpectedIdleTime
	tick periods.  -1 is used because this code will execute part way
	through one of the tick periods. */
	ulReloadValue = ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );

	/* Enter a critical section but don't use the taskENTER_CRITICAL()
	method as that will mask interrupts that should exit sleep mode. */
	__asm volatile( "cpsid i" ::: "memory" );
	__asm volatile( "dsb" );
	__asm volatile( "isb" );

	/* If a context switch is pending or a task is waiting for the scheduler
	to be unsuspended then abandon the low power entry. */
	if( eTaskConfirmSleepModeStatus() == eAbortSleep )
	{
		/* Restart from whatever is left in the count register to complete
		this tick period. */
		TIM3->ARR = TIM3->CNT;

		/* Restart TIM3. */
		TIM3->CR1 &= ~(TIM_CR1_UDIS);

		/* Reset the reload register to the value required for normal tick
		periods. */
		TIM3->ARR = ulTimerCountsForOneTick - 1UL;

		/* Re-enable interrupts - see comments above the cpsid instruction()
		above. */
		__asm volatile( "cpsie i" ::: "memory" );
	}
	else
	{
		/* Set the new reload value. */
		TIM3->ARR = ulReloadValue;

		/* Set TIM3 the count value back to
		default. */
		TIM3->CNT = TIM3->ARR;

		/* Clear already counted to zero flag */
		TIM3->SR &= ~(TIM_SR_CC1IF);

		/* Restart TIM3. */
		TIM3->CR1 &= ~(TIM_CR1_UDIS);

		/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
		set its parameter to 0 to indicate that its implementation contains
		its own wait for interrupt or wait for event instruction, and so wfi
		should not be executed again.  However, the original expected idle
		time variable must remain unmodified, so a copy is taken. */
		xModifiableIdleTime = xExpectedIdleTime;
		configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );
		if( xModifiableIdleTime > 0 )
		{
			__asm volatile( "dsb" ::: "memory" );
			__asm volatile( "wfi" );
			__asm volatile( "isb" );
		}
		configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );

		/* Re-enable interrupts to allow the interrupt that brought the MCU
		out of sleep mode to execute immediately.  see comments above
		__disable_interrupt() call above. */
		__asm volatile( "cpsie i" ::: "memory" );
		__asm volatile( "dsb" );
		__asm volatile( "isb" );

		/* Disable interrupts again because the clock is about to be stopped
		and interrupts that execute while the clock is stopped will increase
		any slippage between the time maintained by the RTOS and calendar
		time. */
		__asm volatile( "cpsid i" ::: "memory" );
		__asm volatile( "dsb" );
		__asm volatile( "isb" );

		/* Disable the TIM3 clock.  Again,
		the time the TIM3 is stopped for is accounted for as best it can
		be, but using the tickless mode will inevitably result in some tiny
		drift of the time maintained by the kernel with respect to calendar
		time*/
		TIM3->CR1 |= (TIM_CR1_UDIS);

		/* Determine if the TIM3 clock has already counted to zero and
		been set back to the current reload value (the reload back being
		correct for the entire expected idle time) or if the SysTick is yet
		to count to zero (in which case an interrupt other than the SysTick
		must have brought the system out of sleep mode). */
		if( ( TIM3->SR & TIM_SR_CC1IF ) != 0 )
		{
			/* Clear TIM3 count reached to zero flag */
			TIM3->SR &= ~(TIM_SR_CC1IF);

			/* As the pending tick will be processed as soon as this
			function exits, the tick value maintained by the tick is stepped
			forward by one less than the time spent waiting. */
			ulCompleteTickPeriods = xExpectedIdleTime - 1UL;

			/* The TIM3 interrupt is already pending, and the TIM3 count
			reloaded with ulReloadValue.  Reset the
			TIM3->ARR with whatever remains of this tick
			period. */
			TIM3->ARR = ulTimerCountsForOneTick - 1;
		}
		else
		{
			/* Something other than the TIM3 interrupt ended the sleep.
			Work out how long the sleep lasted rounded to complete tick
			periods (not the ulReload value which accounted for part
			ticks). */
			ulCompletedTimDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - TIM3->CNT;

			/* How many complete tick periods passed while the processor
			was waiting? */
			ulCompleteTickPeriods = ulCompletedTimDecrements / ulTimerCountsForOneTick;

			/* The reload value is set to whatever fraction of a single tick
			period remains. */
			TIM3->ARR = TIM3->CNT;
		}

		/* Restart TIM3 so it runs from portNVIC_SYSTICK_LOAD_REG
		again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
		value. */
		TIM3->CNT = TIM3->ARR;
		TIM3->CR1 &= ~(TIM_CR1_UDIS);
		vTaskStepTick( ulCompleteTickPeriods );
		TIM3->ARR = ulTimerCountsForOneTick - 1UL;

		/* Exit with interrupts enabled. */
		__asm volatile( "cpsie i" ::: "memory" );
	}

#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14);
#endif /* configUSE_TICKLESS_IDLE */
}
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Setup the TIM3 timer to generate the tick interrupts at the required
 * frequency.
 */
void vPortSetupTimerInterrupt( void )
{
	/*
	 * Initialize TIM3 as follow:
	 * Period            = CK_PSC/Prescaler/configTICK_RATE_HZ
	 *                     to have (configTICK_RATE_HZ) time base.
	 * Prescaler         = 2 * tim_clock/configTICK_RATE_HZ - 1
	 *                     to have a 2 * configTICK_RATE_HZ counter clock.
	 * Counter direction = down
	 **/

	RCC_ClkInitTypeDef clk_config;
	uint32_t           flash_latency;
	uint32_t           tim_clock;
	uint16_t           prescaler_value;

	/* Configure the TIMx IRQ priority */
	HAL_NVIC_SetPriority(TIM3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);

	/* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable TIMx clock */
	__HAL_RCC_TIM3_CLK_ENABLE();

	/* Get clock configuration */
	HAL_RCC_GetClockConfig(&clk_config, &flash_latency);

	/* Compute TIM3 clock */
	tim_clock = 2 * HAL_RCC_GetPCLK1Freq();

	/* Compute the prescaler value to have TIM3 counter clock equal to 2 * configTICK_RATE_HZ  */
	prescaler_value = (uint16_t) (((tim_clock / configTICK_RATE_HZ) / 2) - 1);

	tim_clock /= (prescaler_value + 1);

	/* Calculate the constants required to configure the tick interrupt. */
	#if( configUSE_TICKLESS_IDLE == 2 )
	{
		ulTimerCountsForOneTick = tim_clock/configTICK_RATE_HZ;
		xMaximumPossibleSuppressedTicks = portMAX_16_BIT_NUMBER / ulTimerCountsForOneTick;
	}
	#endif /* configUSE_TICKLESS_IDLE */

	{
		uint32_t tmpcr1;
		tmpcr1 = TIM3->CR1;

		/* Select the Counter Mode Down */
		tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
		tmpcr1 |= (TIM_COUNTERMODE_DOWN);

		/* Set the clock division - input filter */
		tmpcr1 &= ~(TIM_CR1_CKD);
		tmpcr1 |= TIM_CLOCKDIVISION_DIV1;

		/* Set the auto-reload preload */
		tmpcr1 |= (TIM_CR1_ARPE);

		TIM3->CR1 = tmpcr1;
	}

    /* Set the Autoreload value */
    TIM3->ARR = tim_clock/configTICK_RATE_HZ - 1;

    /* Set the Prescaler value */
    TIM3->PSC = prescaler_value;

    /* Set the TIM capture/compare value */
    TIM3->CCR1 = 0UL;

    /* Generate an update event to reload the Prescaler
       and the repetition counter (only for advanced timer) value immediately */
    TIM3->EGR = TIM_EGR_UG;

    /* Enable the TIM Update interrupt */
    TIM3->DIER |= (TIM_IT_UPDATE);

    /* Enable the Peripheral */
    TIM3->CR1 |= (TIM_CR1_CEN);
}

void TIM3_IRQHandler(void)
{
#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_15);
#endif /* configUSE_TICKLESS_IDLE */

	if (TIM3->SR & TIM_SR_UIF) {
		if (TIM3->DIER & TIM_DIER_UIE) {
			TIM3->SR &= ~(TIM_SR_UIF);
		}
	}

	/* The TIM3 runs at the lowest interrupt priority, so when this interrupt
	executes all interrupts must be unmasked.  There is therefore no need to
	save and then restore the interrupt mask value as its value is already
	known. */
	portDISABLE_INTERRUPTS();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* A context switch is required.  Context switching is performed in
			the PendSV interrupt.  Pend the PendSV interrupt. */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	portENABLE_INTERRUPTS();

#if defined(PORT_TIMER_TIM_DBG_GPIO)
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15);
#endif /* configUSE_TICKLESS_IDLE */
}
#endif
