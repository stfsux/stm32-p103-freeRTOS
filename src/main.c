/*
 * main.c
 *
 *  Created on: Jan 29, 2014
 *      Author: stfsux
 */

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

static void stm32_setup_hardware (void);
void task1 (void *param);
void task2 (void *param);
void vApplicationStackOverflowHook (xTaskHandle pxTask, char *pcTaskName);

unsigned char bitval1 = 0;
unsigned char bitval2 = 0;

int
 main (void)
{
	stm32_setup_hardware ();
	xTaskCreate(task1, "task1", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
	xTaskCreate(task2, "task2", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
	vTaskStartScheduler ();
	while (1);
	return 0;
}

static void
 stm32_setup_hardware (void)
{
	GPIO_InitTypeDef gpio_conf =
	{
			.GPIO_Pin = GPIO_Pin_All,
			.GPIO_Speed = GPIO_Speed_50MHz,
			.GPIO_Mode = GPIO_Mode_Out_PP
	};
	unsigned int i = 0;

	/* Reset the clock system configuration. */
	RCC_DeInit ();
	/* Use external oscillator. */
	RCC_HSEConfig (RCC_HSE_ON);
	/* Wait for that shit. */
	RCC_WaitForHSEStartUp ();
	/* Use that shit for sysclk. */
	RCC_SYSCLKConfig (RCC_SYSCLKSource_HSE);

	/* Setup the AHB clock. */
	RCC_HCLKConfig (RCC_SYSCLK_Div1);
	/* Setup the APB1 clock. */
	RCC_PCLK1Config (RCC_HCLK_Div1);
	/* Setup the APB2 clock. */
	RCC_PCLK2Config (RCC_HCLK_Div1);
	/* Enable APB2 for GPIOs. */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
			RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
			RCC_APB2Periph_AFIO, ENABLE );

	/* Initialize GPIO. */
	GPIO_DeInit (GPIOC);
	GPIO_Init (GPIOC, &gpio_conf);

	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

}

void
 task1 (void *param)
{
	(void)param;
	while (1)
	{
		GPIO_WriteBit (GPIOC, GPIO_Pin_0, bitval1);
		bitval1 = bitval1 ^ 1;
		vTaskDelay (500);
	}
}

void
 task2 (void *param)
{
	(void)param;
	while (1)
	{
		GPIO_WriteBit (GPIOC, GPIO_Pin_1, bitval2);
		bitval2 = bitval2 ^ 1;
		vTaskDelay (250);
	}
}

void vApplicationStackOverflowHook (xTaskHandle pxTask, char *pcTaskName)
{
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
