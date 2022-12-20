/*
 * 03button_int.c
 *
 *  Created on: Nov 17, 2022
 *      Author: rupesh
 */

#include "nucleof41xx.h"


#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

		GPIO_Handle_t GpioLed, GPIOBtn;

		memset(&GpioLed,0,sizeof(GpioLed));
		memset(&GPIOBtn,0,sizeof(GPIOBtn));

		//GPIO LED configuration
		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOA, ENABLE);

		GPIO_Init(&GpioLed);

		//GPIO Button configuration
		GPIOBtn.pGPIOx = GPIOB;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

		GPIO_PeriClockControl(GPIOB, ENABLE);

		GPIO_Init(&GPIOBtn);

		//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, GPIO_PIN_RESET);

		//IRQ configuration
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


		while(1);
}

void EXTI9_5_IRQHandler(void)
		{
			GPIO_IRQHandling(GPIO_PIN_NO_5);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_12);
		}




