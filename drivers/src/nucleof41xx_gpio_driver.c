/*
 * nucleof41xx_gpio_driver.c
 *
 *  Created on: May 12, 2022
 *      Author: rupesh
 */


#include "nucleof41xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*********************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- base address of the GPIO peripheral
 *
 * @param[in]		- ENABLE or DISABLE macros
 *
 *
 * @return			- none
 *
 * @note			- none
 *
 **********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/*********************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize or de-initialize the GPIOs
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;		//temp.register

	//enabling the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);


	//1. Configure the mode of the GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;	//setting

	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure FTSR
			EXTI->FTSR 	|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing the corresponding RTSR bit
			EXTI->RTSR 	&= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure RTSR
			EXTI->RTSR 	|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure both FTSR and RTSR
			EXTI->FTSR 	|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR 	|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configuring the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		//enabling SYSCFG clk
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//setting

	temp = 0;

	//3. COnfigure the PuPd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR 	&= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR 	|= temp;	//setting

	temp = 0;
	//4. Configure the OpType
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;	//setting

	temp = 0;

	//5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));	//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}
/*********************************************************************
 * @fn				- GPIO_Init, GPIO_DeInit
 *
 * @brief			- This function initialize or de-initialize the GPIOs
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
/*********************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @param[in]		- GPIO's port
 *
 * @param[in]		- GPIOS's pin number
 *
 * @return			- 0 or 1
 *
 * @note			-
 *
 **********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*********************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		- GPIO port
 *
 * @return			- 0 or 1
 *
 * @note			-
 *
 **********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		- GPIO's Pins
 *
 * @param[in]		- GPIO's Port
 *
 * @return			- void i.e write function
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*********************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
/*********************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			-
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program Interrupt Set-Enable Register (ISER0)
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64) //32 -63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 i.e 64-95
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register (Interrupt Clear Enable)
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 6 && IRQNumber < 96)
		{
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn				- GPIO_IRQPriority Configuration
 *
 * @brief			-

 * @param[in]		-
 *
 * @return			-
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//Find out IPR register
	uint8_t iprx = IRQNumber / 4; 	//which IPR register
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PRI_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + (iprx)) |= (IRQPriority << shift_amount);

}



/*********************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			-

 * @param[in]		-
 *
 * @return			-
 *
 * @note			-
 *
 **********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clera the EXTI Peripheral register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
