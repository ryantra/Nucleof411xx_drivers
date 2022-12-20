/*
 * nucleof41xx_spi_driver.c
 *
 *  Created on: May 22, 2022
 *      Author: rupesh
 */


#include "nucleof41xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Enabling of Peripheral clock for SPI
 */
/*********************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- Base address of SPI peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();

		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();

		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();

		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();

		}else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}

	}
	else
	{

		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();

		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();

		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();

		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();

		}else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
	}

}

/*
 * Initialization of SPI
 */
/*********************************************************************
 * @fn				- SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//1.configure SPI_CR1 register
	uint32_t tempreg = 0;

	//2.configuring device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configure the bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3.configure the SPI baud rate
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5.configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6.configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*
 * De-initialization of SPI
 */
/*********************************************************************
 * @fn				- SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{


}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data sending from SPI
 */
/*********************************************************************
 * @fn				- SPI_SendData
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
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1.wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
	}

	//2.check the DFF bit in CR1
	if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit DFF
		//load data in to DR
		pSPIx->DR = *((uint16_t*)pTxBuffer);
		Len--;
		Len--;
		(uint16_t*)pTxBuffer++;

	}else
	{
		//8bit DFF
		pSPIx->DR = *pTxBuffer;
		Len--;
		pTxBuffer++;
	}

}

/*
 * Data receiving to SPI
 */
/*********************************************************************
 * @fn				- SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1.wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		}

		//2.check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16bit DFF
			//load data from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;

		}else
		{
			//8bit DFF
			*pRxBuffer = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}


}

/*
 * Other Peripheral Control APIs
 */
/*********************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- To perform SPI enable after SPI init
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- To perform SPI enable after SPI init
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}

/*********************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			- To perform SPI enable after SPI init
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/*
 * Interrupt configuration for SPI
 */
/*********************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- Internal Slave Select
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @note			- Can be used for MODF errors
 *
 **********************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

/*
 * Interrupt Priority configuration for SPI
 */
/*********************************************************************
 * @fn				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{


}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1.save the TxBuffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2.mark the SPI state as busy in transmission so that no other code can take
		//same SPI peripheral until the transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1.save the TxBuffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2.mark the SPI state as busy in transmission so that no other code can take
		//same SPI peripheral until the transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	return state;

}

/*
 * Interrupt Handling configuration for SPI
 */
/*********************************************************************
 * @fn				- SPI_IRQHandling
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


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle ove error
		spi_ovr_err_interrupt_handle(pHandle);
	}


}

/*
 * helper function implementation
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit DFF
		//load data in to DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	}else
	{
		//8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		/*
		 * TxLen is zero, so close the SPI communication
		 * and inform the application that Tx is over
		 * Also prevents interrupts from setting up of TXE flag
		 */
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}


}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16bit DFF
		//load data from DR to RX
		  *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;

	}else
	{
		//8bit DFF
		*(pSPIHandle->pRxBuffer)  = (uint8_t)pSPIHandle ->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		/*
		 * reception is complete and
		 * start rxneie interrupts
		 */
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);

	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1.Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2.Inform the application
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);



}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*
 *This is a weak implementation. The application may override this function
 *
 */
__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}

