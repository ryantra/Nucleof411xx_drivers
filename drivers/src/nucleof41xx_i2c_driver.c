/*
 * nucleof41xx_i2c_driver.c
 *
 *  Created on: Nov 7, 2022
 *      Author: rupesh
 */


#include "nucleof41xx_i2c_driver.h"


/************************************************************
 *
 * Helper function declaration
 *
 ***********************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr 	= SlaveAddr << 1;
	SlaveAddr 	&= ~(1);		//SlaveAddr is Slave Address + r/nw bit = 0
	pI2Cx->DR	= SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr 	= SlaveAddr << 1;
	SlaveAddr 	|= 1;			//SlaveAddr is Slave Address + r/nw bit = 1
	pI2Cx->DR	= SlaveAddr;

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;

	//Check for device Mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in Master Mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//First disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag( read SR1, read SR2)
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else
		{
			//Clear the ADDR flag (read SR1, read SR2)
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else
	{
		//Device in Slave Mode
		//Clear the ADDR flag (read SR1, read SR2)
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}

}


/*
 * Enabling of Peripheral clock for I2Cx
 */
/*********************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- Base address of I2C peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();

		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();

		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();

		}

	}
	else
	{

		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();

		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();

		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();

		}
	}

}




/*
 * Initialization of Peripheral for I2Cx
 */
/*********************************************************************
 * @fn				- I2C_Init
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- Base address of I2C peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed < I2C_SCL_SPEED_SM)
	{
		//mode is standard
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg  |= (ccr_value & 0xFFF);

	}else
	{
		//mode is fast
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

		}else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg  |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed < I2C_SCL_SPEED_SM)
		{
			//mode is standard

		tempreg = (RCC_GetPCLK1Value()/ 1000000U) + 1; 	//i.e from specification I2C TRISE register

		}else
		{
			//mode is fast
			tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;		//i.e 1ns
		}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/*
 * Di-Initialization of Peripheral for I2Cx
 */
/*********************************************************************
 * @fn				- I2C_DeInit
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- Base address of I2C peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{


}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data send and receive
 */
/*
 * Data sending of I2Cx Master to Slave
 */
/*********************************************************************
 * @fn				- I2C_MasterSendData
 *
 * @brief			- This function
 *
 * @param[in]		- Base address of I2C peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1.Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.Confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched i.e pulled to LOW.
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.Send the address of the Slave with r/nw bit set to W(0) (total of 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4.Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5.Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be stretched i.e pulled to LOW
	I2C_ClearADDRFlag(pI2CHandle);

	//6.Send data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait till TxE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7.When Len becomes zero wait for TxE=1 and BTF=1 before generating STOP condition
	//  NOte: TXE = 1, BTF = 1, means that both SR and DR are empty and next transmissions
	//  should begin when BTF = 1. SCL will be stretched i.e pulled to LOW

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));


	//8.Generate STOP condition and master need not to wait for the completion of stop conditions.
	//	Note: Generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/*
 * Data receiving of I2Cx Master from Slave
 */
/*********************************************************************
 * @fn				- I2C_MasterReveiveData
 *
 * @brief			- This function enables or disables peripheral clock
 * 					  for the given port
 *
 * @param[in]		- Base address of I2C peripheral
 *
 * @param[in]		- Enable or disable macros
 *
 *
 * @return			- none
 *
 * @note			- n/a
 *
 **********************************************************************/
void I2C_MasterReveiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1.Generate the START conditions
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.Confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched i.e pulled to LOW.
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.Send the address of the Slave with r/nw bit set to R(1) (total of 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4.Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//procedure to read only one byte from Slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//Generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);		//data reception only begin after clearing ADDR

		//read the data untill Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until the RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

			if(i == 2) 	//if last 2 bits remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enabling ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

}


/*
* Data sending of I2Cx Master to Slave(Non-blocking Mode)
*/
/*********************************************************************
* @fn				- I2C_MasterSendDataIT
*
* @brief			- This function
*
* @param[in]		- Base address of I2C peripheral
*
* @param[in]		- Enable or disable macros
*
*
* @return			- none
*
* @note				- n/a
*
**********************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);


			//to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}


/*
* Data sending of I2Cx Master to Slave (non-blocking mode)
*/
/*********************************************************************
* @fn				- I2C_MasterReveiveDataIT
*
* @brief			- This function
*
* @param[in]		- Base address of I2C peripheral
*
* @param[in]		- Enable or disable macros
*
*
* @return			- none
*
* @note				- n/a
*
**********************************************************************/
uint8_t I2C_MasterReveiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
		uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; 					//Rx size is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);


			//to enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1.Load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2.Decrement the TxLen
		pI2CHandle->TxLen--;

		//3.Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

			//read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//Close the I2C data reception and notify the application

		//1.Generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2.Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		//3.Notify the application
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}

}
/*
* ISR Event Handle
*/
/*********************************************************************
* @fn				- I2C_EV_IRQHandling
*
* @brief			- ISR1 Event Handler
*
* @param[in]		- Base address of I2C peripheral
*
* @param[in]		- Enable or disable macros
*
*
* @return			- none
*
* @note				- n/a
*
**********************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	//page 481 of Reference manual <I2C Interrupts requests>
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated by SB event
		//This block will not executed in Slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//Interrupt is generated because of ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF Flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1.Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2.Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3.Notify the application about the transmission complete
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CPMLT);
				}
			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// 4. Handle For interrupt generated by STOPF event
	// Note: Stop detection flag is applicable only slave mode . For master this flag will never be set
	// Below code block will not be executed by the master since STOPF will not set in Master Mode
	if(temp1 && temp3)
	{
		//STOPF Flag is set
		//Clear the STOPF i.e '1' read SR1 '2' Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check the device Mode (Master/Slace in SR2->MSL)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXE Flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}

		}else
		{
			//for slave and make sure slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//Check the device mode i.e Master or Slave
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//RXNE Flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)			//the device is master
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}

		}else
		{
			//for slave and making sure slave is really in receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}

		}

	}

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable IBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable IBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReveiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}

/*
* ISR Error Handle
*/
/*********************************************************************
* @fn				- I2C_EV_IRQHandling
*
* @brief			- ISR2 Error Handler
*
* @param[in]		- Base address of I2C peripheral
*
* @param[in]		- Enable or disable macros
*
*
* @return			- none
*
* @note				- n/a
*
**********************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_ARLO);

		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_AF);

		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_TIMEOUT);

		}

}



