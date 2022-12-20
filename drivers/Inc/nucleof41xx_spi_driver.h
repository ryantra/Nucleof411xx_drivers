/*
 * nucleof41xx_spi_driver.h
 *
 *  Created on: May 22, 2022
 *      Author: rupesh
 */

#ifndef INC_NUCLEOF41XX_SPI_DRIVER_H_
#define INC_NUCLEOF41XX_SPI_DRIVER_H_

#include "nucleof41xx.h"

/*
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;				//Data frame format	<8-bit default i.e '0', 16-bit i.e '1'>
	uint8_t SPI_CPOL;				//Clock polarity
	uint8_t SPI_CPHA;				//Clock phase
	uint8_t SPI_SSM;				//Software slave management

}SPI_Config_t;

/*
 * Handle structure of SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t	*pSPIx;		//This holds the base address of SPIx(x:0,1,2) peripheral
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;		//to store the app. Tx buffer address
	uint8_t			*pRxBuffer;		//to store the app. Rx buffer address
	uint8_t			TxLen;			//to store Tx len
	uint8_t			RxLen;			//to store Rx len
	uint8_t			TxState;		//to store Tx state
	uint8_t			RxState;		//to store Rx state

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1	//for full duplex
#define SPI_BUS_CONFIG_HD				2	//for half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3	//Simplex Rx

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0

/*
 * SPI related flags definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/*
 * Possible SPI Application States
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4


/****************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *****************************************************************************/

/*
 * Peripheral clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);    //blocking mode >no Interrupt<
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);    //non-blocking >Interrupt<
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);



/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_NUCLEOF41XX_SPI_DRIVER_H_ */
