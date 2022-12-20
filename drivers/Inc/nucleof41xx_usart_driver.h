/*
 * nucleof41xx_usart_driver.h
 *
 *  Created on: Nov 13, 2022
 *  Author: rupesh
 */

#ifndef INC_NUCLEOF41XX_USART_DRIVER_H_
#define INC_NUCLEOF41XX_USART_DRIVER_H_

#include "nucleof41xx.h"

/*
 * Configuration structure for USART
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;

}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t 	*pUSARTx;
	USART_Config_t 	USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t *TxLen;
	uint32_t *RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

}USART_Handle_t;

/*
 * @USART_Mode refrence CR1 register
 * Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2


/*
 *@USART_Baud
 */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_400			400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400		38400
#define USART_STD_BAUD_57600		57600
#define USART_STD_BAUD_115200		115200
#define USART_STD_BAUD_230400		230400
#define USART_STD_BAUD_460800		460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2000000		2000000
#define USART_STD_BAUD_300000		3000000

/*
 * @USART_ParityControl
 * Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD			2
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_DISABLE		0

/*
 * @USART_WordLength refer USART_CR1
 * Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1

/* refer CR2
 * Possible options for USART_NoOfStopBits refer CR2
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3

/*
 * @USART_HWFlowControl
 * Possible option for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART flags
 */
#define USART_FLAG_TXE		(1 << USART_SR_TXE)
#define USART_FLAG_RXNE		(1 << USART_SR_RXNE)
#define USART_FLAG_TC		(1 << USART_SR_TC)

/*
 * Application States
 */
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2
#define USART_READY					0

#define USART_EVENT_TX_CPMLT		0
#define USART_EVENT_RX_CPMLT		1
#define USART_EVENT_IDLE			2					//IDLE line detected
#define USART_EVENT_CTS				3					//CTS flag
#define USART_EVENT_PE				4					//Parity Error
#define USART_ERR_FE				5					//Framing Error
#define USART_ERR_NE				6					//Noise flag
#define USART_ERR_ORE				7					//overrun error



/****************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *****************************************************************************/

/*
 * Peripheral clock control
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);



/*
 * Init and De-Init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data send and receive <Blocking mode, no interrupt>
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReveiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data send and receive <Non -blocking mode, no interrupt>
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReveiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


/*
 * Other Peripheral Control APIs
 */


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callback
 */
void USART_ApplicationEventCallBack(USART_Handle_t *pUSARTHandle,uint8_t AppEv);




#endif /* INC_NUCLEOF41XX_USART_DRIVER_H_ */
