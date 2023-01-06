/*
 * nucleof41xx.h
 *
 *  Created on: May 12, 2022
 *      Author: rupesh
 */



#ifndef INC_NUCLEOF41XX_H_
#define INC_NUCLEOF41XX_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define __vo 	volatile
#define __weak	__attribute__((weak))



/********************START:Processor Specific Detail*******************/
/*
 *ARM COrtex-Mx Processor NVIC ISER (Interrupt Set enable Resister
 *
 **********************************************************************/

#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)


 /*
 *ARM COrtex-Mx Processor NVIC ICER (Interrupt Clear enable Resister
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)



 /*
 *ARM COrtex-Mx Processor Priority Register Addresses
 */
#define NVIC_PR_BASEADDR			((__vo uint32_t*)0xE000E400)

 /*
 *ARM Cortex-Mx Processor Priority bits implemented in Priority Register
 */
#define NO_PRI_BITS_IMPLEMENTED		4

/*
 *base addresses of Flash and SRAM Memories
 */

#define FLASH_BASEADDR						0X80000000U			//flash  base address
#define SRAM1_BASEADDR						OX20000000U			//SRAM1 base address
#define ROM_BASEADDR						0x1FFF0000U			//System Memory base address
#define SRAM								SRAM1_BASEADDR		//Embedded SRAM

/*
 * AHBX and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U


/*
 * base addresses of peripheral which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * base addresses of peripheral which are hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)


#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)


/*
 * base addresses of peripheral which are hanging on APB2 bus
 */

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0X3C00)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)


/**********************peripheral register definition structures*******************/

/*
 * Peripheral register structure definition for MCU
 */

typedef struct
{
	__vo uint32_t MODER;				//GPIO port mode register
	__vo uint32_t OTYPER;				//GPIO port output type register
	__vo uint32_t OSPEEDR;				//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;					//GPIO port input data register
	__vo uint32_t ODR;					//GPIO port output data register
	__vo uint32_t BSRR;					//GPIO port bit set/reset register
	__vo uint32_t LCKR;					//GPIO port configuration lock register
	__vo uint32_t AFR[2];				//AFR[0]GPIO alternate function low register AFR[1]GPIO alternate function high register


}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	__vo uint32_t CR;					//RCC clock control register
	__vo uint32_t PLLCFGR;				//RCC PLL configuration register
	__vo uint32_t CFGR;					//RCC clock configuration register
	__vo uint32_t CIR;					//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;				//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;				//RCC AHB2 peripheral reset register
	uint32_t	  RESERVED1[2];			//Reserved 0x18-0x1C
	__vo uint32_t APB1RSTR;				//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;				//RCC APB2 peripheral reset register
	uint32_t	  RESERVED2[2];			// Reserved 0x28-0x2C
	__vo uint32_t AHB1ENR;				//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;				//RCC AHB2 peripheral clock enable register
	uint32_t	  RESERVED3[2];			//Reserved 0x38-0x3C
	__vo uint32_t APB1ENR;				//RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;				//RCC APB2 peripheral clock enable register
	uint32_t	  RESERVED4[2];			//Reserved 0x48-0x4C
	__vo uint32_t AHB1LPENR;			//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;			//RCC AHB2 peripheral clock enable in low power mode register
	uint32_t 	  RESERVED5[2];			//reserved 0x58-0x5C
	__vo uint32_t APB1LPENR;			//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;			//RCC APB2 peripheral clock enabled in low power mode register
	uint32_t 	  RESERVED6[2];			//Reserved 0x68-0x6c
	__vo uint32_t BDCR;					//RCC Backup domain control register
	__vo uint32_t CSR;					//RCC clock control & status register
	uint32_t	  RESERVED7[2];			//Reserved 0x78-0x7C
	__vo uint32_t SSCGR;				//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;			//RCC PLLI2S configuration register
	uint32_t	  RESERVED8;			//Reserved 0x88
	__vo uint32_t DCKCFGR;				//RCC Dedicated Clocks Configuration Register

}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI External Interrupt/Event Controller
 */

typedef struct
{
	__vo uint32_t IMR;					//Interrupt Mask Register 				<offset 0x00>
	__vo uint32_t EMR;					//Event Mask Register 					<offset 0x04>
	__vo uint32_t RTSR;					//Rising Trigger Selection Register 	<offset 0x08>
	__vo uint32_t FTSR;					//Falling Trigger Selection Register	<offset 0x0C>
	__vo uint32_t SEIER;				//Software Interrupt Event Register 	<offset 0x10>
	__vo uint32_t PR;					//Pending Register						<offset 0x14>
}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;				//SYSCFG Memory Remap Register					<offset 0x00>
	__vo uint32_t PMC;					//SYSCFG Peripheral Mode Configuration Register	<offset 0x04>
	__vo uint32_t EXTICR[4];			//SYSCFG External Configuration Register 		<offset 0x08-14>
	__vo uint32_t RESERVED1[2];			//offset 0x18-1C
	__vo uint32_t CMPCR;				//Compensational Cell Control Register			<offset 0x20>
	__vo uint32_t RESERVED2[2];			//offset 0x24-28
	__vo uint32_t CFGR;					//Not available for NucleoF411xx 				<offset 0x2c>
}SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;					//offset 0x00
	__vo uint32_t CR2;					//offset 0x04
	__vo uint32_t SR;					//offset 0x08
	__vo uint32_t DR;					//offset 0x0C
	__vo uint32_t CRCPR;				//offset 0x10
	__vo uint32_t RXCRCR;				//offset 0x14
	__vo uint32_t TXCRCR;				//offset 0x18
	__vo uint32_t I2SCFGR;				//offset 0x1C
	__vo uint32_t I2SPR;				//offset 0x20

}SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;					//offset 0x00
	__vo uint32_t CR2;					//offset 0x04
	__vo uint32_t OAR1;					//offset 0x08
	__vo uint32_t OAR2;					//offset 0x0C
	__vo uint32_t DR;					//offset 0x10
	__vo uint32_t SR1;					//offset 0x14
	__vo uint32_t SR2;					//offset 0x18
	__vo uint32_t CCR;					//offset 0x1C
	__vo uint32_t TRISE;				//offset 0x20
	__vo uint32_t FLTR;					//offset 0x24

}I2C_RegDef_t;

/*
 * Peripheral register definition for UART
 */
typedef struct
{
	__vo uint32_t SR;					//offset 0x00
	__vo uint32_t DR;					//offset 0x04
	__vo uint32_t BRR;					//offset 0x08
	__vo uint32_t CR1;					//offset 0x0C
	__vo uint32_t CR2;					//offset 0x10
	__vo uint32_t CR3;					//offset 0x14
	__vo uint32_t GTPR;					//offset 0x18

}USART_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5			((SPI_RegDef_t*)SPI5_BASEADDR)


#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)

/*
 * CLock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))



/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))



/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))



/*
 * CLock Enable Macros for USARTx peripheral
 */
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 6))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 <<14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))

/*
 * CLock Enable Macros for USARTx peripheral
 */
#define USART2_PCLK_DI()	(RCC->APB1ENR &= (1 << 17))
#define USART1_PCLK_DI()	(RCC->APB2ENR &= (1 << 4))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= (1 << 6))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= (1 <<14))


/*
 * Macros to reset GPIOx peripheral
 */
#define GPIOA_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()			do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Macros to reset SPIx peripheral
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))


/*
 * Returns the port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOH) ? 5 :0)

/*
 * IRQ(Interrupt Request) Number of NUCLEOF411REx MCU
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84
#define IRQ_NO_SPI5			85
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART6		71

/*
 * Macros for possible Priorities Level
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

/*
 * Some generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIOP_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/****************************************************************************
*				BIT Position definitions of SPI peripheral
 *****************************************************************************/
/*
 * BIt positions definitions of SPI_CR1 control Register
 */
#define SPI_CR1_CPHA		0			//Clock phase
#define SPI_CR1_CPOL		1			//Clock polarity
#define SPI_CR1_MSTR		2			//Master Selection
#define SPI_CR1_BR			3			//Baud Rate control
#define SPI_CR1_SPE			6			//SPI Enable
#define SPI_CR1_LSBFIRST	7			//LSB MSB type frame format
#define SPI_CR1_SSI			8			//Internal slave select, Bit has only effect when SSM bit is set.
#define SPI_CR1_SSM			9			//Software slave select [0:1] 0:SSM disable, 1: SSM enable ->>>> NSS is replaced with value from SSI bit
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE 	15

/*
 * Bit positions definitions of SPI_CR1
 */
#define SPI_CR2_RXDMAEN		0			//Rx Buffer DMA Enable
#define SPI_CR2_TXDMAEN		1			//Tx Buffer DMA Enable
#define SPI_CR2_SSOE		2			//SS OutPut Enable
#define SPI_CR2_FRF			4			//Frame Format
#define SPI_CR2_ERRIE		5			//Error Interrupt Enable
#define SPI_CR2_RXNEIE		6			//Rx Buffer not empty interrupt enable
#define SPI_CR2_TXEIE		7			//Tx Buffer not emty interrupt enable

/*
 * Bit positions definitions of SPI_SR status register
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSID		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/****************************************************************************
*				BIT Position definitions of I2C peripheral
 *****************************************************************************/
/*
 * Bit positions definitions of I2C_CR1 control Register
 */
#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

/*
 * Bit positions definitions of I2C_CR2 control Register
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

/*
 * Bit positions definitions of I2C_SR1 control Register
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/*
 * Bit positions definitions of I2C_OAR1 control Register
 */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15


/*
 * Bit positions definitions of I2C_SR2 control Register
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

/*
 * Bit positions definitions of I2C_CCR control Register
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15



/*
 * Bit position definitions of USART_CR1 control register
 */
#define USART_CR1_SBK		0			//send break
#define USART_CR1_RWU		1			//Receiver wakeup
#define USART_CR1_RE		2			//Receiver enable
#define USART_CR1_TE		3			//Transmitter enable
#define USART_CR1_IDLEIE	4			//IDLE interrupt enable
#define USART_CR1_RXNEIE	5			//RXNE interrupt enable
#define USART_CR1_TCIE		6			//Transmission Complete interrupt enable
#define USART_CR1_TXEIE		7			//TXE interrupt enable
#define USART_CR1_PEIE		8			//PE Interrupt enable
#define USART_CR1_PS		9			//Parity selection
#define USART_CR1_PCE		10			//Parity control enable
#define USART_CR1_WAKE		11			//Wakeup method
#define USART_CR1_M			12			//Word length
#define USART_CR1_UE		13			//USART enable
#define USART_CR1_OVER8		15			//Oversampling mode


/*
 * Bit position for USART_CR2 control register
 */
#define USART_CR2_ADD		0			//Address of USART Node[3:0]
#define USART_CR2_LBDL		5			//lin break detection length
#define USART_CR2_LBDIE		6			//LIN break detection interrupt enable
#define USART_CR2_LBCL		8			//Last bit clock pulse
#define USART_CR2_CPHA		9			//Clock phase
#define USART_CR2_CPOL		10			//Clock Polarity
#define USART_CR2_CLKEN		11			//Clock enable
#define USART_CR2_STOP		12			//STOP bits
#define USART_CR2_LINEN		14			//LIN mode enable

/*
 * Bit position for USART_CR3 control register
 */
#define USART_CR3_EIE   	0			//Error Interrupt enable
#define USART_CR3_IREN   	1			//IrDA mode Enable
#define USART_CR3_IRLP  	2			//IrDA low power
#define USART_CR3_HDSEL   	3			//Half-Duplex selection
#define USART_CR3_NACK   	4			//Smartcard NACK enable
#define USART_CR3_SCEN   	5			//Smartcard mode enable
#define USART_CR3_DMAR  	6			//DMA enable receiver
#define USART_CR3_DMAT   	7			//DMA enable transmitter
#define USART_CR3_RTSE   	8			//RTS enable
#define USART_CR3_CTSE   	9			//CTS enable
#define USART_CR3_CTSIE   	10			//CTS interrupt enable
#define USART_CR3_ONEBIT   	11			//One bit sample method enable

/*
 * Bit position for USART_CR3 control register
 */
#define USART_SR_PE        	0			//Parity Enable
#define USART_SR_FE			1			//Framing Error
#define USART_SR_NE        	2			//Noise Detected Flag
#define USART_SR_ORE       	3			//Overrun Error
#define USART_SR_IDLE       4			//IDLE line detected
#define USART_SR_RXNE       5			//Read data Register not empty
#define USART_SR_TC        	6			//Transmission Complete
#define USART_SR_TXE        7			//Transmit data register empty
#define USART_SR_LBD        8			//LIN break detection
#define USART_SR_CTS        9			//CTS Flag


#include "nucleof41xx_gpio_driver.h"
#include "nucleof41xx_spi_driver.h"
#include "nucleof41xx_i2c_driver.h"
#include "nucleof41xx_usart_driver.h"
#include "nucleof41xx_rcc_driver.h"

#endif /* INC_NUCLEOF41XX_H_ */
