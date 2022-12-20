/*
 *
 *  Created on: Nov 5, 2022
 *  Author: rupesh
 */

/*
 * PB14----->SPI2_MISO
 * PB15----->SPI2_MOSI
 * PB13----->SPI2_SCLK
 * PB12----->SPI2_NSS
 * ALT func Mode ---> 5
 */
#include "nucleof41xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 300000; i++);
}
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	//for SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	/*/MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);*/

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx 					= SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig 	= SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed	= SPI_SCLK_SPEED_DIV8; //2Mhz
	SPI2handle.SPIConfig.SPI_DFF 		= SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL 		= SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA 		= SPI_CPHL_LOW;
	SPI2handle.SPIConfig.SPI_SSM 		= SPI_SSM_DI;	//Hardware slave management enable for NSS pin

	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);


}


int main(void)
{
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	//function to initialize the GPIOs to behave as SPI2 PINS
	SPI2_GPIOInits();

	SPI2_Inits();

	/*
	 * For enabling NSS output
	 * NSS pins automatically managed by hardware
	 * i.e SPE=1, NSS will be pulled low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);



	while(1)
	{
		//wait till the button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

		//avoid button debouncing
		delay();

		//to enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send length information,
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//to send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//to confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//to disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}
