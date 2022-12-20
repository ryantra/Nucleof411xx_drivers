/*
 * nucleof41xx_rcc_driver.c
 *
 *  Created on: Nov 15, 2022
 *      Author: rupesh
 */

#include "nucleof41xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 ********************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc,temp,ahbp,apb1;

	clksrc = (RCC->CFGR >> 2) &  0x3;			//bring those 2 bits to LSB position and mask

	if(clksrc == 0)
	{
		SystemClk = 16000000;					//HSI is 16Mhz

	}else if(clksrc == 1)
	{
		SystemClk = 8000000;					//HSE is 8Mhz

	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1 = 1;
	}else
	{
		apb1 = APB_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1;

	return pclk1;

}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 *********************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{

	uint32_t SystemClock=0,pclk2, tmp;

	uint8_t ahbp,apb2;

	uint8_t clk_src = (RCC->CFGR >> 2) &  0x3;			//bring those 2 bits to LSB position and mask

	if(clk_src == 0)
	{
		SystemClock = 16000000;					//HSI is 16Mhz

	}else
	{
		SystemClock = 8000000;					//HSE is 8Mhz

	}

	tmp = ((RCC->CFGR >> 4) & 0xF);

	if(tmp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[tmp-8];
	}

	//APB2
	tmp = ((RCC->CFGR >> 13) & 0x7);

	if(tmp < 4)
	{
		apb2 = 1;
	}else
	{
		apb2 = APB_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp) / apb2;

	return pclk2;

}


uint32_t RCC_GetPLLOutputClock()
{

	return 0;
}






