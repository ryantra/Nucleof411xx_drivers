/*
 * nucleof41xx_rcc_driver.h
 *
 *  Created on: Nov 15, 2022
 *      Author: rupesh
 */

#ifndef INC_NUCLEOF41XX_RCC_DRIVER_H_
#define INC_NUCLEOF41XX_RCC_DRIVER_H_

#include "nucleof41xx.h"

//returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_NUCLEOF41XX_RCC_DRIVER_H_ */
