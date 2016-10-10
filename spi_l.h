/*
 * spi.h
 *
 *  Created on: Sep 30, 2016
 *      Author: shuh
 */

#ifndef SPI_L_H_
#define SPI_L_H_

//Common Interface Includes
#include "spi.h"
#include "gpio_if.h"

//#define SPI_IF_BIT_RATE  2000000
#define SPI_IF_BIT_RATE  2000000

#define SPI_TRF_CS_ON MAP_SPICSEnable(GSPI_BASE);
#define SPI_TRF_CS_OFF MAP_SPICSDisable(GSPI_BASE);

extern void SPIInit(void);

#endif /* SPI_L_H_ */
