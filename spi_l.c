/*
 * spi.c
 *
 *  Created on: Sep 30, 2016
 *      Author: shuh
 */

//Driver Lib Includes
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "prcm.h"

// Project Includes
#include "spi_l.h"

void SPIInit(void) {
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_1,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

}
