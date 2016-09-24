/*
 * lcd.c
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */
#include "lcd.h"

// Driverlib includes

#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "prcm.h"

// Common interface include
#include "spi.h"

void lcdInit(void) {
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

    // Enable Chip select
    MAP_SPICSEnable(GSPI_BASE);
}

void lcdPutChar(char lcdChar) {
	unsigned long userData = (long)lcdChar;
	unsigned long ulDummy;

	MAP_SPIDataPut(GSPI_BASE,userData);
	//Clean up register. Otherwise, SPI hangs here for some reason (WTF?)
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}
