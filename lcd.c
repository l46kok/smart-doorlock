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

unsigned long ulDummy;

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

static void lcdPutCommand(lcdCommandEnum cmdType) {
	unsigned char command = CMD_START_BIT;
	switch (cmdType) {
	case CLEAR_SCREEN:
		command |= (0 << CMD_RW) | (0 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		command = (0x01 << CMD_DATA_SHIFT_BIT);
		MAP_SPIDataPut(GSPI_BASE,command);
		command = 0x00;
		MAP_SPIDataPut(GSPI_BASE,command);
		break;
	case WRITE_CHAR:
		//RW = 0
		//RS = 1
		command |= (0 << CMD_RW) | (1 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		break;
	}
	//Clean up register.
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}

void lcdClearScreen(void) {
	lcdPutCommand(CLEAR_SCREEN);
}

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void lcdPutChar(unsigned char lcdChar) {
	MAP_SPIDataPut(GSPI_BASE,lcdChar);
	//Clean up register. Otherwise, SPI hangs here for some reason (WTF?)
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}

/*

void lcdPutChar(unsigned char lcdChar) {
	//http://www.lcd-module.de/fileadmin/eng/pdf/zubehoer/ssd1803_2.0.pdf
	//Describes how SPI is supposed to be done with this LCD
	//The level of confusion is LEGENDARY

	lcdPutCommand(WRITE_CHAR);

	//Reverse bits
	unsigned char spiData = reverse(lcdChar);
	//Get lower 4 bits (Since bit was reversed, we actually do and operation on upper 4 bit)
	unsigned char lowerData = spiData & 0xF0;
	MAP_SPIDataPut(GSPI_BASE,lowerData);
	//Clean up register. Otherwise, SPI hangs here for some reason (WTF?)
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);

	//Get upper 4 bits
	unsigned char upperData = spiData & 0x0F;
	//Shift left 4 bits
	upperData = upperData << CMD_DATA_SHIFT_BIT;
	MAP_SPIDataPut(GSPI_BASE,upperData);
	//Clean up register. Otherwise, SPI hangs here for some reason (WTF?)
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}
*/
