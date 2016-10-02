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

static unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

static void lcdSPIPutData(unsigned char spiData) {
	//SPI Data Protocol (Weirdest crap I've ever seen in my life when it comes to SPI)
	//First, 5 start bytes are transferred (Handled in lcdputcommand).
	//Then, a byte of data is sent into two chunks. To be specific,
	//(4 Lower Bit + 0000) then (4 Upper Bit + 0000), LSB first
	//http://www.lcd-module.de/fileadmin/eng/pdf/zubehoer/ssd1803_2.0.pdf
	//The level of confusion is LEGENDARY
	unsigned char reversedData = reverse(spiData);
	//Get lower 4 bits (Since bit was reversed, we actually do and operation on upper 4 bit)
	unsigned char lowerData = reversedData & 0xF0;
	MAP_SPIDataPut(GSPI_BASE,lowerData);
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
	//Get upper 4 bits
	unsigned char upperData = (reversedData & 0x0F) << CMD_DATA_SHIFT_BIT;
	MAP_SPIDataPut(GSPI_BASE,upperData);
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}

static void lcdPutCommand(lcdCommandEnum cmdType) {
	unsigned char command = CMD_START_BIT;
	unsigned char spiData;
	switch (cmdType) {
	case CLEAR_SCREEN:
		command |= (0 << CMD_RW) | (0 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		lcdSPIPutData(0x01);
		break;
	case DISPLAY_ON:
		command |= (0 << CMD_RW) | (0 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//Display on command
		lcdSPIPutData(0x0F);
		break;
	case LCD_INIT:
		command |= (0 << CMD_RW) | (0 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//8-Bit, RE=1
		lcdSPIPutData(0x34);
		//4 Line Mode
		lcdSPIPutData(0x09);
		//8-Bit, RE=0
		lcdSPIPutData(0x30);
		break;
	case RETURN_HOME:
		command |= (0 << CMD_RW) | (0 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//Return home
		lcdSPIPutData(0x06);
		break;
	case WRITE_CHAR:
		//RW = 0
		//RS = 1
		command |= (0 << CMD_RW) | (1 << CMD_RS);
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		break;
	}

}

void lcdClearScreen(void) {
	lcdPutCommand(CLEAR_SCREEN);
	lcdPutCommand(RETURN_HOME);
}

void lcdInit(void) {
	lcdPutCommand(LCD_INIT);
}

/*
void lcdPutChar(unsigned char lcdChar) {
	MAP_SPIDataPut(GSPI_BASE,lcdChar);
	//Clean up register. Otherwise, SPI hangs here for some reason (WTF?)
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}
*/

void lcdDisplayOn(void) {
	lcdPutCommand(DISPLAY_ON);
}

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

