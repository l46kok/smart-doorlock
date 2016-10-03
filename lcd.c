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
#include "simplelink.h"

// Common interface include
#include "spi.h"

// Project Includes
#include "spi_l.h"

#define CMD_START_BIT 0xF8 //5 1s for start bit
#define CMD_RW 2
#define CMD_RS 1
#define CMD_DATA_SHIFT_BIT 4

#define LCD_HOME_L1	0x80

unsigned long ulDummy;

static unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void lcdCheckBusy(void) {
	//For some reason, checkbusy bugs (Causes LCD to lock up)?
	//LCD seems to work fine without it... for now

/*	unsigned char command = CMD_START_BIT;
	unsigned char isBusy = 1;
	command |= (1 << CMD_RW) | (0 << CMD_RS);

	do {
		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		isBusy = (char)ulDummy;
		SPI_LCD_CS_OFF;
	}
	while (isBusy & 0x01 > 0);*/
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
	command |= (0 << CMD_RW) | (0 << CMD_RS);
	switch (cmdType) {
	case CLEAR_SCREEN:
		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		lcdSPIPutData(0x01);
		SPI_LCD_CS_OFF;
		break;
	case DISPLAY_ON:
		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//Display on command
		//lcdSPIPutData(0x0F);
		lcdSPIPutData(0x0C);
		SPI_LCD_CS_OFF;
		break;
	case LCD_INIT:
		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//8-Bit, RE=1
		lcdSPIPutData(0x34);
		SPI_LCD_CS_OFF;
		osi_Sleep(2);

		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//4 Line Mode
		lcdSPIPutData(0x09);
		SPI_LCD_CS_OFF;
		osi_Sleep(2);


		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//8-Bit, RE=0
		lcdSPIPutData(0x30);
		SPI_LCD_CS_OFF;
		break;
	case RETURN_HOME:
		SPI_LCD_CS_ON;
		MAP_SPIDataPut(GSPI_BASE,command);
		MAP_SPIDataGet(GSPI_BASE,&ulDummy);
		//Return home
		lcdSPIPutData(0x06);
		SPI_LCD_CS_OFF;
		break;
	}
}

void lcdSetPosition(unsigned char position) {
	lcdCheckBusy();
	SPI_LCD_CS_ON;
	unsigned char command = CMD_START_BIT;
	MAP_SPIDataPut(GSPI_BASE,command);
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
	lcdSPIPutData(LCD_HOME_L1 + position);
	SPI_LCD_CS_OFF;
}

void lcdClearScreen(void) {
	lcdCheckBusy();
	lcdPutCommand(CLEAR_SCREEN);
	lcdPutCommand(RETURN_HOME);
	osi_Sleep(2);
}

void lcdInit(void) {
	lcdPutCommand(LCD_INIT);
	osi_Sleep(2);
}

void lcdReset(void) {
	//Reset LCD
	GPIO_IF_Set(13,0);
	osi_Sleep(50);
	GPIO_IF_Set(13,1);
}

void lcdDisplayOn(void) {
	lcdCheckBusy();
	lcdPutCommand(DISPLAY_ON);
	osi_Sleep(2);
}

void lcdPutChar(unsigned char lcdChar) {
	//RW = 0
	//RS = 1
	SPI_LCD_CS_ON;
	unsigned char command = CMD_START_BIT;
	command |= (0 << CMD_RW) | (1 << CMD_RS);
	MAP_SPIDataPut(GSPI_BASE,command);
	MAP_SPIDataGet(GSPI_BASE,&ulDummy);
	lcdSPIPutData(lcdChar);
	SPI_LCD_CS_OFF;
}

void lcdPutString(unsigned char* str) {
	do
	{
		lcdCheckBusy();
		lcdPutChar(*str++);
	}
	while(*str);
}
