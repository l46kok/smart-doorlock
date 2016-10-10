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
#include "uart.h"

// Common Interface Includes
#include "uart_if.h"

typedef enum
{
	LCD_INIT,
	CLEAR_SCREEN
} lcdCommandEnum;

static void lcdPutCommand(lcdCommandEnum cmdType) {

	switch (cmdType) {
	case CLEAR_SCREEN:
		MAP_UARTCharPut(UARTA1_BASE,0xFE);
		MAP_UARTCharPut(UARTA1_BASE,0x51);
		break;
	case LCD_INIT:
	    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(CONSOLE_PERIPH),
	                    9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                     UART_CONFIG_PAR_NONE));
	    osi_Sleep(5);
		MAP_UARTCharPut(UARTA1_BASE,0xFE);
		MAP_UARTCharPut(UARTA1_BASE,0x41);
		break;
	}
}

void lcdSetPosition(unsigned char position) {
	MAP_UARTCharPut(UARTA1_BASE,0xFE);
	MAP_UARTCharPut(UARTA1_BASE,0x45);
	MAP_UARTCharPut(UARTA1_BASE,position);
}

void lcdClearScreen(void) {
	lcdPutCommand(CLEAR_SCREEN);
	osi_Sleep(3);
}

void lcdInit(void) {
	lcdPutCommand(LCD_INIT);
	osi_Sleep(1);
}

void lcdPutChar(unsigned char lcdChar) {
	MAP_UARTCharPut(UARTA1_BASE,lcdChar);
}

void lcdPutString(unsigned char* str) {
	do
	{
		lcdPutChar(*str++);
	}
	while(*str);
}
