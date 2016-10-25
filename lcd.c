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

#define LCD_LINE1	0
#define LCD_LINE2	0x40
#define LCD_LINE3	0x14
#define	LCD_LINE4 	0x54

const unsigned char *menuList[3] = {
	"Active",
	"Configuration",
	"Exit"
};

const unsigned char *configMenuList[4] = {
	"Register Phone",
	"Unregister Phone",
	"Setup Wifi",
	"Test Wifi Conn."
};

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

void lcdSetPosition(unsigned int position) {
	if (position < 1 || position > 4)
		return;
	MAP_UARTCharPut(UARTA1_BASE,0xFE);
	MAP_UARTCharPut(UARTA1_BASE,0x45);
	switch (position) {
		case 1:
			MAP_UARTCharPut(UARTA1_BASE,LCD_LINE1);
			break;
		case 2:
			MAP_UARTCharPut(UARTA1_BASE,LCD_LINE2);
			break;
		case 3:
			MAP_UARTCharPut(UARTA1_BASE,LCD_LINE3);
			break;
		case 4:
			MAP_UARTCharPut(UARTA1_BASE,LCD_LINE4);
			break;
	}
}

void lcdClearScreen(void) {
	lcdPutCommand(CLEAR_SCREEN);
	osi_Sleep(4);
}

void lcdInit(void) {
	lcdPutCommand(LCD_INIT);
	osi_Sleep(1);
}

void lcdPutChar(unsigned char lcdChar) {
	MAP_UARTCharPut(UARTA1_BASE,lcdChar);
}

void lcdPutString(unsigned char* str) {
	osi_Sleep(1);
	do
	{
		lcdPutChar(*str++);
	}
	while(*str);

}


void SmartDoorlockLCDDisplay(sdLcdEnum lcdEnum) {
	lcdClearScreen();
	switch (lcdEnum) {
		case LCD_DISP_INIT:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Initializing");
			break;
		case LCD_DISP_CONNECT_AP:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Connecting to AP...");
			lcdSetPosition(3);
			lcdPutString("SSID: SW_Private");
			break;
		case LCD_DISP_CONNECT_MQTT:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Connecting to");
			lcdSetPosition(3);
			lcdPutString("MQTT Broker...");
			break;
		case LCD_DISP_ACTIVE:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("NFC / IoT Ready");
			break;
		case LCD_DISP_OPENING_DOOR:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Opening Door...");
			break;
		case LCD_DISP_UNREGISTERED_PHONE:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Unregistered Phone");
			break;
		case LCD_DISP_EXITING_APP:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Exiting App.");
			break;
		case LCD_DISP_REGISTERING_PHONE:
			lcdPutString("Registering phone");
			lcdSetPosition(3);
			lcdPutString("Please remove phone");
			lcdSetPosition(4);
			lcdPutString("from the doorlock");
			break;
		case LCD_DISP_REGISTER_ACTIVE:
			lcdPutString("Registering phone");
			lcdSetPosition(3);
			lcdPutString("Please place phone");
			lcdSetPosition(4);
			lcdPutString("on the doorlock");
			break;
	}
}

void MoveMenu(int menuOption) {
	lcdClearScreen();
	int i = 0;
	for (i = 0; i < MENU_COUNT; i++) {
		lcdSetPosition(i+1);
		i == menuOption ? lcdPutChar('>') : lcdPutChar(' ');
		lcdPutString((unsigned char*)menuList[i]);
	}
}

void MoveConfigMenu(int menuOption) {
	lcdClearScreen();
	int i = 0;
	for (i = 0; i < CONFIG_MENU_COUNT; i++) {
		lcdSetPosition(i+1);
		i == menuOption ? lcdPutChar('>') : lcdPutChar(' ');
		lcdPutString((unsigned char*)configMenuList[i]);
	}
}
