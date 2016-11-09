/*
 * lcd.c
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

// Driverlib includes

#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "prcm.h"
#include "simplelink.h"
#include "uart.h"

// Common Interface Includes
#include "uart_if.h"

//Project includes
#include "sd_globals.h"
#include "lcd.h"
#include "s_flash.h"

#define LCD_LINE1	0
#define LCD_LINE2	0x40
#define LCD_LINE3	0x14
#define	LCD_LINE4 	0x54

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
			lcdPutString("SSID: ");
			lcdPutString((unsigned char*)g_ConfigData.SSID);
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
			switch (g_ConfigData.operationMode) {
				case OPER_NFC_IOT:
					lcdPutString("NFC / IoT Ready");
					break;
				case OPER_NFC_ONLY:
					lcdPutString("NFC Ready");
					break;
				case OPER_IOT_ONLY:
					lcdPutString("IoT Ready");
					break;
			}
			break;
		case LCD_DISP_OPENING_DOOR:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Opening Door...");
			break;
		case LCD_DISP_NFC_DISABLED:
			lcdPutString("NFC is disabled");
			lcdSetPosition(2);
			lcdPutString("Please enable NFC");
			lcdSetPosition(3);
			lcdPutString("From Oper. Setup");
			break;
		case LCD_DISP_UNREGISTERED_PHONE_TAPPED:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Unregister Phone");
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
		case LCD_DISP_NO_PHONE_REGISTERED:
			lcdPutString("No Phone Registered");
			break;
		case LCD_DISP_FACTORY_RESET:
			lcdPutString("Factory Resetting..");
			lcdSetPosition(2);
			lcdPutString("Please do not");
			lcdSetPosition(3);
			lcdPutString("turn off the power");
			break;
		case LCD_DISP_REBOOTING:
			lcdPutString("Rebooting..");
			lcdSetPosition(2);
			lcdPutString("Please do not");
			lcdSetPosition(3);
			lcdPutString("turn off the power");
			break;
		case LCD_DISP_UNREGISTER_PHONE_SUCCESS:
			lcdPutString("Phone Unregistered");
			break;
		case LCD_DISP_FIRST_TIME_SETUP:
			lcdPutString("Welcome to");
			lcdSetPosition(2);
			lcdPutString("Smart Doorlock");
			lcdSetPosition(3);
			lcdPutString("We will walk you");
			lcdSetPosition(4);
			lcdPutString("Through Setup");
			break;
		case LCD_DISP_WIFI_SETUP_NFC:
			lcdPutString("Wifi Config (NFC)");
			lcdSetPosition(2);
			lcdPutString("Select AP from");
			lcdSetPosition(3);
			lcdPutString("Android Application");
			lcdSetPosition(4);
			lcdPutString("Then tap phone");
			break;
		case LCD_DISP_IOT_DISABLED:
			lcdPutString("IoT is disabled");
			lcdSetPosition(2);
			lcdPutString("Please enable IoT");
			lcdSetPosition(3);
			lcdPutString("From Oper. Setup");
			break;
		case LCD_DISP_WIFI_SETUP_NFC_CONFIGURING:
			lcdPutString("Configuring Wi-Fi...");
			lcdSetPosition(3);
			lcdPutString("Please remove phone");
			lcdSetPosition(4);
			lcdPutString("from the doorlock");
			break;
		case LCD_DISP_AP_CONN_FAILURE:
			lcdPutString("Failed to connect");
			lcdSetPosition(2);
			lcdPutString("to Access Point.");
			lcdSetPosition(3);
			lcdPutString("Please check your");
			lcdSetPosition(4);
			lcdPutString("Settings");
			break;
		case LCD_DISP_MQTT_CONN_FAILURE:
			lcdPutString("Connection to MQTT");
			lcdSetPosition(2);
			lcdPutString("broker failed!");
			break;
		case LCD_DISP_WIFI_TEST_LAN:
			lcdPutString("Testing Access Point");
			lcdSetPosition(2);
			lcdPutString("Connectivity...");
			break;
		case LCD_DISP_WIFI_TEST_MQTT_BROKER:
			lcdPutString("Ping Testing");
			lcdSetPosition(2);
			lcdPutString("MQTT Broker...");
			break;
		case LCD_DISP_WIFI_TEST_PASS:
			lcdPutString("Wi-Fi Testing");
			lcdSetPosition(2);
			lcdPutString("Successful");
			lcdSetPosition(3);
			lcdPutString("IoT is Available");
			break;
	}
}

