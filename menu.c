/*
 * menu.c
 *
 *  Created on: Oct 31, 2016
 *      Author: shuh
 */

// Driverlib includes


#include "simplelink.h"

//Standard Library Includes
#include <string.h>
#include <stdio.h>

//Project Includes
#include "s_flash.h"
#include "sd_globals.h"
#include "menu.h"
#include "lcd.h"
#include "mcu.h"

#define MENU_COUNT 3
#define CONFIG_MENU_COUNT 6
#define OPER_COUNT 3

static unsigned int innerMenuOption = 0;

const unsigned char *menuList[MENU_COUNT] = {
	"Active",
	"Configuration",
	"Exit"
};

const unsigned char *configMenuList[CONFIG_MENU_COUNT] = {
	"Operation Setup",
	"Register Phone",
	"Unregister Phone",
	"Setup Wifi",
	"Test Wifi Conn.",
	"Factory Reset"
};

const unsigned char *menuOperList[OPER_COUNT] = {
	"NFC Only",
	"IoT Only",
	"NFC / IoT"
};

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
	int i;
	int menuIdx = 0;
	int menuCount = 4;

	if (menuOption >= 4) {
		menuIdx = 4;
		menuCount = CONFIG_MENU_COUNT - 4;
	}

	for (i = 0; i < menuCount; i++) {
		lcdSetPosition(i+1);
		i == (menuOption % 4) ? lcdPutChar('>') : lcdPutChar(' ');
		lcdPutString((unsigned char*)configMenuList[menuIdx]);
		menuIdx++;
	}
}

void MoveUnregisterMenu(unsigned int phoneIdx) {
	char phoneId[19];
	sprintf(phoneId,"%u: ", phoneIdx+1);

	char strippedId[16];
	memcpy(strippedId, g_ConfigData.doorlockPhoneId[phoneIdx], sizeof(strippedId));
	strncat(phoneId, strippedId, 16);

	lcdClearScreen();
	lcdPutString((unsigned char*)phoneId);
	lcdSetPosition(2);

	char date[16] = "Date: ";
	char dateStripped[10];
	memcpy(dateStripped, g_ConfigData.doorlockRegDate[phoneIdx], sizeof(dateStripped));
	strncat(date, dateStripped, 10);

	lcdPutString((unsigned char*)date);
	lcdSetPosition(3);
	lcdPutString("[Up/Down]: Select");
	lcdSetPosition(4);
	lcdPutString("[OK]: Delete");
}

void MoveOperMenu(unsigned int operMenu) {
	lcdClearScreen();
	switch (g_ConfigData.operationMode) {
		case OPER_NFC_IOT:
			lcdPutString("Current: NFC/IOT");
			break;
		case OPER_NFC_ONLY:
			lcdPutString("Current: NFC Only");
			break;
		case OPER_IOT_ONLY:
			lcdPutString("Current: IoT Only");
			break;
		case OPER_NOT_SET:
			lcdPutString("Select Operation");
			break;
	}

	int i;
	for (i = 0; i < OPER_COUNT; i++) {
		lcdSetPosition(i+2);
		i == operMenu ? lcdPutChar('>') : lcdPutChar(' ');
		lcdPutString((unsigned char*)menuOperList[i]);
	}
}


static void FactoryReset() {
	SmartDoorlockLCDDisplay(LCD_DISP_FACTORY_RESET);
	ManageConfigData(SF_DELETE_DATA_RECORD);
	osi_Sleep(2000);
	RebootMCU();
}

static void SetOperationMode() {
	g_ConfigData.operationMode = innerMenuOption;
	ManageConfigData(SF_WRITE_DATA_RECORD);
	if (g_firstTimeSetup) {
		if (g_ConfigData.operationMode == OPER_NFC_IOT || g_ConfigData.operationMode == OPER_NFC_ONLY) {
			g_appMode = MODE_REGISTER_ACTIVE;
			g_currMenuOption = MENU_REGISTER_PHONE;
			MenuProcessConfig(ENTER);
		}
	}
	else {
		SmartDoorlockLCDDisplay(LCD_DISP_REBOOTING);
		osi_Sleep(2000);
		RebootMCU();
	}
}

static void UnregisterPhone(unsigned int phoneIdx) {
	int i;
	memset(g_ConfigData.doorlockPhoneId[phoneIdx], 0, sizeof(g_ConfigData.doorlockPhoneId[phoneIdx]));
	memset(g_ConfigData.doorlockRegDate[phoneIdx], 0, sizeof(g_ConfigData.doorlockRegDate[phoneIdx]));
	for (i = phoneIdx; i < g_ConfigData.regDoorlockCount-1; i++) {
		strncpy(g_ConfigData.doorlockPhoneId[i], g_ConfigData.doorlockPhoneId[i+1], 40);
		strncpy(g_ConfigData.doorlockRegDate[i], g_ConfigData.doorlockRegDate[i+1], 10);
	}

	g_ConfigData.regDoorlockCount--;
	ManageConfigData(SF_WRITE_DATA_RECORD);
	SmartDoorlockLCDDisplay(LCD_DISP_UNREGISTER_PHONE_SUCCESS);
	osi_Sleep(1000);
}

void MenuProcessMain(buttonEnum pressedBtn) {
	if (pressedBtn == UP_ARROW && g_currMenuOption > 0) {
		g_currMenuOption--;
		MoveMenu(g_currMenuOption);
	}
	else if (pressedBtn == DOWN_ARROW && g_currMenuOption < MENU_COUNT - 1) {
		g_currMenuOption++;
		MoveMenu(g_currMenuOption);
	}
	else if (pressedBtn == ENTER) {
		if (g_currMenuOption == MENU_ACTIVE) {
			g_appMode = MODE_ACTIVE;
			SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
		}
		else if (g_currMenuOption == MENU_CONFIG) {
			g_appMode = MODE_CONFIG;
			g_currMenuOption = MENU_OPERATION_SETUP;
			MoveConfigMenu(g_currMenuOption);
		}
		else if (g_currMenuOption == MENU_EXIT) {
			SmartDoorlockLCDDisplay(LCD_DISP_EXITING_APP);
			g_appMode = MODE_EXIT;
			return;
		}
	}
}

void MenuProcessConfig(buttonEnum pressedBtn) {
	if (pressedBtn == CANCEL) {
		g_appMode = MODE_MENU;
		g_currMenuOption = MENU_CONFIG;
		MoveMenu(g_currMenuOption);
	}
	else if (pressedBtn == UP_ARROW && g_currMenuOption > 0) {
		g_currMenuOption--;
		MoveConfigMenu(g_currMenuOption);
	}
	else if (pressedBtn == DOWN_ARROW && g_currMenuOption < CONFIG_MENU_COUNT - 1) {
		g_currMenuOption++;
		MoveConfigMenu(g_currMenuOption);
	}
	else if (pressedBtn == ENTER) {
		if (g_currMenuOption == MENU_REGISTER_PHONE) {
			if (g_ConfigData.operationMode == OPER_IOT_ONLY) {
				SmartDoorlockLCDDisplay(LCD_DISP_NFC_DISABLED);
				osi_Sleep(1500);
				MoveConfigMenu(g_currMenuOption);
			}
			else {
				g_appMode = MODE_REGISTER_ACTIVE;
				SmartDoorlockLCDDisplay(LCD_DISP_REGISTER_ACTIVE);
			}
		}
		else if (g_currMenuOption == MENU_UNREGISTER_PHONE) {
			if (g_ConfigData.regDoorlockCount == 0) {
				SmartDoorlockLCDDisplay(LCD_DISP_NO_PHONE_REGISTERED);
				osi_Sleep(1500);
				MoveConfigMenu(g_currMenuOption);
				return;
			}

			innerMenuOption = 0;
			g_appMode = MODE_UNREGISTER_PHONE;
			MoveUnregisterMenu(innerMenuOption);
		}
		else if (g_currMenuOption == MENU_FACTORY_RESET) {
			FactoryReset();
			return;
		}
		else if (g_currMenuOption == MENU_OPERATION_SETUP) {
			g_appMode = MODE_OPERATION_SETUP;
			if (g_firstTimeSetup)
				innerMenuOption = OPER_NFC_ONLY;
			else
				innerMenuOption = g_ConfigData.operationMode;
			MoveOperMenu(innerMenuOption);
		}
		else if (g_currMenuOption == MENU_WIFI_CONFIG) {
			if (g_ConfigData.operationMode == OPER_NFC_ONLY) {
				SmartDoorlockLCDDisplay(LCD_DISP_IOT_DISABLED);
				osi_Sleep(2000);
			}
			else {
				SmartDoorlockLCDDisplay(LCD_DISP_WIFI_SETUP_NFC);
				g_appMode = MODE_WIFI_CONFIG_NFC;
			}
		}
	}
}

void MenuProcessConfigInner(buttonEnum pressedBtn) {
	if (!g_firstTimeSetup && pressedBtn == CANCEL) {
		g_appMode = MODE_CONFIG;
		MoveConfigMenu(g_currMenuOption);
		return;
	}

	if (g_appMode == MODE_UNREGISTER_PHONE) {
		if (pressedBtn == UP_ARROW) {
			if (innerMenuOption > 0) {
				innerMenuOption--;
			}
			MoveUnregisterMenu(innerMenuOption);
		}
		else if (pressedBtn == DOWN_ARROW) {
			if (innerMenuOption < g_ConfigData.regDoorlockCount - 1) {
				innerMenuOption++;
			}
			MoveUnregisterMenu(innerMenuOption);
		}
		else if (pressedBtn == ENTER) {
			UnregisterPhone(innerMenuOption);
			if (g_ConfigData.regDoorlockCount > 0) {
				innerMenuOption = 0;
				MoveUnregisterMenu(innerMenuOption);
			}
			else {
				g_appMode = MODE_CONFIG;
				MoveConfigMenu(g_currMenuOption);
			}
		}
		return;
	}

	if (g_appMode == MODE_OPERATION_SETUP) {
		if (pressedBtn == UP_ARROW) {
			if (innerMenuOption > 0) {
				innerMenuOption--;
			}
			MoveOperMenu(innerMenuOption);
		}
		else if (pressedBtn == DOWN_ARROW) {
			if (innerMenuOption < OPER_COUNT - 1) {
				innerMenuOption++;
			}
			MoveOperMenu(innerMenuOption);
		}
		else if (pressedBtn == ENTER) {
			SetOperationMode();
		}
		return;
	}
}


