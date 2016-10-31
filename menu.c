/*
 * menu.c
 *
 *  Created on: Oct 31, 2016
 *      Author: shuh
 */

#include "sd_globals.h"
#include "menu.h"
#include "lcd.h"

#define MENU_COUNT 3
#define CONFIG_MENU_COUNT 6

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
			g_currMenuOption = MENU_REGISTER_PHONE;
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
			g_appMode = MODE_REGISTER_ACTIVE;
			SmartDoorlockLCDDisplay(LCD_DISP_REGISTER_ACTIVE);
		}
	}
}
