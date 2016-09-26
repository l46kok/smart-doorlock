/*
 * Smart Doorlock
 * main.c
 *
 * Created on: 2016. 8. 27.
 *
 * Author: Sokwhan Huh
 */

// C-Library includes
#include <string.h>

// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "interrupt.h"
#include "pinmux.h"
#include "utils.h"
#include "prcm.h"
#include "simplelink.h"

// Common interface include
#include "common.h"
#include "uart_if.h"
#include "gpio_if.h"

// Project includes
#include "network.h"
#include "keypad.h"
#include "lcd.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9
#define CONNECTION_TIMEOUT_COUNT  	20  /* 10sec */

//Globals
unsigned int g_appReady = 0;

static void DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t    %s  \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

static void BoardInit(void)
{
    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


void KeypadTask(void *pvParameters) {
	for (;;) {
		if (g_appReady) {
			buttonEnum pressedBtn = getPressedButton();
			char *btnType;//
			switch (pressedBtn) {
				case UP_ARROW:
					btnType = "UP";
					break;
				case LEFT_ARROW:
					btnType = "LEFT";
					break;
				case DOWN_ARROW:
					btnType = "DOWN";
					break;
				case RIGHT_ARROW:
					btnType = "RIGHT";
					break;
				case ENTER:
					btnType = "ENTER";
					break;
			}
			if (pressedBtn != NONE) {
				Report("Pressed: %s \n\r",btnType);
			}
		}
		osi_Sleep(50);
	}
}

void SmartDoorlockApp(void *pvParameters) {
	int retVal = ConnectAP("SW_Private", "smartdoorlock");
	if (retVal != 0) {
		Report("Connection to AP failed!\n\r");
		return;
	}

	Report("Connection Successful!\n\r");
	g_appReady = 1;

	lcdInit();
	unsigned long spiTest = 0;
	for (;;) {
		lcdPutChar(spiTest);
		osi_Sleep(500);
		spiTest++;
	}
}

int main(void) {
    // Initailizing the board
    BoardInit();
    // Muxing for Enabling GPIO, UART_TX and UART_RX.
    PinMuxConfig();
    // Init Terminal
    InitTerm();
    ClearTerm();
    DisplayBanner(APP_NAME);

    //Start the simplelink host
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);

	// Start the SmartDoorlock task
	osi_TaskCreate( SmartDoorlockApp,
			(const signed char*)"Smart Doorlock App",
			OSI_STACK_SIZE, NULL, 1, NULL );

	// Start the Keypad task
	osi_TaskCreate( KeypadTask,
			(const signed char*)"Keypad Task",
			OSI_STACK_SIZE, NULL, 1, NULL );
	osi_start();
	return 0;
}
