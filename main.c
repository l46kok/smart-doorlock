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

// Project includes
#include "globals.h"
#include "gpio_if.h"
#include "network.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9

#define CONNECTION_TIMEOUT_COUNT  	20  /* 10sec */


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

void SmartDoorlockApp(void *pvParameters) {
	long lRetVal = -1;
	unsigned int uiConnectTimeoutCnt = 0;

	//GPIO_IF_LedConfigure(LED1|LED2|LED3);

	GPIO_IF_Set(PIN_KEYPAD_R1,1);
	GPIO_IF_Set(PIN_KEYPAD_R2,1);
	GPIO_IF_Set(PIN_KEYPAD_R3,1);
	GPIO_IF_Set(PIN_KEYPAD_R4,1);

	unsigned int rowEnum = 0;

	for (;;) {
		unsigned int test = 0;
		if (rowEnum == 0) {
			GPIO_IF_Set(PIN_KEYPAD_R1,0);
			GPIO_IF_Set(PIN_KEYPAD_R2,1);
			GPIO_IF_Set(PIN_KEYPAD_R3,1);
			GPIO_IF_Set(PIN_KEYPAD_R4,1);
		}
		else if (rowEnum == 1) {
			GPIO_IF_Set(PIN_KEYPAD_R1,1);
			GPIO_IF_Set(PIN_KEYPAD_R2,0);
			GPIO_IF_Set(PIN_KEYPAD_R3,1);
			GPIO_IF_Set(PIN_KEYPAD_R4,1);
		}
		else if (rowEnum == 2) {
			GPIO_IF_Set(PIN_KEYPAD_R1,1);
			GPIO_IF_Set(PIN_KEYPAD_R2,1);
			GPIO_IF_Set(PIN_KEYPAD_R3,0);
			GPIO_IF_Set(PIN_KEYPAD_R4,1);
		}
		else if (rowEnum == 3) {
			GPIO_IF_Set(PIN_KEYPAD_R1,1);
			GPIO_IF_Set(PIN_KEYPAD_R2,1);
			GPIO_IF_Set(PIN_KEYPAD_R3,1);
			GPIO_IF_Set(PIN_KEYPAD_R4,0);
		}
		Report("------------------------");
		Report("Row: %d", rowEnum + 1);
		test = GPIO_IF_Get(PIN_KEYPAD_C1);
		Report("C1: %d ",test);
		test = GPIO_IF_Get(PIN_KEYPAD_C2);
		Report("C2: %d ",test);
		test = GPIO_IF_Get(PIN_KEYPAD_C3);
		Report("C3: %d\n\r",test);
		/*test = GPIO_IF_Get(PIN_KEYPAD_R1);
		Report("R1: %d ",test);
		test = GPIO_IF_Get(PIN_KEYPAD_R2);
		Report("R2: %d ",test);
		test = GPIO_IF_Get(PIN_KEYPAD_R3);
		Report("R3: %d ",test);
		test = GPIO_IF_Get(PIN_KEYPAD_R4);
		Report("R4: %d \n\r",test);*/
		rowEnum++;
		if (rowEnum >= 4)
			rowEnum = 0;
		osi_Sleep(500);



/*		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_RS);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_RW);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_E);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D0);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D1);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D2);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D3);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D4);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D5);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D6);
		osi_Sleep(100);
		GPIO_IF_Toggle(PIN_LCD_D7);*/
	}

	/*lRetVal = ConnectAP("SW_PRIVATE", "ic3SolidG4me");
	do
	{
		GPIO_IF_LedOn(MCU_RED_LED_GPIO);
		osi_Sleep(250);
		GPIO_IF_LedOff(MCU_RED_LED_GPIO);
		osi_Sleep(250);
		uiConnectTimeoutCnt++;

		if (uiConnectTimeoutCnt>CONNECTION_TIMEOUT_COUNT) {
			Report("Not able to connect to AP\n\r");
			break;
		}
		Report(".");
	}
	while (!IS_CONNECTED(g_ulStatus));

	for (;;) {
		osi_Sleep(500);
		GPIO_IF_LedOff(MCU_RED_LED_GPIO);
		osi_Sleep(500);
	}*/
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
	osi_start();
	return 0;
}
