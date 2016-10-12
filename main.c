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
#include "gpio.h"

// Common interface include
#include "common.h"
#include "uart_if.h"
#include "gpio_if.h"

// Project includes
#include "network.h"
#include "keypad.h"
#include "lcd.h"
#include "nfc.h"
#include "mqtt_client.h"
#include "spi_l.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9
#define CONNECTION_TIMEOUT_COUNT  	20  /* 10sec */

//Globals
unsigned int g_appMode;
unsigned int g_currMenuOption;

typedef enum
{
	MODE_INITIALIZING,
	MODE_MENU,
	MODE_ACTIVE,
	MODE_CONFIG,
	MODE_OPENING_DOOR
} appModeEnum;

typedef enum
{
	MENU_ACTIVE,
	MENU_CONFIG,
	MENU_EXIT
} appMenuEnum;


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

static void OpenDoor() {
	Report("Opening Doorlock\n\r");
	SmartDoorlockLCDDisplay(LCD_DISP_OPENING_DOOR);
	g_appMode = MODE_OPENING_DOOR;
	GPIO_IF_Set(22,1);
	osi_Sleep(4000);
	GPIO_IF_Set(22,0);
	g_appMode = MODE_ACTIVE;
	SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
	Report("Closing Doorlock\n\r");
}

static void SmartDoorlockMenuTask(void *pvParameters) {
	lcdClearScreen();
	SmartDoorlockLCDDisplay(LCD_DISP_INIT);

	g_currMenuOption = 0;
	while (g_appMode == MODE_INITIALIZING) {
		osi_Sleep(1);
	}

	MoveMenu(g_currMenuOption);
	for (;;) {
		buttonEnum pressedBtn = getPressedButton();

		if (g_appMode == MODE_MENU) {
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
				else if (g_currMenuOption == MENU_EXIT) {
					SmartDoorlockLCDDisplay(LCD_DISP_EXITING_APP);
					Report("Disconnecting from MQTT/AP\n\r");
					Mqtt_ClientExit();
					Network_IF_DisconnectFromAP();
					Network_IF_DeInitDriver();
					Report("Exiting");
					return;
				}
			}
		}
		else if (g_appMode == MODE_ACTIVE) {
			if (pressedBtn == CANCEL) {
				g_appMode = MODE_MENU;
				MoveMenu(g_currMenuOption);
			}
		}
		osi_Sleep(40);
	}
}

static void SmartDoorlockNFCTask(void *pvParameters) {
	Report("Entering NFC tag read mode\n\r");

	for (;;) {
		if (g_appMode != MODE_ACTIVE) {
			osi_Sleep(1);
			continue;
		}

		unsigned int retVal = readNFCTag();
		if (retVal) {
			OpenDoor();
		}
	}
}


static void SmartDoorlockIoTTask(void *pvParameters) {
	osi_Sleep(500);
	SmartDoorlockLCDDisplay(LCD_DISP_CONNECT_AP);

	int retVal = ConnectAP("SW_Private", "smartdoorlock");

	if (retVal != 0) {
		lcdClearScreen();
		lcdPutString("Connection to AP failed!");
		Report("Connection to AP failed!\n\r");
		return;
	}

	Report("Connection Successful!\n\r");
	retVal = initMqtt();
	if (retVal != 0)
		return;

	osi_Sleep(100);
	SmartDoorlockLCDDisplay(LCD_DISP_CONNECT_MQTT);
	retVal = mqttConnect();
	osi_Sleep(500);
	if (retVal != 0) {
		lcdClearScreen();
		lcdPutString("Connection to MQTT");
		lcdSetPosition(2);
		lcdPutString("broker failed!");
		Network_IF_DisconnectFromAP();
		Network_IF_DeInitDriver();
		return;
	}
	g_appMode = MODE_INITIALIZING;

	event_msg RecvQue;
	for(;;)
	{
		osi_MsgQRead( &g_PBQueue, &RecvQue, OSI_WAIT_FOREVER);
		if (g_appMode != MODE_ACTIVE) {
			Report("IoT Task: Msg received but not in active mode\n\r");
			osi_Sleep(1);
			continue;
		}
		if(BROKER_DISCONNECTION == RecvQue.event)
		{
			attemptReconnect();
		}
		if(DOORLOCK_OPEN == RecvQue.event)
		{
			if (g_appMode == MODE_OPENING_DOOR) {
				Report("IoT Task: Doorlock is already being opened\n\r");
				continue;
			}
			OpenDoor();
		}
/*		const char *pub_topic_sw3 = "/cc3200/ButtonPressEvtSw3";
		unsigned char *data_sw2={"Push button sw2 is pressed on CC32XX device"};
		sl_ExtLib_MqttClientSend((void*)local_con_conf[0].clt_ctx,//
				pub_topic_sw3,data_sw2,strlen((char*)data_sw2),QOS2,RETAIN);
		UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
		UART_PRINT("Topic: %s\n\r","TEST");
		UART_PRINT("Data: %s\n\r","TEST");*/
	}

}


int main(void) {
    // Initailizing the board
    BoardInit();
    // Muxing for Enabling GPIO, UART_TX and UART_RX.
    PinMuxConfig();
    // Init SPI
    SPIInit();

    // Init Terminal
    InitTerm();
    ClearTerm();
    DisplayBanner(APP_NAME);

    // Init LCD
	lcdInit();

    // Init NFC hardware
    NFCInit();

    //Start the simplelink host
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);

    //Set app mode to initializing
    g_appMode = MODE_INITIALIZING;

	// Start the Keypad task
	osi_TaskCreate( SmartDoorlockMenuTask,
			(const signed char*)"MenuTask",
			OSI_STACK_SIZE, NULL, 1, NULL );

	// Start the SmartDoorlock NFC task
	osi_TaskCreate( SmartDoorlockNFCTask,
			(const signed char*)"Smart Doorlock NFCTask",
			OSI_STACK_SIZE, NULL, 1, NULL );


	// Start the SmartDoorlock IoT task
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);
	osi_TaskCreate( SmartDoorlockIoTTask,
			(const signed char*)"Smart Doorlock IoTTask",
			OSI_STACK_SIZE, NULL, 1, NULL );

	osi_start();

	return 0;
}
