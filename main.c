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
#include "s_flash.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9

//SD Related Defines
#define DOORLOCK_OPEN_DELAY 4000
#define PHONE_REGISTER_DELAY 6000

//Globals
unsigned int g_appMode;
unsigned int g_currMenuOption;

typedef enum
{
	MODE_INITIALIZING,
	MODE_INITIALIZING_IOT,
	MODE_INITIALIZING_NFC,
	MODE_INITIALIZE_COMPLETE,
	MODE_MENU,
	MODE_ACTIVE,
	MODE_CONFIG,
	MODE_OPENING_DOOR,
	MODE_REGISTER_ACTIVE,
	MODE_REGISTERING_PHONE,
	MODE_UNREGISTERED_PHONE,
	MODE_EXIT
} appModeEnum;

typedef enum
{
	MENU_ACTIVE,
	MENU_CONFIG,
	MENU_EXIT
} appMenuEnum;

typedef enum
{
	MENU_REGISTER_PHONE,
	MENU_UNREGISTER_PHONE,
	MENU_WIFI_CONFIG,
	MENU_WIFI_TEST
} configMenuEnum;

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
	GPIO_IF_Set(13,1);
	osi_Sleep(DOORLOCK_OPEN_DELAY);
	GPIO_IF_Set(13,0);
	g_appMode = MODE_ACTIVE;
	SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
	Report("Closing Doorlock\n\r");
}

static void ExitSmartDoorlock() {
	g_appMode = MODE_EXIT;
	Report("Disconnecting from MQTT/AP\n\r");
	Mqtt_ClientExit();
	Network_IF_DisconnectFromAP();
	Network_IF_DeInitDriver();
	Report("Exiting");
}

static void MenuProcessMain(buttonEnum pressedBtn) {
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
			ExitSmartDoorlock();
			return;
		}
	}
}

static void MenuProcessConfig(buttonEnum pressedBtn) {
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

static void RegisterNewPhone() {
	g_appMode = MODE_REGISTERING_PHONE;
	SmartDoorlockLCDDisplay(LCD_DISP_REGISTERING_PHONE);
	Report("Register Phone\n\r");
	strcpy(g_ConfigData.doorlockPhoneId[g_ConfigData.regDoorlockCount],nfcCmdPayload);
	Report("Writing Phone ID: %s",g_ConfigData.doorlockPhoneId[g_ConfigData.regDoorlockCount]);
	g_ConfigData.regDoorlockCount++;
	ManageConfigData(SF_WRITE_DATA_RECORD);
	osi_Sleep(PHONE_REGISTER_DELAY);
	SmartDoorlockLCDDisplay(LCD_DISP_REGISTER_ACTIVE);
	g_appMode = MODE_REGISTER_ACTIVE;
}

static long IsPhoneIdRegistered(char *phoneId) {
	int i;
	Report("Received ID: %s\n\r",phoneId);
	for (i = 0; i < g_ConfigData.regDoorlockCount; i++) {
		Report("Comparing: %s\n\r",g_ConfigData.doorlockPhoneId[i]);
		if (strcmp(phoneId,g_ConfigData.doorlockPhoneId[i]) == 0) {
			return 1;
		}
	}
	return 0;
}

static void SmartDoorlockMenuTask(void *pvParameters) {
	g_currMenuOption = 0;
	while (g_appMode != MODE_INITIALIZE_COMPLETE) {
		osi_Sleep(1);
	}

	Report("Initializing Menu\n\r");
	g_appMode = MODE_MENU;
	MoveMenu(g_currMenuOption);
	for (;;) {
		if (g_appMode == MODE_EXIT)
			return;
		buttonEnum pressedBtn = getPressedButton();

		if (g_appMode == MODE_MENU) {
			MenuProcessMain(pressedBtn);
		}
		else if (g_appMode == MODE_ACTIVE) {
			if (pressedBtn == CANCEL) {
				g_appMode = MODE_MENU;
				MoveMenu(g_currMenuOption);
			}
		}
		else if (g_appMode == MODE_CONFIG) {
			MenuProcessConfig(pressedBtn);
		}
		else if (g_appMode == MODE_REGISTER_ACTIVE) {
			if (pressedBtn == CANCEL) {
				g_appMode = MODE_CONFIG;
				g_currMenuOption = MENU_REGISTER_PHONE;
				MoveConfigMenu(g_currMenuOption);
			}
		}
		osi_Sleep(40);
	}
}

static void SmartDoorlockNFCTask(void *pvParameters) {
    // Init NFC hardware
	Report("Initializing NFC\n\r");
    NFCInit();

	for (;;) {
		if (g_appMode == MODE_EXIT)
			return;
		if (g_appMode != MODE_ACTIVE && g_appMode != MODE_REGISTER_ACTIVE) {
			osi_Sleep(1);
			continue;
		}

		nfcCmdEnum cmd = readNFCTag();
		switch (cmd) {
			case NFC_OPEN_DOORLOCK:
				if (g_appMode == MODE_ACTIVE) {
					if (IsPhoneIdRegistered(nfcCmdPayload)) {
						OpenDoor();
					}
					else {
						g_appMode = MODE_UNREGISTERED_PHONE;
						SmartDoorlockLCDDisplay(LCD_DISP_UNREGISTERED_PHONE);
						osi_Sleep(3000);
						SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
						g_appMode = MODE_ACTIVE;
					}
				}
				break;
			case NFC_REG_PHONE:
				if (g_appMode == MODE_REGISTER_ACTIVE) {
					RegisterNewPhone();
				}
				break;
			case NFC_WIFI_CONFIG:
				Report("Wifi Config\n\r");
				break;
			default:
				break;
		}
	}
}

static void SmartDoorlockIoTTask(void *pvParameters) {
	while (g_appMode == MODE_INITIALIZING) {
		osi_Sleep(1);
	}

	SmartDoorlockLCDDisplay(LCD_DISP_CONNECT_AP);

	int retVal = ConnectAP("SW_Private", "smartdoorlock");

	if (retVal != 0) {
		lcdClearScreen();
		lcdPutString("Connection to AP failed!");
		ExitSmartDoorlock();
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
		ExitSmartDoorlock();
		return;
	}
	g_appMode = MODE_INITIALIZE_COMPLETE;

	if (g_ConfigData.nfcEnabled) {
		// Start the SmartDoorlock NFC task
		osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
	}

	event_msg RecvQue;
	for(;;)
	{
		if (g_appMode == MODE_EXIT)
			return;
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

static void SmartDoorlockInitTask(void *pvParameters) {
    // Init LCD
	lcdInit();
	lcdClearScreen();
	SmartDoorlockLCDDisplay(LCD_DISP_INIT);

    //Initialize simplelink
	long lMode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(lMode);

	//ManageConfigData(SF_DELETE_DATA_RECORD);
	if (ManageConfigData(SF_TEST_DATA_RECORD) < 0) {
		ManageConfigData(SF_CREATE_DATA_RECORD);
	}

	if (g_ConfigData.iotEnabled) {
		g_appMode = MODE_INITIALIZING_IOT;

		// Start the SmartDoorlock IoT task
	    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);
		osi_TaskCreate( SmartDoorlockIoTTask,
				(const signed char*)"Smart Doorlock IoTTask",
				OSI_STACK_SIZE, NULL, 1, NULL );

	}
	else {
		g_appMode = MODE_INITIALIZING_NFC;
		// Start the SmartDoorlock NFC task
		osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
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

    //Start the simplelink host
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);

    //Set app mode to initializing
    g_appMode = MODE_INITIALIZING;

	// Start the Keypad task
	osi_TaskCreate( SmartDoorlockMenuTask,
			(const signed char*)"MenuTask",
			OSI_STACK_SIZE, NULL, 1, NULL );

	// Start the SmartDoorlock Initialization Task
	osi_TaskCreate( SmartDoorlockInitTask,
			(const signed char*)"Smart Doorlock InitTask",
			OSI_STACK_SIZE, NULL, 1, NULL );


	osi_start();

	return 0;
}
