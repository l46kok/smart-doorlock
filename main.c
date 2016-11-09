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
#include "sd_globals.h"
#include "network.h"
#include "keypad.h"
#include "lcd.h"
#include "nfc.h"
#include "mqtt_client.h"
#include "spi_l.h"
#include "s_flash.h"
#include "menu.h"
#include "mcu.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9

//SD Related Defines
#define DOORLOCK_OPEN_DELAY 4000
#define PHONE_REGISTER_DELAY 6000
#define MENU_NAVIGATE_DELAY 110
#define BUZZER_DELAY 55

//Globals
unsigned int g_firstTimeSetup;
unsigned int g_appMode;
unsigned int g_currMenuOption;

static unsigned int g_nfcFirstTimeSetup = 0;

//Function Prototypes
static void SmartDoorlockNFCTask(void *pvParameters);
static void SmartDoorlockMenuTask(void *pvParameters);
static void SmartDoorlockInitTask(void *pvParameters);

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

static void SoundBuzzer(unsigned int count) {
	unsigned int i;
	for (i = 0; i < count; i++) {
		GPIO_IF_Set(PIN_BUZZER,1);
		osi_Sleep(BUZZER_DELAY);
		GPIO_IF_Set(PIN_BUZZER,0);
		osi_Sleep(BUZZER_DELAY);
	}
	GPIO_IF_Set(PIN_BUZZER,0);
}

static void OpenDoor(unsigned char *logMsg) {
	Report("Opening Doorlock\n\r");
	SmartDoorlockLCDDisplay(LCD_DISP_OPENING_DOOR);
	if (g_ConfigData.operationMode != OPER_NFC_ONLY) {
		MqttPublishLockAccess(logMsg);
	}
	g_appMode = MODE_OPENING_DOOR;
	GPIO_IF_Set(13,1);
	SoundBuzzer(1);
	osi_Sleep(DOORLOCK_OPEN_DELAY);
	GPIO_IF_Set(13,0);
	g_appMode = MODE_ACTIVE;
	SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
	Report("Closing Doorlock\n\r");
}

static void RebootSmartDoorlock() {
	g_appMode = MODE_EXIT;
	Report("Disconnecting from MQTT/AP\n\r");
/*	Mqtt_ClientExit();
	Network_IF_DisconnectFromAP();
	Network_IF_DeInitDriver();*/
	SmartDoorlockLCDDisplay(LCD_DISP_REBOOTING);
	osi_Sleep(2000);
	RebootMCU();
	Report("Rebooting");
}


static void RegisterNewPhone() {
	g_appMode = MODE_REGISTERING_PHONE;
	SmartDoorlockLCDDisplay(LCD_DISP_REGISTERING_PHONE);
	Report("Register Phone\n\r");
	strcpy(g_ConfigData.doorlockPhoneId[g_ConfigData.regDoorlockCount],nfcCmdPayload);
	strcpy(g_ConfigData.doorlockRegDate[g_ConfigData.regDoorlockCount],nfcCmdPayload2);
	Report("Writing Phone ID: %s\n\r",g_ConfigData.doorlockPhoneId[g_ConfigData.regDoorlockCount]);
	Report("Date: %s\n\r",g_ConfigData.doorlockRegDate[g_ConfigData.regDoorlockCount]);
	g_ConfigData.regDoorlockCount++;
	ManageConfigData(SF_WRITE_DATA_RECORD);
	osi_Sleep(PHONE_REGISTER_DELAY);
	if (g_firstTimeSetup) {
		if (g_ConfigData.operationMode == OPER_NFC_ONLY) {
			SmartDoorlockLCDDisplay(LCD_DISP_REBOOTING);
			osi_Sleep(2000);
			RebootMCU();
		}
		else {
			SmartDoorlockLCDDisplay(LCD_DISP_WIFI_SETUP_NFC);
			g_appMode = MODE_WIFI_CONFIG_NFC;
		}
	}
	else {
		SmartDoorlockLCDDisplay(LCD_DISP_REGISTER_ACTIVE);
		g_appMode = MODE_REGISTER_ACTIVE;
	}

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

static void NFCWifiConfig() {
	g_appMode = MODE_REGISTERING_PHONE;
	SmartDoorlockLCDDisplay(LCD_DISP_WIFI_SETUP_NFC_CONFIGURING);
	Report("Wifi Config (NFC)\n\r");
	strcpy(g_ConfigData.SSID,nfcCmdPayload);
	strcpy(g_ConfigData.Password,nfcCmdPayload2);
	Report("SSID: %s\n\r",g_ConfigData.SSID);
	Report("Password: %s\n\r",g_ConfigData.Password);
	ManageConfigData(SF_WRITE_DATA_RECORD);
	osi_Sleep(3000);
	SmartDoorlockLCDDisplay(LCD_DISP_REBOOTING);
	osi_Sleep(2000);
	RebootMCU();
}

static void SmartDoorlockMenuTask(void *pvParameters) {
	if (g_firstTimeSetup) {
		g_currMenuOption = MENU_OPERATION_SETUP;
		MenuProcessConfig(ENTER);
	}
	while (g_firstTimeSetup) {
		buttonEnum pressedBtn = getPressedButton();
		if (g_appMode == MODE_REGISTER_ACTIVE && !g_nfcFirstTimeSetup) {
			g_nfcFirstTimeSetup = 1;
			// Start the SmartDoorlock NFC task
			osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
		}
		MenuProcessConfigInner(pressedBtn);
		osi_Sleep(MENU_NAVIGATE_DELAY);
	}
	while (g_appMode != MODE_INITIALIZE_COMPLETE) {
		osi_Sleep(1);
	}

	g_currMenuOption = 0;
	Report("Initializing Menu\n\r");
	g_appMode = MODE_ACTIVE;
	SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
	/*g_appMode = MODE_MENU;
	MoveMenu(g_currMenuOption);*/
	for (;;) {
		if (g_appMode == MODE_EXIT) {
			RebootSmartDoorlock();
			return;
		}
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
		else if (g_appMode == MODE_REGISTER_ACTIVE ||
				 g_appMode == MODE_UNREGISTER_PHONE ||
				 g_appMode == MODE_OPERATION_SETUP ||
				 g_appMode == MODE_WIFI_CONFIG_NFC) {
			MenuProcessConfigInner(pressedBtn);
		}
		osi_Sleep(MENU_NAVIGATE_DELAY);
	}
}

static void SmartDoorlockNFCTask(void *pvParameters) {
    // Init NFC hardware
	Report("Initializing NFC\n\r");
    NFCInit();

    unsigned char logPayload[100];

    if (!g_firstTimeSetup) {
    	g_appMode = MODE_INITIALIZE_COMPLETE;
    }

	for (;;) {
		if (g_appMode == MODE_EXIT)
			return;
		if (g_appMode != MODE_ACTIVE && g_appMode != MODE_REGISTER_ACTIVE && g_appMode != MODE_WIFI_CONFIG_NFC) {
			osi_Sleep(1);
			continue;
		}

		nfcCmdEnum cmd = readNFCTag();
		switch (cmd) {
			case NFC_OPEN_DOORLOCK:
				if (g_appMode == MODE_ACTIVE) {
					if (IsPhoneIdRegistered(nfcCmdPayload)) {
						memset(logPayload, 0, sizeof(logPayload));
						strncpy((char*)logPayload, "LOG|NFC|", 8);
						strncat((char*)logPayload, nfcCmdPayload, sizeof(nfcCmdPayload));
						OpenDoor(logPayload);
					}
					else {
						g_appMode = MODE_UNREGISTERED_PHONE_TAPPED;
						SmartDoorlockLCDDisplay(LCD_DISP_UNREGISTERED_PHONE_TAPPED);
						osi_Sleep(3000);
						SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
						g_appMode = MODE_ACTIVE;
					}
				}
				break;
			case NFC_REG_PHONE:
				if (g_appMode == MODE_REGISTER_ACTIVE) {
					SoundBuzzer(1);
					RegisterNewPhone();
				}
				break;
			case NFC_WIFI_CONFIG:
				if (g_appMode == MODE_WIFI_CONFIG_NFC) {
					SoundBuzzer(1);
					NFCWifiConfig();
				}
				break;
			default:
				break;
		}
	}
}

static void SmartDoorlockIoTTask(void *pvParameters) {
	unsigned char logPayload[100];
	while (g_appMode == MODE_INITIALIZING) {
		osi_Sleep(1);
	}

	SmartDoorlockLCDDisplay(LCD_DISP_CONNECT_AP);

	int retVal = ConnectAP(g_ConfigData.SSID, g_ConfigData.Password);

	if (retVal != 0) {
		SmartDoorlockLCDDisplay(LCD_DISP_AP_CONN_FAILURE);
		g_ConfigData.operationMode = OPER_NFC_ONLY;
		osi_Sleep(3000);
		// Start the SmartDoorlock NFC task
		osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
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
		SmartDoorlockLCDDisplay(LCD_DISP_MQTT_CONN_FAILURE);
		g_ConfigData.operationMode = OPER_NFC_ONLY;
		osi_Sleep(3000);
		// Start the SmartDoorlock NFC task
		osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
		return;
	}


	if (g_ConfigData.operationMode == OPER_NFC_IOT) {
		// Start the SmartDoorlock NFC task
		osi_TaskCreate( SmartDoorlockNFCTask,
				(const signed char*)"Smart Doorlock NFCTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
	}
	else {
		g_appMode = MODE_INITIALIZE_COMPLETE;
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

			memset(logPayload, 0, sizeof(logPayload));
			strncpy((char*)logPayload, "LOG|IOT|SmartDoorlock", 21);
			OpenDoor(logPayload);
		}
	}
}

static void SmartDoorlockInitTask(void *pvParameters) {
    // Init LCD
	lcdInit();
	lcdClearScreen();
	SmartDoorlockLCDDisplay(LCD_DISP_INIT);

	SoundBuzzer(3);

    //Initialize simplelink
	long lMode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(lMode);

	//ManageConfigData(SF_DELETE_DATA_RECORD);
	if (ManageConfigData(SF_TEST_DATA_RECORD) < 0) {
		ManageConfigData(SF_CREATE_DATA_RECORD);
		g_firstTimeSetup = 1;
		g_appMode = MODE_OPERATION_SETUP;
		osi_Sleep(1000);
		SmartDoorlockLCDDisplay(LCD_DISP_FIRST_TIME_SETUP);
		osi_Sleep(4000);

		// Start the Menu task
		osi_TaskCreate( SmartDoorlockMenuTask,
				(const signed char*)"MenuTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
		return;
	}
	else {
		ManageConfigData(SF_READ_DATA_RECORD);
		g_firstTimeSetup = 0;
		// Start the Menu task
		osi_TaskCreate( SmartDoorlockMenuTask,
				(const signed char*)"MenuTask",
				OSI_STACK_SIZE, NULL, 1, NULL );
	}

	if (g_ConfigData.operationMode == OPER_NFC_IOT || g_ConfigData.operationMode == OPER_IOT_ONLY) {
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

    GPIO_IF_Set(PIN_BUZZER,0);

    //Start the simplelink host
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);

    //Set app mode to initializing
    g_appMode = MODE_INITIALIZING;

	// Start the SmartDoorlock Initialization Task
	osi_TaskCreate( SmartDoorlockInitTask,
			(const signed char*)"Smart Doorlock InitTask",
			OSI_STACK_SIZE, NULL, 1, NULL );


	osi_start();

	return 0;
}
