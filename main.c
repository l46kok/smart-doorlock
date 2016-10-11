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
#include "timer.h"

// Common interface include
#include "common.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "timer_if.h"

// Project includes
#include "network.h"
#include "keypad.h"
#include "lcd.h"
#include "mqtt_client.h"
#include "spi_l.h"
#include "trf7970.h"
#include "trf7970BoosterPack.h"
#include "iso15693.h"

#define APP_NAME             "Smart Doorlock"

//RTOS Related Defines
#define OSI_STACK_SIZE				4096 /* 2048 */
#define SPAWN_TASK_PRIORITY     	9
#define CONNECTION_TIMEOUT_COUNT  	20  /* 10sec */

//Globals
unsigned int g_appReady;
unsigned int g_openingDoor;
unsigned int g_appMode;
unsigned int g_currMenuOption;

//===============================================================
/********** GLOBAL VARIABLES TRF7970A **********/
//===============================================================
u08_t buf[300];					// TX/RX BUFFER FOR TRF7970A
u08_t g_uid[300] = "none";		// used for coping card ID
char g_tag_content[600]; 		// used for saving a content of TAG buffer
char g_block_content[200];      // used for saving a content of single/multiple block(s)

u08_t g_rssi[10];
u08_t g_tag_found = 0;          // 0->no tag found
								// 1- ISO15693 tag found
								// 2- ISO14443A tag found
								// 8 - MASTER
u08_t Tag_Count;
u08_t i_reg = 0x01;             // INTERRUPT REGISTER
u08_t irq_flag = 0x00;
u08_t rx_error_flag = 0x00;
s08_t rxtx_state = 1;           // USED FOR TRANSMIT RECEIVE BYTE COUNT
u08_t host_control_flag = 0;
u08_t stand_alone_flag = 1;

int g_tag_count;                 // Tag counter
char g_tag_count_str[10];        // string representation of tag counter

typedef enum
{
    LCD_DISP_INIT,
	LCD_DISP_CONNECT_AP,
	LCD_DISP_CONNECT_MQTT,
	LCD_DISP_ACTIVE,
	LCD_DISP_OPENING_DOOR,
	LCD_DISP_EXITING_APP
} sdLcdEnum;

typedef enum
{
	MODE_MENU,
	MODE_ACTIVE,
	MODE_CONFIG
} appModeEnum;

typedef enum
{
	MENU_ACTIVE,
	MENU_CONFIG,
	MENU_EXIT
} appMenuEnum;

#define MENU_COUNT 3
const unsigned char *menuList[3];

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

static void SmartDoorlockLCDDisplay(sdLcdEnum lcdEnum) {
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
		case LCD_DISP_EXITING_APP:
			lcdPutString("Smart Doorlock");
			lcdSetPosition(2);
			lcdPutString("Exiting App.");
		break;
	}
}

static void MoveMenu(int menuOption) {
	lcdClearScreen();
	int i = 0;
	for (i = 0; i < MENU_COUNT; i++) {
		lcdSetPosition(i+1);
		i == menuOption ? lcdPutChar('>') : lcdPutChar(' ');
		lcdPutString((unsigned char*)menuList[i]);
	}
}

static void OpenDoor() {
	Report("Opening Doorlock\n\r");
	SmartDoorlockLCDDisplay(LCD_DISP_OPENING_DOOR);
	g_openingDoor = 1;
	GPIO_IF_Set(22,1);
	osi_Sleep(3000);
	GPIO_IF_Set(22,0);
	g_openingDoor = 0;
	SmartDoorlockLCDDisplay(LCD_DISP_ACTIVE);
	Report("Closing Doorlock\n\r");
}

static void SmartDoorlockMenuTask(void *pvParameters) {
	lcdInit();
	lcdClearScreen();
	SmartDoorlockLCDDisplay(LCD_DISP_INIT);

	menuList[0] = "Active";
	menuList[1] = "Configuration";
	menuList[2] = "Exit";
	g_appMode = MODE_MENU;
	g_currMenuOption = 0;
	while (!g_appReady) {
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
		else if (g_appMode == MODE_ACTIVE && g_openingDoor == 0) {
			if (pressedBtn == CANCEL) {
				g_appMode = MODE_MENU;
				MoveMenu(g_currMenuOption);
			}
		}
		osi_Sleep(30);
	}
}

static void SmartDoorlockNFCTask(void *pvParameters) {
	Report("Entering NFC tag read mode\n\r");

	g_tag_found = 0;
	// TRF IRQ disable and clear
	IRQ_OFF;
	// TRF disable
	TRF_OFF;
	// delay at least 10 ms
	osi_Sleep(100);

	// Enter LPM3
	TRF_ON;
	// Must wait at least 4.8 mSec to allow TRF7970A to initialize.
	osi_Sleep(5);

	for (;;) {
		if (g_appMode != MODE_ACTIVE && g_openingDoor == 1)
			continue;
		g_tag_found = 0;
		Iso15693FindTag(); // Scan for 15693 tags
		Report("Scanning for tag\n\r");

		if(g_tag_found) {
			UART_PRINT("Tag Found \n\r");
			OpenDoor();
		}

		osi_Sleep(300);
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
		lcdPutString("Connection to MQTT failed!");
		return;
	}
	g_appReady = 1;
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
			if (g_openingDoor == 1) {
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

//*****************************************************************************
//
//! This function gets triggered when A2CNT interrupt occures
//!
//! \param none
//!
//! \return None
//!
//*****************************************************************************
void A2CNTIntHandler (void){
	u08_t irq_status[4];
	TimerIntClear(TIMERA2_BASE, TIMER_A); // STOP_COUNTER;

	irq_flag = 0x02;

	Trf7970ReadIrqStatus(irq_status);

	*irq_status = *irq_status & 0xF7;                // set the parity flag to 0

	if(*irq_status == 0x00 || *irq_status == 0x80)
	{
		i_reg = 0x00;                                // timer interrupt
	}
	else
	{
		i_reg = 0x01;
	}
}

int main(void) {
    // Initailizing the board
    BoardInit();
    // Muxing for Enabling GPIO, UART_TX and UART_RX.
    PinMuxConfig();
    //Init SPI
    SPIInit();
    //Turn off TRF7970A
    SPI_TRF_CS_OFF;

	// GPIO interrupt setting
	// TRF7970 IRQ
	GPIOIntInit(GPIOA1_BASE, GPIO_PIN_4, INT_GPIOA1, Trf7970PortB, GPIO_RISING_EDGE, INT_PRIORITY_LVL_1);

	// Set Clock Frequency and Modulation
	Trf7970InitialSettings();

	A2CounterInit(A2CNTIntHandler);

    // Init Terminal
    InitTerm();
    ClearTerm();
    DisplayBanner(APP_NAME);

    //Start the simplelink host
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);


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
