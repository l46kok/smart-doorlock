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
unsigned int g_appReady = 0;
unsigned int g_activeMode = 0;

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

static void DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t    %s  \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}


static void SmartDoorlockLCDBanner() {
	lcdReset();
	lcdInit();
	lcdDisplayOn();
	lcdClearScreen();

	lcdPutString("Smart Doorlock");
	lcdSetPosition(LCD_LINE2);
	lcdPutString("Initializing");
}

static void BoardInit(void)
{
    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


static void SmartDoorlockMenuTask(void *pvParameters) {
    SmartDoorlockLCDBanner();
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
				case CANCEL:
					Report("Disconnecting from MQTT/AP\n\r");
					Mqtt_ClientExit();
					Network_IF_DisconnectFromAP();
					Network_IF_DeInitDriver();
					Report("Exiting");
					return;
			}
			if (pressedBtn != NONE) {
				Report("Pressed: %s \n\r",btnType);
			}
		}
		osi_Sleep(50);
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
		g_tag_found = 0;
		Iso15693FindTag(); // Scan for 15693 tags

		if(g_tag_found) {
			UART_PRINT("Tag Found \n\r");
		}

		osi_Sleep(10);
	}
}


static void SmartDoorlockIoTTask(void *pvParameters) {
	int retVal = ConnectAP("SW_Private", "smartdoorlock");
	if (retVal != 0) {
		Report("Connection to AP failed!\n\r");
		return;
	}

	Report("Connection Successful!\n\r");
	g_appReady = 1;
	initMqtt();


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
    SPI_LCD_CS_OFF;

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



/*
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);
	// Start the SmartDoorlock task
	osi_TaskCreate( SmartDoorlockNFCTask,
			(const signed char*)"Smart Doorlock NFCTask",
			OSI_STACK_SIZE, NULL, 1, NULL );
*/


	// Start the SmartDoorlock task
	osi_TaskCreate( SmartDoorlockIoTTask,
			(const signed char*)"Smart Doorlock IoTTask",
			OSI_STACK_SIZE, NULL, 1, NULL );

	// Start the Keypad task
	osi_TaskCreate( SmartDoorlockMenuTask,
			(const signed char*)"MenuTask",
			OSI_STACK_SIZE, NULL, 1, NULL );
	osi_start();
	return 0;
}
