/*
 * nfc.c
 *
 *  Created on: 2016. 10. 12.
 *      Author: Sokwhan
 */

//Driver lib includes
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "prcm.h"
#include "simplelink.h"
#include "uart.h"
#include "gpio.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "timer.h"

//Common interface includes
#include "common.h"
#include "timer_if.h"
#include "gpio_if.h"
#include "uart_if.h"

//Project includes
#include "nfc.h"
#include "trf797x.h"
#include "trf7970BoosterPack.h"
#include "iso15693.h"
#include "spi_l.h"

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


void NFCInit() {
    //Turn off TRF7970A CS
    SPI_TRF_CS_OFF;

	// GPIO interrupt setting
	// TRF7970 IRQ
	GPIOIntInit(GPIOA1_BASE, GPIO_PIN_4, INT_GPIOA1, Trf797xIRQ, GPIO_RISING_EDGE, INT_PRIORITY_LVL_1);

	// Set Clock Frequency and Modulation
	Trf797xCommunicationSetup();
}

unsigned int readNFCTag() {
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

	//ISO15693FindTag();	// Scan for 15693 tags
	ISO14443aFindTag();
	if(g_tag_found) {
		UART_PRINT("Tag Found \n\r");
		return 1;
	}

	osi_Sleep(400);
	return 0;
}
