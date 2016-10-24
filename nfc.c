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
#include "iso14443a.h"
#include "spi_l.h"

//===============================================================
/********** GLOBAL VARIABLES TRF7970A **********/
//===============================================================


u08_t i_reg = 0x01;             // INTERRUPT REGISTER
u08_t irq_flag = 0x00;
s08_t rxtx_state = 1;           // USED FOR TRANSMIT RECEIVE BYTE COUNT
u08_t stand_alone_flag = 1;

const char *CMD_DELIM_CHAR = "|";

char nfcCmdPayload[100];

void NFCInit() {
    //Turn off TRF7970A CS
    SPI_TRF_CS_OFF;

	// GPIO interrupt setting
	// TRF7970 IRQ
	GPIOIntInit(GPIOA1_BASE, GPIO_PIN_4, INT_GPIOA1, Trf797xIRQ, GPIO_RISING_EDGE, INT_PRIORITY_LVL_1);

	// Set Clock Frequency and Modulation
	Trf797xCommunicationSetup();
}

static nfcCmdEnum parsePayload(char *ndef_content) {
	char *splitStr;
	splitStr = strtok (ndef_content,CMD_DELIM_CHAR);

	nfcCmdEnum cmd = NFC_NONE;

	int idx = 0;
	while (splitStr != NULL)
	{
		if (idx == 0) {
			if (strcmp(splitStr,"DOORLOCK_REGISTRATION") == 0) {
				cmd = NFC_REG_PHONE;
			}
			else if (strcmp(splitStr,"DOORLOCK_CONTROL") == 0) {
				cmd = NFC_OPEN_DOORLOCK;
			}
			else if (strcmp(splitStr,"WIFI_CONFIG") == 0) {
				cmd = NFC_WIFI_CONFIG;
			}
		}
		if (idx == 1) {
			strcpy (nfcCmdPayload, splitStr);
		}
		splitStr = strtok (NULL,CMD_DELIM_CHAR);
		idx++;
	}
	return cmd;
}

nfcCmdEnum readNFCTag() {
	g_ndef_content_received = 0;
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

	if(g_ndef_content_received) {
		nfcCmdEnum cmd = parsePayload(g_ndef_content);
		return cmd;
	}

	osi_Sleep(400);
	return NFC_NONE;
}
