/*
 * File Name: trf797x.h
 *
 * Description: Headers and Defines for TRF797x Driver Functions
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef _TRF797x_H_
#define _TRF797x_H_

//===============================================================

#include "spi_for_trf.h"
#include "uart.h"
#include "uart_if.h"
#include "types.h"

#define ENABLE_STANDALONE

#define NFC_FIFO_SIZE 		120

//===============================================================

//==== TRF797x definitions ======================================

//---- Direct commands ------------------------------------------

#define IDLE						0x00
#define SOFT_INIT					0x03
#define INITIAL_RF_COLLISION		0x04
#define RESPONSE_RF_COLLISION_N		0x05
#define RESPONSE_RF_COLLISION_0		0x06
#define	RESET						0x0F
#define TRANSMIT_NO_CRC				0x10
#define TRANSMIT_CRC				0x11
#define DELAY_TRANSMIT_NO_CRC		0x12
#define DELAY_TRANSMIT_CRC			0x13
#define TRANSMIT_NEXT_SLOT			0x14
#define CLOSE_SLOT_SEQUENCE			0x15
#define STOP_DECODERS				0x16
#define RUN_DECODERS				0x17
#define CHECK_INTERNAL_RF			0x18
#define CHECK_EXTERNAL_RF			0x19
#define ADJUST_GAIN					0x1A

//---- Registers ------------------------------------------------

#define TRF797x_STATUS_CONTROL				0x00
#define TRF797x_ISO_CONTROL					0x01
#define TRF797x_ISO_14443_TX_OPTIONS		0x02
#define TRF797x_ISO_14443_BITRATE_OPTIONS	0x03
#define TRF797x_TX_TIMER_EPC_HIGH			0x04
#define TRF797x_TX_TIMER_EPC_LOW			0x05
#define TRF797x_TX_PULSE_LENGTH_CONTROL		0x06
#define TRF797x_RX_NO_RESPONSE_WAIT_TIME	0x07
#define TRF797x_RX_WAIT_TIME				0x08
#define TRF797x_MODULATOR_CONTROL			0x09
#define TRF797x_RX_SPECIAL_SETTINGS			0x0A
#define TRF797x_REGULATOR_CONTROL			0x0B
#define TRF797x_IRQ_STATUS					0x0C	// IRQ Status Register
#define TRF797x_IRQ_MASK					0x0D	// Collision Position and Interrupt Mask Register
#define	TRF797x_COLLISION_POSITION			0x0E
#define TRF797x_RSSI_LEVELS					0x0F
#define TRF797x_SPECIAL_FUNCTION_1			0x10
#define TRF797x_SPECIAL_FUNCTION_2			0x11
#define TRF797x_FIFO_IRQ_LEVELS				0x14
#define TRF797x_NFC_LOW_DETECTION			0x16
#define TRF797x_NFC_TARGET_LEVEL			0x18
#define TRF797x_NFC_TARGET_PROTOCOL			0x19
#define TRF797x_TEST_SETTINGS_1				0x1A
#define TRF797x_TEST_SETTINGS_2				0x1B
#define TRF797x_FIFO_STATUS					0x1C
#define TX_LENGTH_BYTE_1					0x1D
#define TX_LENGTH_BYTE_2					0x1E
#define FIFO								0x1F

//---- IRQ STATUS -----------------------------------------------

#define TRF797x_IRQ_STATUS_IDLE					0x00
#define TRF797x_IRQ_STATUS_NO_RESPONSE			0x01
#define TRF797x_IRQ_STATUS_COLLISION_ERROR		0x02
#define TRF797x_IRQ_STATUS_FRAMING_ERROR 		0x04
#define TRF797x_IRQ_STATUS_PARITY_ERROR 		0x08
#define TRF797x_IRQ_STATUS_CRC_ERROR	 		0x10
#define TRF797x_IRQ_STATUS_FIFO_HIGH_OR_LOW 	0x20
#define TRF797x_IRQ_STATUS_RX_COMPLETE 			0x40
#define TRF797x_IRQ_STATUS_TX_COMPLETE 			0x80

//==== End of TRF797x definitions ===============================

//===============================================================

typedef enum
{
	TRF_IDLE,
	RX_COMPLETE,
	TX_COMPLETE,
	TX_ERROR,
	RX_WAIT,
	RX_WAIT_EXTENSION,
	TX_WAIT,
	PROTOCOL_ERROR,
	NO_RESPONSE_RECEIVED,
	NO_RESPONSE_RECEIVED_15693
}tTRF797x_Status;

//===============================================================

extern void Trf797xCommunicationSetup(void);
extern void Trf797xDirectCommand(u08_t *ui8Value);
extern void Trf797xDisableSlotCounter(void);
extern void Trf797xEnableSlotCounter(void);
extern void Trf797xInitialSettings(void);
extern void Trf797xRawWrite(u08_t * pui8Payload, u08_t ui8Length);
extern void Trf797xReConfig(void);
extern void Trf797xReadCont(u08_t * pui8Payload, u08_t ui8Length);
extern u08_t Trf797xReadIsoControl(void);
extern void Trf797xReadIrqStatus(u08_t * pui8Value);
extern void Trf797xReadSingle(u08_t *pbuf, u08_t number);
extern u08_t Trf797xReadStatusControl(void);
extern void Trf797xReset(void);
extern void Trf797xResetIrqStatus(void);
extern void Trf797xRunDecoders(void);
extern void Trf797xStopDecoders(void);
extern void Trf797xTransmitNextSlot(void);
extern void Trf797xTurnRfOff(void);
extern void Trf797xTurnRfOn(void);
extern void Trf797xWriteCont(u08_t * pui8Payload, u08_t ui8Length);
extern void Trf797xWriteIsoControl(u08_t ui8IsoControl);
extern void Trf797xWriteRegister(u08_t ui8TrfRegister, u08_t ui8Value);
extern void Trf797xWriteInitiatorSetup(u08_t ui8IsoControl);
extern void Trf797xWriteSingle(u08_t *pbuf, u08_t length);

extern void Trf797xIrqWaitTimeout(long ui8TxTimeout, long ui8RxTimeout);
extern void Trf797xIrqWaitTimeoutTxOnly(u08_t ui8TxTimeout);
extern void Trf797xIrqWaitTimeoutRxOnly(u08_t ui8RxTimeout);
extern void Trf797xIrqWaitTimeoutFeliCa(void);

extern tTRF797x_Status Trf797xGetTrfStatus(void);
extern void Trf797xSetTrfStatus(tTRF797x_Status sTrfStatus);
extern u08_t Trf797xGetCollisionPosition(void);
extern void Trf797xSetCollisionPosition(u08_t ui8ColPos);

extern u08_t Trf797xGetRxBytesReceived(void);
extern u08_t Trf797xGetIsoControlValue(void);
extern u08_t Trf797xReadRssiLevels(void);
extern bool Trf797xCheckRfField(void);

extern void A2CNTIntHandler (void);
extern void Trf797xIRQ(void);
extern void StatusPrint(tTRF797x_Status sTrfStatus);

//===============================================================

#endif
