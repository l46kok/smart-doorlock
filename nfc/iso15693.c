/*
 * {iso15693.c}
 *
 * {ISO15693 Specific Functions & Anti-collision}
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#include "hw_types.h"
#include <string.h>
#include <stdio.h>
#include "iso15693.h"
#include "trf7970BoosterPack.h"
#include "uart_if.h"
#include "timer.h"
#include "timer_if.h"
#include "utils.h"
#include "gpio.h"
#include "gpio_if.h"
#include "hw_memmap.h"
#include "uart_if.h"
#include "osi.h"

//===============================================================
//		Global Variables
//===============================================================

extern uint8_t g_ui8TrfBuffer[NFC_FIFO_SIZE];
static volatile tTRF797x_Status g_sTrfStatus;

static uint8_t g_pui8Iso15693UId[8];
static uint8_t g_pui8AnticollisionMaskBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t g_ui8TagDetectedCount = 0;

//===============================================================
//
// Iso15693_SingleSlotInventory - Issue a single slot Inventory
// command for ISO15693 tags.
//
// This function issues a single slot Inventory command for tag
// detection of ISO15693 tags. If a tag is found, the UID is
// stored in the g_pui8Iso15693UId buffer.
//
// If UART is enabled, the tag ID is sent out to a host via UART
// as well.
//
// \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
// to indicate if the Inventory command resulted in a successful
// tag detection or not.
//
//===============================================================

uint8_t Iso15693_SingleSlotInventory(void)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8LoopCount = 0;
	uint8_t ui8Status = STATUS_FAIL;
	uint8_t ui8RssiLevel;

	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x30;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x26;		// ISO15693 flags
	g_ui8TrfBuffer[ui8Offset++] = 0x01;		// Inventory command code
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Mask Length = 0 (Also not sending AFI)

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the ISO15693 Inventory Command

	Trf797xIrqWaitTimeout(5,15);			// 5 millisecond TX timeout, 15 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			ui8Status = STATUS_SUCCESS;

			ui8RssiLevel = Trf797xReadRssiLevels();	// Read the RSSI levels

			// UID Starts at the 3rd received bit (1st is flags and 2nd is DSFID)
			for (ui8LoopCount = 2; ui8LoopCount < 10; ui8LoopCount++)
			{
				g_pui8Iso15693UId[ui8LoopCount-2] = g_ui8TrfBuffer[ui8LoopCount];	// Store UID into a Buffer
			}

			g_ui8TagDetectedCount++;

			// Print out UID and RSSI level to UART Host
			UartPutCrlf();
			UartSendCString("ISO15693/NFC-V  UID:  ");
			UartPutChar('[');
			for (ui8LoopCount = 0; ui8LoopCount < 8; ui8LoopCount++)
			{
				UartPutByte(g_pui8Iso15693UId[7-ui8LoopCount]);		// Send UID to host
			}
			UartPutChar(']');
			UartPutCrlf();
			UartSendCString("RSSI LEVEL:  ");
			UartPutChar('[');
			UartPutByte(ui8RssiLevel);		// Send RSSI level to host
			UartPutChar(']');
			UartPutCrlf();

		}
	}
	else
	{
		ui8Status = STATUS_FAIL;
	}


	// clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}

//===============================================================
//
// Iso15693_Anticollision - Issue an Inventory command for either
// 1 or 16 slot anticollision of ISO15693 tags.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
// \param pui8Mask are the masked nibbles to issue with the
// Inventory command
// \param ui8MaskLength is the number of significant bits in the
// mask value.
// \param ui8Afi is the AFI to be issued with command (if AFI
// flag is included in ui8ReqFlag)
//
// This function issues a single or sixteens lot Inventory
// command for tag detection of ISO15693 tags. If a tag is found,
// the UID is stored in the g_pui8Iso15693UId buffer. The process
// will run until all ISO15693 detected have responded with their
// UID's.
//
// The function uses a recursive call for the anticollision
// process. Since the UID is stored inside of a buffer, only
// the last ISO15693 tag to respond with it's UID will have
// that UID stored in order to read data from the tag.
//
// If UART is enabled, the UID of each ISO15693 tag detected is
// sent out to a host via UART.
//
// \return ui8Status returns STATUS_SUCCESS if the anticollision
// function resulted in a successful tag detection. Otherwise,
// returns STATUS_FAIL.
//
//===============================================================

uint8_t Iso15693_Anticollision(uint8_t ui8ReqFlags, uint8_t ui8MaskLength, uint8_t ui8Afi)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8LoopCount1 = 1;
	uint8_t ui8LoopCount2 = 1;
	uint8_t ui8SlotCount;
	uint16_t ui16TransmitByteLength;
	uint16_t ui16SlotNumber = 0x0000;
	uint8_t ui8MaskValue;
	uint8_t ui8MaskByteCount;
	uint8_t ui8Status = STATUS_FAIL;

	if ((ui8ReqFlags & BIT5) == 0x00)	// Check if Bit 5 in the Request Flag to determine the number of slots
	{
		ui8SlotCount = 16;				// If Bit 5 is cleared, then use 16 slots
	}
	else
	{
		ui8SlotCount = 1;				// If Bit 5 is set, then use 1 slot
	}

	ui8MaskByteCount = (((ui8MaskLength >> 2) + 1) >> 1);	// Set ui8MaskByteCount based on the inputted Mask Length
															// ui8MaskByteCount will be 1 for length = 4 or 8,
															// ui8MaskByteCount will be 2 for length = 12 or 16,
															// and so on

	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	// Format Anti-collision command packet
	g_ui8TrfBuffer[ui8Offset++] = 0x8F;					// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;					// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;					// Write Continuous

	// Calculate how long the output byte will be
	if (ui8ReqFlags & 0x10) 							// Check if AFI will be included or not
	{
		ui16TransmitByteLength = ui8MaskByteCount + 4;	// Set packet size = Mask Value + Mask Length + AFI Byte + ISO15693 Command Code + ISO15693 Request Flags
	}
	else
	{
		ui16TransmitByteLength = ui8MaskByteCount + 3;	// Set packet size = Mask Value + Mask Length + ISO15693 Command Code + ISO15693 Request Flags
	}

	g_ui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16TransmitByteLength >> 8);		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16TransmitByteLength << 4);		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlags;			// ISO15693 Request Flags
	g_ui8TrfBuffer[ui8Offset++] = 0x01;					// Inventory Request Command Code

	if (ui8ReqFlags & 0x10)								// Check if AFI will be included or not
	{
		g_ui8TrfBuffer[ui8Offset++] = ui8Afi;			// Optional AFI Byte
		g_ui8TrfBuffer[ui8Offset++] = ui8MaskLength;	// Mask Length
		if (ui8MaskLength > 0)
		{
			for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8MaskByteCount; ui8LoopCount1++)
			{
				g_ui8TrfBuffer[ui8Offset++] = g_pui8AnticollisionMaskBuffer[(ui8MaskByteCount-ui8LoopCount1)];		// Fill in inputted Mask Values
			}
		}
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = ui8MaskLength;	// Mask Length
		if (ui8MaskLength > 0)
		{
			for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8MaskByteCount; ui8LoopCount1++)
			{
				g_ui8TrfBuffer[ui8Offset++] = g_pui8AnticollisionMaskBuffer[((ui8MaskByteCount-1)-ui8LoopCount1)];		// Fill in inputted Mask Values
			}
		}
	}

	Trf797xEnableSlotCounter();

	Trf797xResetIrqStatus();

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the ISO15693 Inventory Command

	Trf797xIrqWaitTimeoutTxOnly(5);				// 5 millisecond TX timeout

	for (ui8LoopCount2 = 1; ui8LoopCount2 <= ui8SlotCount; ui8LoopCount2++)		// There will be either 1 or 16 available time slots
	{
		Trf797xIrqWaitTimeoutRxOnly(15);		// 15 millisecond RX timeout

		g_sTrfStatus = Trf797xGetTrfStatus();	// Get the TRF797x Status

		switch (g_sTrfStatus)
		{
		case RX_COMPLETE:						// If data has been received, then UID is in the buffer
			if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
			{
				ui8Status = STATUS_SUCCESS;

				// UID Starts at the 3rd received bit (1st is flags and 2nd is DSFID)
				for (ui8LoopCount1 = 2; ui8LoopCount1 < 10; ui8LoopCount1++)
				{
					g_pui8Iso15693UId[ui8LoopCount1-2] = g_ui8TrfBuffer[ui8LoopCount1];	// Store UID to a Buffer
				}

				g_ui8TagDetectedCount++;

				// Print out UID and RSSI level to UART Host
				UartPutCrlf();
				UartSendCString("ISO15693/NFC-V  UID:  ");
				UartPutChar('[');
				for (ui8LoopCount1 = 0; ui8LoopCount1 < 8; ui8LoopCount1++)
				{
					UartPutByte(g_pui8Iso15693UId[7-ui8LoopCount1]);		// Send UID to host
				}
				UartPutChar(']');
				UartPutCrlf();

			}
			break;

		case PROTOCOL_ERROR:		// A collision has occurred for this slot
			ui16SlotNumber |= (0x01 << (ui8LoopCount2-1));	// Mark a collision occurred in the correct Slot Number bit.
			McuDelayMillisecond(5);	// Allow time for tag to finish responding before issuing EOF
			break;

		case NO_RESPONSE_RECEIVED:	// No Response was received, break out of the switch statement as there is no tag present
			break;

		case NO_RESPONSE_RECEIVED_15693:	// No Response was received, break out of the switch statement as there is no tag present
			break;

		default:
			break;
		}

		Trf797xReset();				// FIFO has to be reset before receiving the next response

		if ((ui8SlotCount == 16) && (ui8LoopCount2 < 16))		// If 16 slots used, and the last slot as not been reached, then send EOF (i.e. next slot indicator)
		{
			Trf797xStopDecoders();
			Trf797xRunDecoders();
			Trf797xTransmitNextSlot();
		}
		else if ((ui8SlotCount == 16) && (ui8LoopCount2 == 16))	// Once at the end of slot 16, then stop the slot counter
		{
			Trf797xStopDecoders();
			Trf797xDisableSlotCounter();
		}
		else if (ui8SlotCount == 1)								// Only 1 slot to be used, no more anti-collision to be performed
		{
			break;
		}
	}

	Trf797xDisableSlotCounter();

	ui8MaskLength = ui8MaskLength + 4; 						// The mask length is a multiple of 4 bits

	ui8MaskByteCount = (((ui8MaskLength >> 2) + 1) >> 1);	// Set ui8MaskByteCount based on the inputted Mask Length
															// ui8MaskByteCount is 1 for length = 4 or 8,
															// ui8MaskByteCount is 2 for length = 12 or 16,
															// and so on

	// If the slot number pointer is not 0, the slot count is 16 (to indicate anticollision is needed),
	// the mask length doesn't exceed 60 bits, and the slot number is not 16 then proceed to recursive function call
	while ((ui16SlotNumber != 0x00)
			&& (ui8SlotCount == 16)
			&& (ui8MaskLength < 61))
	{
		ui8MaskValue = 0x00;
		ui8LoopCount1 = 0;

		while (ui8LoopCount1 < 16)
		{
			if ((ui16SlotNumber & (0x01 << ui8LoopCount1)) != 0x00)
			{
				ui8MaskValue = ui8LoopCount1;

				ui16SlotNumber &= ~(0x01 << ui8LoopCount1); 				// Clear that slot bit from the array

				break;
			}
			ui8LoopCount1++;
		}

		if ((ui8MaskLength & 0x04) == 0x00)
		{
			ui8MaskValue = ui8MaskValue << 4;								// Shift slot pointer if mask length doesn't have Bit 2 (0x04) set (since it is a multiple of 4 bits)
		}
		else
		{																	// Otherwise re-copy the mask values
			for (ui8LoopCount1 = 7; ui8LoopCount1 > 0; ui8LoopCount1--)
			{
				g_pui8AnticollisionMaskBuffer[ui8LoopCount1] = g_pui8AnticollisionMaskBuffer[ui8LoopCount1 - 1];
			}
			g_pui8AnticollisionMaskBuffer[0] &= 0x00;									// And set the mask value for the first byte in the array = 0
		}

		g_pui8AnticollisionMaskBuffer[0] |= ui8MaskValue;								// Now update the mask value of the first byte based on the slot number pointer

		McuDelayMillisecond(2);

		ui8Status = Iso15693_Anticollision(ui8ReqFlags, ui8MaskLength, ui8Afi); // Issue a recursive call with new Mask

		// Restore the Global AnticollisionMaskBuffer with the values from the current anticollision function.
		if ((ui8MaskLength & 0x04) == 0x00)
		{
			// If mask length doesn't have Bit 2 (0x04) set (since it is a multiple of 4 bits) - clear the upper nibble which is where the new mask value was placed
			g_pui8AnticollisionMaskBuffer[0] &= 0x0F;
		}
		else
		{	// Otherwise re-shift the mask values
			for (ui8LoopCount1 = 0; ui8LoopCount1 < 7; ui8LoopCount1++)
			{
				g_pui8AnticollisionMaskBuffer[ui8LoopCount1] = g_pui8AnticollisionMaskBuffer[ui8LoopCount1 + 1];
			}
			g_pui8AnticollisionMaskBuffer[7] = 0x00;									// And set the mask value for the first byte in the array = 0
		}
	}



	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}

//===============================================================
//
// Iso15693_GetSystemInfo - Issue the Get System Information
// command for ISO15693 tags.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
//
// This function issues a Get System Information command for
// ISO15693 tags. This can be used to determine how many blocks
// of data can be read from the tag.
//
// If UART is enabled, the contents of the Get System Information
// response is sent out to a host via UART as well.
//
// \return ui16NumberOfBlocks returns the number of blocks
// contained in the ISO15693 tag.
//
//===============================================================

uint16_t Iso15693_GetSystemInfo(uint8_t ui8ReqFlag)
{
	uint8_t ui8Offset = 0;
	uint16_t ui16NumberOfBlocks = 0x00;

	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;


	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0xA0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag;	// ISO15693 flags
	g_ui8TrfBuffer[ui8Offset++] = 0x2B;			// Get System Information command code

	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	Trf797xIrqWaitTimeout(5,15);			// 5 millisecond TX timeout, 15 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{

			// Output received data to UART
			UartSendCString("Get Sys Info ");
			UartSendCString("Data:  ");
			UartPutChar('[');

			ui8RxLength = Trf797xGetRxBytesReceived();

			for (ui8LoopCount = 1; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UartPutByte(g_ui8TrfBuffer[ui8LoopCount]);		// Send Get System Info data to host
			}

			UartPutChar(']');
			UartPutCrlf();

			// Check to see that no error flags were sent and that there is a block number data available
			if (g_ui8TrfBuffer[0] == 0x00 && ((g_ui8TrfBuffer[1] & 0x07) == 0x07))
			{
				ui16NumberOfBlocks = g_ui8TrfBuffer[12];
			}
			else if (g_ui8TrfBuffer[0] == 0x00 && (((g_ui8TrfBuffer[1] & 0x07) == 0x06) || ((g_ui8TrfBuffer[1] & 0x07) == 0x05)))
			{
				ui16NumberOfBlocks = g_ui8TrfBuffer[11];
			}
			else if (g_ui8TrfBuffer[0] == 0x00 && ((g_ui8TrfBuffer[1] & 0x07) == 0x04))
			{
				ui16NumberOfBlocks = g_ui8TrfBuffer[10];
			}
		}
	}
	else if ((g_sTrfStatus == NO_RESPONSE_RECEIVED) || (g_sTrfStatus == NO_RESPONSE_RECEIVED_15693))
	{
		// Case for TI HF-I Pro and Standard tags
		ui16NumberOfBlocks = 0x0A;
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui16NumberOfBlocks;
}

//===============================================================
//
// Iso15693_GetSystemInfoExtended - Issue the Get System
// Information command for ISO15693 tags with the protocol
// extention flag set.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
//
// This function issues a Get System Information command for
// ISO15693 tags with the Protocol Extension bit set in the
// request flags. This can be used to determine how many blocks
// of data can be read from the tag.
//
// If UART is enabled, the contents of the Get System Information
// response is sent out to a host via UART as well.
//
// \return ui16NumberOfBlocks returns the number of blocks
// contained in the ISO15693 tag.
//
//===============================================================

uint16_t Iso15693_GetSystemInfoExtended(uint8_t ui8ReqFlag)
{
	uint8_t ui8Offset = 0;
	uint16_t ui16NumberOfBlocks = 0x00;

	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;

	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0xA0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag | 0x08;		// ISO15693 flags + protocol extension bit
	g_ui8TrfBuffer[ui8Offset++] = 0x2B;			// Get System Information command code

	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	Trf797xIrqWaitTimeout(5,20);	// 5 millisecond TX timeout, 15 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{

			UartSendCString("Get Sys Info ");
			UartSendCString("Data:  ");
			UartPutChar('[');

			ui8RxLength = Trf797xGetRxBytesReceived();

			// Output received data to UART
			for (ui8LoopCount = 1; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UartPutByte(g_ui8TrfBuffer[ui8LoopCount]);		// Send Get System Info data to host
			}

			UartPutChar(']');
			UartPutCrlf();

			ui16NumberOfBlocks = 0x0800;		// Set block size in order to read STM M24LR64 tag
		}
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui16NumberOfBlocks;
}

//===============================================================
//
// Iso15693_ReadSingleBlock - Issue the Read Single Block command
// for ISO15693 tags.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
// \param ui8BlockNumber is the block number to read data from.
//
// This function issues a Read Single Block with the specified
// request flags and block number to read data from an ISO15693
// tag.
//
// If UART is enabled, the data read from the ISO15693 tag is
// sent out to a host via UART as well.
//
// \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
// to indicate if the Read Single Block was successful or not.
//
//===============================================================

uint8_t Iso15693_ReadSingleBlock(uint8_t ui8ReqFlag, uint8_t ui8BlockNumber)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;

	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;


	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0xB0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x30;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}

	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag;	// ISO15693 flags
	g_ui8TrfBuffer[ui8Offset++] = 0x20;			// Read Single Block command code

	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	g_ui8TrfBuffer[ui8Offset++] = ui8BlockNumber;		// Block # (variable, for HF-I Plus device can go to 0x3F, Pro and Standard handled with "error" response flags)

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	Trf797xIrqWaitTimeout(5,15);		// 5 millisecond TX timeout, 15 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{

			UartSendCString("NFC-V Block ");
			UartPutByte(ui8BlockNumber);		// Output block number
			UartSendCString(" Data:  ");
			UartPutChar('[');

			ui8RxLength = Trf797xGetRxBytesReceived();

			if (ui8ReqFlag & BIT6) // Handle case for Option Flag causing one extra byte to be transmitted.
			{
				ui8Offset = 2;
			}
			else
			{
				ui8Offset = 1;
			}

			// Output received data to UART
			for (ui8LoopCount = ui8Offset; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UartPutByte(g_ui8TrfBuffer[ui8LoopCount]);		// Send out data read from tag to host
			}

			UartPutChar(']');
			UartPutCrlf();

			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{

			// 	Indicates when an error occurs or block addresses are unreachable - useful for debugging
			UartSendCString("NFC-V Block ");
			UartPutByte(ui8BlockNumber);			// Output block number
			UartSendCString(" Error");
			UartPutCrlf();
			UartSendCString("ISO15693 Error Code:  ");
			UartPutByte(g_ui8TrfBuffer[1]);		// Output ISO15693 error code
			UartPutCrlf();

			// Response with error
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// No response
		ui8Status = STATUS_FAIL;
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}

//===============================================================
//
// Iso15693_ReadMultipleBlocks - Issue the Read Multiple Block
// command for ISO15693 tags.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
// \param ui8FirstBlock is the starting block number to read data
// from.
// \param ui8NumberOfBlocks is the amount of blocks to read data
// from.
//
// This function issues a Read Multiple Block with the specified
// request flags, the starting block number, and the number
// blocks to read data from an ISO15693 tag.
//
// If UART is enabled, the data read from the ISO15693 tag is
// sent out to a host via UART as well.
//
// \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
// to indicate if the Read Single Block was successful or not.
//
//===============================================================

uint8_t Iso15693_ReadMultipleBlocks(uint8_t ui8ReqFlag, uint8_t ui8FirstBlock, uint8_t ui8NumberOfBlocks)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;

	uint8_t ui8LoopCount1 = 0;
	uint8_t ui8LoopCount2 = 0;
	uint8_t ui8RxLength = 0;
	uint8_t ui8BlockSize = 0;

	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0xC0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x40;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}

	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag;	// ISO15693 flags
	g_ui8TrfBuffer[ui8Offset++] = 0x23;			// Read Multiple Block command code

	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	g_ui8TrfBuffer[ui8Offset++] = ui8FirstBlock;			// Number of the first block to read from

	if (ui8NumberOfBlocks > 0)
	{
		g_ui8TrfBuffer[ui8Offset++] = ui8NumberOfBlocks-1;	// Index for number of blocks to be read - this value is one less than
	}
	else
	{
		// Invalid count provided
		return ui8Status = STATUS_FAIL;
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	Trf797xIrqWaitTimeout(5,15+ui8NumberOfBlocks);		// 5 millisecond TX timeout, 15 millisecond RX timeout - adding number of blocks to extend timeout for larger read requests

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{

			ui8RxLength = Trf797xGetRxBytesReceived();

			if (ui8ReqFlag & BIT6) // Handle case for Option Flag causing one extra byte to be transmitted.
			{
				ui8Offset = 2;
			}
			else
			{
				ui8Offset = 1;
			}

			ui8LoopCount1 = ui8RxLength-ui8Offset;

			while (ui8LoopCount1 > 0)
			{
				if (ui8LoopCount1 > ui8NumberOfBlocks)
				{
					ui8LoopCount1 = ui8LoopCount1 - ui8NumberOfBlocks;
				}
				else
				{
					ui8LoopCount1 = 0;
				}
				ui8BlockSize++;
			}

			for (ui8LoopCount2 = 0; ui8LoopCount2 < ui8NumberOfBlocks; ui8LoopCount2++)
			{
				UartSendCString("NFC-V Block ");
				UartPutByte(ui8FirstBlock+ui8LoopCount2);		// Output block number
				UartSendCString(" Data:  ");
				UartPutChar('[');

				// Output received data to UART
				for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8BlockSize; ui8LoopCount1++)
				{
					UartPutByte(g_ui8TrfBuffer[ui8Offset++]);		// Send out data read from tag to host
				}

				UartPutChar(']');
				UartPutCrlf();
			}

			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{
			// 	Indicates when an error occurs or block addresses are unreachable - useful for debugging
			UartSendCString("NFC-V Block ");
			UartPutByte(ui8FirstBlock);			// Output block number
			UartSendCString(" Error");
			UartPutCrlf();
			UartSendCString("ISO15693 Error Code:  ");
			UartPutByte(g_ui8TrfBuffer[1]);		// Output ISO15693 error code
			UartPutCrlf();

			// Response with error
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// No response
		ui8Status = STATUS_FAIL;
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}


//===============================================================
//
// Iso15693_ReadSingleBlockExtended - Issue the Read Single Block
// command for ISO15693 tags with the protocol extention flag set
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
// \param ui16BlockNumber is the block number to read data from.
//
// This function issues a Read Single Block with the block number
// and the specified request flags, including the Protocol
// Extension bit, to read data from ISO15693 tags which require
// the use of extended protocol commands.
//
// If UART is enabled, the data read from the ISO15693 tag is
// sent out to a host via UART as well.
//
// \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
// to indicate if the Read Multiple Block was successful or not.
//
//===============================================================

uint8_t Iso15693_ReadSingleBlockExtended(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;

	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;


	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0xC0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x40;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag | 0x08;	// ISO15693 flags with protocol extension bit set
	g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Read Single Block command code

	if (ui8ReqFlag & 0x20)
	{
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID

		g_ui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16BlockNumber & 0xFF);			// Block # (variable, for this device it can go to 0xFF)
		g_ui8TrfBuffer[ui8Offset++] = (uint8_t) ((ui16BlockNumber >> 8) & 0xFF);		// Block # (variable, for this device it can go to 0x07)
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16BlockNumber & 0xFF);			// Block # (variable, for this device it can go to 0xFF)
		g_ui8TrfBuffer[ui8Offset++] = (uint8_t) ((ui16BlockNumber >> 8) & 0xFF);		// Block # (variable, for this device it can go to 0x07)
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Read Single Block command

	Trf797xIrqWaitTimeout(3,10);		// 3 millisecond TX timeout, 10 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			// Response received
			ui8Status = STATUS_SUCCESS;


			UartSendCString("NFC-V Block ");
			UartPutByte((ui16BlockNumber >> 8) & 0xFF);			// Output block number
			UartPutByte(ui16BlockNumber & 0xFF);
			UartSendCString(" Data:  ");
			UartPutChar('[');

			ui8RxLength = Trf797xGetRxBytesReceived();

			if (ui8ReqFlag & BIT6) // Handle case for Option Flag causing one extra byte to be transmitted.
			{
				ui8Offset = 2;
			}
			else
			{
				ui8Offset = 1;
			}

			// Output received data to UART
			for (ui8LoopCount = ui8Offset; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UartPutByte(g_ui8TrfBuffer[ui8LoopCount]);		// Send out data read from tag to host
			}

			UartPutChar(']');
			UartPutCrlf();

		}
		else
		{
			// Received an error from the tag
			ui8Status = STATUS_FAIL;

			// 	Indicates when an error occurs or block addresses are unreachable - useful for debugging
			UartSendCString("NFC-V Block ");
			UartPutByte(((ui16BlockNumber >> 8) & 0xFF));		// Output block number
			UartPutByte((ui16BlockNumber & 0xFF));
			UartSendCString(" Error");
			UartPutCrlf();
			UartSendCString("ISO15693 Error Code: ");
			UartPutByte(g_ui8TrfBuffer[1]);						// Output ISO15693 error code
			UartPutCrlf();

		}
	}
	else
	{
		// Did not receive a proper response from tag
		ui8Status = STATUS_FAIL;
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}

//===============================================================
//
// Iso15693_WriteSingleBlock - Issue the Write Single Block
// command for ISO15693 tags.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
// \param ui8BlockNumber is the block number to write data to.
// \param ui8BlockSize is the tag block size.
// \param pui8BlockData is the data to be written.
//
// Function issues an addressed Write Single Block with the
// specified request flags as well as the Address flag. The
// write single block command will write the provided data
// packet to an ISO15693 tag. This command supports writing to
// tags with more than 4 bytes of data per block.
//
// \return ui8Status returns either STATUS_SUCCESS or
// STATUS_FAIL to indicate if the Write Single Block was
// successful or not.
//
//===============================================================

uint8_t Iso15693_WriteSingleBlock(uint8_t ui8ReqFlag, uint8_t ui8BlockNumber, uint8_t ui8BlockSize, uint8_t * pui8BlockData)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
	uint8_t ui8LoopCount = 0;

	if (Trf797xGetIsoControlValue() != 0x02)
	{
		// Trf797x has not been properly configured for ISO15693
		Trf797xWriteIsoControl(0x02);			// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// The VCD should wait at least 2mSec after activating the
		// magnetic field before sending the first request, to
		// ensure that the VICC is ready to receive it. (ISO15693-3)
		McuDelayMillisecond(6);
	}

	ui8ReqFlag = ui8ReqFlag | 0x20; 	// Mandatory use of addressed writes

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous

	g_ui8TrfBuffer[ui8Offset++] = (((0x0B+ui8BlockSize) & 0xF0) >> 0x04);	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = ((0x0B+ui8BlockSize) << 0x04);			// Length of packet in bytes - lower and broken nibbles of transmit byte length

	g_ui8TrfBuffer[ui8Offset++] = ui8ReqFlag;	// ISO15693 flags
	g_ui8TrfBuffer[ui8Offset++] = 0x21;			// Write Single Block command code

	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
	g_ui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID

	g_ui8TrfBuffer[ui8Offset++] = ui8BlockNumber;		// Block # (variable, for HF-I Plus device can go to 0x3F, Pro and Standard handled with "error" response flags)
	for (ui8LoopCount = 0; ui8LoopCount < ui8BlockSize; ui8LoopCount++)
	{
		g_ui8TrfBuffer[ui8Offset++] = pui8BlockData[ui8LoopCount];			// Data to write to tag
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	// Check if the option flag is set
	if (ui8ReqFlag & 0x40)
	{
		Trf797xIrqWaitTimeoutTxOnly(10);	// 10 millisecond TX timeout

		g_sTrfStatus = Trf797xGetTrfStatus();

		if (g_sTrfStatus == TX_COMPLETE)	// If transmit is complete
		{
			McuDelayMillisecond(5);
			Trf797xTransmitNextSlot();		// Send out End of Frame marker
			Trf797xIrqWaitTimeoutRxOnly(30);			// 30 millisecond RX timeout
		}
		else								// Otherwise return an error
		{
			return ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		Trf797xIrqWaitTimeout(10,30);		// 10 millisecond TX timeout, 30 millisecond RX timeout
	}

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_ui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// No response
		ui8Status = STATUS_FAIL;
	}

	// Clear any IRQs
	Trf797xResetIrqStatus();

	return ui8Status;
}

//===============================================================
//
// Iso15693_Get_Uid - Fetches the ISO15693 Tag UID.
//
// This function allows for higher layers to fetch the tag UID of
// an ISO15693 tag. In the current implementation, the UID
// stored is from the most recent tag which responded to an
// inventory command or the last tag to provide it's UID during
// the ISO15693 anticollision procedure.
//
// \return g_pui8Iso15693UId returns the currently stored UID.
//
//===============================================================

uint8_t * Iso15693_Get_Uid(void)
{
	return g_pui8Iso15693UId;
}

//===============================================================
//
// Iso15693_Get_TagCount - Fetches the ISO15693 Tag UID.
//
// This function allows for higher layers to fetch the tag
// detected colunter in order to know how many ISO15693 tags
// have been detected since the last counter reset.
//
// \return g_ui8TagDetectedCount returns the count of ISO15693
// tags detected.
//
//===============================================================

uint8_t Iso15693_Get_TagCount(void)
{
	return g_ui8TagDetectedCount;
}

//===============================================================
//
// Iso15693_Reset_TagCount - Fetches the ISO15693 Tag UID.
//
// This function allows for higher layers to reset the tag
// detected counter.
//
// \return None
//
//===============================================================

void Iso15693_Reset_TagCount(void)
{
	g_ui8TagDetectedCount = 0;
}

//===============================================================
//
// Nfc_Iso15693_ReadTag - Read all blocks of a ISO15693 tag.
//
// \param ui8ReqFlag are the request flags for ISO15693 commands.
//
// This function issues Get System Information command to
// determine how many blocks of data is stored within the
// ISO15693 tag.
// Afterwards, all blocks are read out using a Read Single block,
// unless an error occurs during the read process at which point
// the function will stop reading data and return.
//
// \return None.
//
//===============================================================

void Nfc_Iso15693_ReadTag(uint8_t ui8ReqFlag)
{
	uint16_t ui16ReadBlocks = 0x00;
	uint16_t ui16LoopCount = 0x00;

	ui16ReadBlocks = Iso15693_GetSystemInfo(ui8ReqFlag); 	// Get Tag Information with Request Flag = 0x02

	if (ui16ReadBlocks != 0x00)
	{
		// Read all available blocks on the ISO15693 Tag
		for (ui16LoopCount = 0; ui16LoopCount < ui16ReadBlocks+1; ui16LoopCount++)
		{
			if (Iso15693_ReadSingleBlock(ui8ReqFlag, ui16LoopCount) == STATUS_FAIL)	// Keep reading blocks unless a No Response is received
			{
				// No Response - stop reading
				break;
			}
		}
	}
}


void ISO15693FindTag(void)
{
	uint8_t ui8TagFound = STATUS_FAIL;
	uint8_t ui8AddressedFlag = 0x00;

	Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

	Trf797xWriteInitiatorSetup(0x02);		// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4

	IRQ_CLR;									// PORT2 interrupt flag clear
	IRQ_ON;

	// The VCD should wait at least 1 ms after it activated the
	// powering field before sending the first request, to
	// ensure that the VICCs are ready to receive it. (ISO15693-3)
	osi_Sleep(10);
	//McuDelayMillisecond(20);

	Iso15693_Reset_TagCount();

	ui8TagFound = Iso15693_SingleSlotInventory();							// Send a single slot inventory request to try and detect a single ISO15693 Tag

	// Inventory failed - search with full anticollision routine
	if (ui8TagFound == STATUS_FAIL)
	{
		McuDelayMillisecond(5);				// Delay before issuing the anticollision commmand
		ui8TagFound = Iso15693_Anticollision(0x06, 0x00, 0x00);		// Send 16 Slot Inventory request with no mask length and no AFI
		ui8AddressedFlag = 0x20; 			// Collision occurred, send addressed commands
	}

	if (ui8TagFound == STATUS_SUCCESS)
	{
		if (Iso15693_Get_TagCount() > 1)
		{

			UartPutCrlf();
			UartSendCString("Multiple ISO15693 Tags Found.");
			UartPutCrlf();
			UartSendCString("Number of ISO15693 Tags Detected: ");
			UartPutChar('0' + Iso15693_Get_TagCount());
			UartPutCrlf();
			UartSendCString("Please place only 1 tag in the RF Field at a time to read tag data.");
			UartPutCrlf();

		}
		else
		{
			Nfc_Iso15693_ReadTag(0x02 | ui8AddressedFlag);					// Read an ISO15693 tag
		}
	}
	else
	{

	}

	Trf797xTurnRfOff();						// Turn off RF field once done reading the tag(s)
}
