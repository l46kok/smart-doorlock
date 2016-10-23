/*
 * File Name: type_4_ndef.c
 *
 * Description: Type 4 NDEF Functions
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
#include "type_4_ndef.h"
#include "trf7970BoosterPack.h"

//===============================================================

extern u08_t g_ui8TrfBuffer[NFC_FIFO_SIZE];

static volatile tTRF797x_Status g_sTrfStatus;

static volatile bool g_bBlockNumberBit = 0;

//===============================================================

//Echoes back WTX request from Android HCE
//Overwrites g_sTrfStatus
static void SBlock_WTXEcho(u08_t wtxBuf, u08_t didBuf)
{
	u08_t ui8Offset = 0;
	//S-Block WTX request. Echo back
	// Buffer setup for FIFO writing
	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = wtxBuf;		//RATS Command
	g_ui8TrfBuffer[ui8Offset++] = didBuf;	//RATS Parameters: 128 byte max receive and CID = 0

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(20,50);

	g_sTrfStatus = Trf797xGetTrfStatus();
}

u08_t NDEF_ApplicationSelect(void)
{
	u08_t ui8SelectSuccess = STATUS_FAIL;
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0xE0; 	// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xA4;		// INS = Select (Application in this case)
	g_ui8TrfBuffer[ui8Offset++] = 0x04;		// P1
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// P2
	g_ui8TrfBuffer[ui8Offset++] = 0x07;		// Lc
	g_ui8TrfBuffer[ui8Offset++] = 0xD2;		// Data = 0xD2760000850101 - per NFC Forum Type 4 Tag Operation
	g_ui8TrfBuffer[ui8Offset++] = 0x76;
	g_ui8TrfBuffer[ui8Offset++] = 0x00;
	g_ui8TrfBuffer[ui8Offset++] = 0x00;
	g_ui8TrfBuffer[ui8Offset++] = 0x85;
	g_ui8TrfBuffer[ui8Offset++] = 0x01;
	g_ui8TrfBuffer[ui8Offset++] = 0x01;
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Le

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(20,100);	// 10 millisecond TX timeout, 20 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	// If data received, should return same PCB, SW1 = 0x90, SW2 = 0x00
	if (g_sTrfStatus == RX_COMPLETE)
	{
		if (g_ui8TrfBuffer[0] == 0xF2)
		{
			//Delay a bit before echoing
			McuDelayMillisecond(50);
			SBlock_WTXEcho(g_ui8TrfBuffer[0], g_ui8TrfBuffer[1]);
		}

		if(g_sTrfStatus == RX_COMPLETE && (g_ui8TrfBuffer[0] == 0x02 | g_bBlockNumberBit) && (g_ui8TrfBuffer[1] == 0x90) && (g_ui8TrfBuffer[2] == 0x00))
		{
			ui8SelectSuccess = STATUS_SUCCESS;
		}

	}

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	return ui8SelectSuccess;
}

u08_t NDEF_CapabilityContainerSelect(void)
{
	u08_t ui8SelectSuccess = STATUS_FAIL;
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x80;	// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xA4;	// INS = Read CC
	g_ui8TrfBuffer[ui8Offset++] = 0x00; // P1
	g_ui8TrfBuffer[ui8Offset++] = 0x0C; // P2
	g_ui8TrfBuffer[ui8Offset++] = 0x02; // Lc
	g_ui8TrfBuffer[ui8Offset++] = 0xE1; // Data = 0xE103 - per NFC Forum Type 4 Tag Operation
	g_ui8TrfBuffer[ui8Offset++] = 0x03;

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(200,200);	// 10 millisecond TX timeout, 20 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	// If data received, should return same PCB, SW1 = 0x90, SW2 = 0x00
	if (g_sTrfStatus == RX_COMPLETE)
	{
		if (g_ui8TrfBuffer[0] == 0xF2)
		{
			SBlock_WTXEcho(g_ui8TrfBuffer[0], g_ui8TrfBuffer[1]);
		}

		if((g_ui8TrfBuffer[0] == 0x02 | g_bBlockNumberBit) && (g_ui8TrfBuffer[1] == 0x90) && (g_ui8TrfBuffer[2] == 0x00))
		{
			ui8SelectSuccess = STATUS_SUCCESS;
		}
	}

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	McuDelayMillisecond(1);						// Short delay before sending next command

	return ui8SelectSuccess;
}

u16_t NDEF_ReadBinary(u16_t ui16FileOffset, u08_t ui8ReadLength)
{
	u08_t ui8Offset = 0;
	u16_t ui16Nlen = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x60;	// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xB0;	// INS = Read Binary
	g_ui8TrfBuffer[ui8Offset++] = ((ui16FileOffset >> 8) & 0xFF);	// File Offset where to start reading data
	g_ui8TrfBuffer[ui8Offset++] = (ui16FileOffset & 0x00FF);		// File Offset where to start reading data
	g_ui8TrfBuffer[ui8Offset++] = ui8ReadLength;					// Read Length

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(20,100);	// 20 millisecond TX timeout, 100 millisecond RX timeout

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_ui8TrfBuffer[0] == 0xF2)
	{
		SBlock_WTXEcho(g_ui8TrfBuffer[0], g_ui8TrfBuffer[1]);
	}

	if(g_sTrfStatus == RX_COMPLETE)
	{
		ui16Nlen = (g_ui8TrfBuffer[1] << 8) + g_ui8TrfBuffer[2];
	}

	McuDelayMillisecond(1);						// Short delay before sending next command

	return ui16Nlen;
}

u08_t NDEF_FileSelect(u16_t ui16FileID)
{
	u08_t ui8SelectSuccess = STATUS_FAIL;
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x80;	// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xA4;	// INS = Select (File in this case)
	g_ui8TrfBuffer[ui8Offset++] = 0x00;
	g_ui8TrfBuffer[ui8Offset++] = 0x0C;
	g_ui8TrfBuffer[ui8Offset++] = 0x02;
	g_ui8TrfBuffer[ui8Offset++] = ((ui16FileID >> 8) & 0xFF);
	g_ui8TrfBuffer[ui8Offset++] = (ui16FileID & 0x00FF);

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(15,50);	// 15 millisecond TX timeout, 50 millisecond RX timeout

	// If data received, should return same PCB, SW1 = 0x90, SW2 = 0x00
	if (g_sTrfStatus == RX_COMPLETE)
	{
		if((g_ui8TrfBuffer[0] == 0x02 | g_bBlockNumberBit) && (g_ui8TrfBuffer[1] == 0x90) && (g_ui8TrfBuffer[2] == 0x00))
		{
			ui8SelectSuccess = STATUS_SUCCESS;
		}
	}

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	McuDelayMillisecond(1);						// Short delay before sending next command

	return ui8SelectSuccess;
}


void NDEF_UpdateBinaryLength(u16_t ui16Nlen)
{
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x80;	// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xD6;	// INS = Update Binary
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Offset, P1
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Offset, P2
	g_ui8TrfBuffer[ui8Offset++] = 0x02;	// Length, Lc
	g_ui8TrfBuffer[ui8Offset++] = ((ui16Nlen >> 8) & 0xFF);	// MSByte NLEN being set to 0
	g_ui8TrfBuffer[ui8Offset++] = (ui16Nlen & 0xFF);		// LSByte NLEN being set to 0

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(10,20);	// 10 millisecond TX timeout, 20 millisecond RX timeout

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number
}

void Ndef_ReadCC(void)
{
	NDEF_CapabilityContainerSelect();		// Selects the Capability Container
	NDEF_ReadBinary(0, 15);					// Read the contents of the capability container
}

void Ndef_ReadApplication(void)
{
	u16_t ui16NdefLength = 0;
	u08_t ui8NdefReadLength = 0;
#ifdef ENABLE_HOST
	u08_t ui8NdefMessageOffset = 0;
	u08_t ui8LoopCount = 0;
#endif

	NDEF_FileSelect(0xE104);					// Selects NDEF Application
	ui16NdefLength = NDEF_ReadBinary(0, 2);		// Reads NDEF Application for length of message

	if (ui16NdefLength > NFC_FIFO_SIZE)
	{
		ui8NdefReadLength = NFC_FIFO_SIZE;
#ifdef ENABLE_HOST
		UartSendCString("NDEF Message Size Exceeds Internal Buffer, Displaying Partial Message Contents Only.");
		UartPutCrlf();
#endif
	}
	else
	{
		ui8NdefReadLength = ui16NdefLength;
	}

	NDEF_ReadBinary(2, ui8NdefReadLength);		// Reads NDEF Application for the NDEF content

#ifdef ENABLE_HOST
	UartSendCString("NDEF Message: ");
	UartPutChar('[');
	if (g_ui8TrfBuffer[4] == 0x54)
	{
		ui8NdefMessageOffset = 8;
	}
	else if (g_ui8TrfBuffer[4] == 0x55)
	{
		ui8NdefMessageOffset = 6;
		if (g_ui8TrfBuffer[5] == 0x01)
		{
			UartSendCString("http://wwww.");
		}
		else if (g_ui8TrfBuffer[5] == 0x02)
		{
			UartSendCString("https://wwww.");
		}
		else if (g_ui8TrfBuffer[5] == 0x03)
		{
			UartSendCString("http://");
		}
		else if (g_ui8TrfBuffer[5] == 0x04)
		{
			UartSendCString("https://");
		}
	}
	else
	{
		ui8NdefMessageOffset = 0x00;
	}

	if (ui16NdefLength > (NFC_FIFO_SIZE-ui8NdefMessageOffset))
	{
		ui8NdefReadLength = NFC_FIFO_SIZE-ui8NdefMessageOffset;
	}
	else
	{
		ui8NdefReadLength = ui16NdefLength-ui8NdefMessageOffset+1;
	}

	for (ui8LoopCount = 0; ui8LoopCount < ui8NdefReadLength; ui8LoopCount++)
	{
		UartPutChar(g_ui8TrfBuffer[ui8NdefMessageOffset++]);
	}
	UartPutChar(']');
	UartPutCrlf();
	UartPutCrlf();
#endif
}

void Ndef_WriteText(void)
{
#ifdef ENABLE_HOST
	UartSendCString("Now Writing New NDEF Message.");
	UartPutCrlf();
	UartPutCrlf();
#endif

	NDEF_UpdateBinaryText();
}

void Ndef_WriteUri(void)
{
#ifdef ENABLE_HOST
	UartSendCString("Now Writing New NDEF Message.");
	UartPutCrlf();
	UartPutCrlf();
#endif

	NDEF_UpdateBinaryUri();
}

void Ndef_SetBlockNumberBit(bool bValue)
{
	g_bBlockNumberBit = bValue;
}

void NDEF_UpdateBinaryText(void)
{
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous

	g_ui8TrfBuffer[ui8Offset++] = 0x01;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0xF0; // Length of packet in bytes - lower and broken nibbles of transmit byte length

	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00; // CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xD6;	// INS = update Binary
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Offset, P1
	g_ui8TrfBuffer[ui8Offset++] = 0x02;	// Offset, P2

	g_ui8TrfBuffer[ui8Offset++] = 0x19;	// Lc, length being written (all bytes)

	g_ui8TrfBuffer[ui8Offset++] = 0xD1;	// MB = 1, ME = 1, Short Record, TNF = NFC Forum Well Known Type
	g_ui8TrfBuffer[ui8Offset++] = 0x01;	// Length of Record Type

	g_ui8TrfBuffer[ui8Offset++] = 0x15;	// Length of Text being written (21 bytes, hardcoded for now)
	g_ui8TrfBuffer[ui8Offset++] = 0x54;	// Text

	g_ui8TrfBuffer[ui8Offset++] = 0x02;	// Language Length
	g_ui8TrfBuffer[ui8Offset++] = 0x65;	// 'e' - For English
	g_ui8TrfBuffer[ui8Offset++] = 0x6E;	// 'n' - For English

	g_ui8TrfBuffer[ui8Offset++] = 0x4E;	// N
	g_ui8TrfBuffer[ui8Offset++] = 0x46;	// F
	g_ui8TrfBuffer[ui8Offset++] = 0x43;	// C
	g_ui8TrfBuffer[ui8Offset++] = 0x20;	//
	g_ui8TrfBuffer[ui8Offset++] = 0x50;	// P
	g_ui8TrfBuffer[ui8Offset++] = 0x6F;	// o
	g_ui8TrfBuffer[ui8Offset++] = 0x77;	// w
	g_ui8TrfBuffer[ui8Offset++] = 0x65;	// e
	g_ui8TrfBuffer[ui8Offset++] = 0x72;	// r
	g_ui8TrfBuffer[ui8Offset++] = 0x65;	// e
	g_ui8TrfBuffer[ui8Offset++] = 0x64;	// d
	g_ui8TrfBuffer[ui8Offset++] = 0x20;	//
	g_ui8TrfBuffer[ui8Offset++] = 0x42;	// B
	g_ui8TrfBuffer[ui8Offset++] = 0x79;	// y
	g_ui8TrfBuffer[ui8Offset++] = 0x20;	//
	g_ui8TrfBuffer[ui8Offset++] = 0x54;	// T
	g_ui8TrfBuffer[ui8Offset++] = 0x49;	// I
	g_ui8TrfBuffer[ui8Offset++] = 0x21;	// !

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(10,20);	// 10 millisecond TX timeout, 20 millisecond RX timeout

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	McuDelayMillisecond(5);

	NDEF_UpdateBinaryLength(0x19);
}

void NDEF_UpdateBinaryUri(void)
{
	u08_t ui8Offset = 0;

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;	// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;	// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;	// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x01;	// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x50; // Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x02 | g_bBlockNumberBit;	// I-Block PCB: Read Block 0 or Block 1, with CID = 0, NAD = 0, no chaining
	g_ui8TrfBuffer[ui8Offset++] = 0x00; // CLA
	g_ui8TrfBuffer[ui8Offset++] = 0xD6;	// INS = update Binary
	g_ui8TrfBuffer[ui8Offset++] = 0x00;	// Offset, P1
	g_ui8TrfBuffer[ui8Offset++] = 0x02;	// Offset, P2
	g_ui8TrfBuffer[ui8Offset++] = 0x0F;	// Lc, length being written (all bytes)
	g_ui8TrfBuffer[ui8Offset++] = 0xD1;	// MB = 1, ME = 1, Short Record, TNF = NFC Forum Well Known Type
	g_ui8TrfBuffer[ui8Offset++] = 0x01;	// Length of Record Type
	g_ui8TrfBuffer[ui8Offset++] = 0x0B;	// Length of URI being written (11 bytes, hardcoded for now)
	g_ui8TrfBuffer[ui8Offset++] = 0x55;	// URI
	g_ui8TrfBuffer[ui8Offset++] = 0x01;	// URI Identifier: http://wwww.
	g_ui8TrfBuffer[ui8Offset++] = 0x74;	// t
	g_ui8TrfBuffer[ui8Offset++] = 0x69;	// i
	g_ui8TrfBuffer[ui8Offset++] = 0x2E;	// .
	g_ui8TrfBuffer[ui8Offset++] = 0x63;	// c
	g_ui8TrfBuffer[ui8Offset++] = 0x6F;	// o
	g_ui8TrfBuffer[ui8Offset++] = 0x6D;	// m
	g_ui8TrfBuffer[ui8Offset++] = 0x2F;	// /
	g_ui8TrfBuffer[ui8Offset++] = 0x6E;	// n
	g_ui8TrfBuffer[ui8Offset++] = 0x66;	// f
	g_ui8TrfBuffer[ui8Offset++] = 0x63;	// c

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the NDEF Command

	Trf797xIrqWaitTimeout(10,20);	// 10 millisecond TX timeout, 20 millisecond RX timeout

	g_bBlockNumberBit = !g_bBlockNumberBit; 	// Toggle the PCB Block Number

	McuDelayMillisecond(5);

	NDEF_UpdateBinaryLength(0x0F);
}
