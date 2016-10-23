/*
 * File Name: trf797x.c
 *
 * Description: TRF797x Driver Functions
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

#include <string.h>
#include <stdio.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "trf797x.h"
#include "trf7970BoosterPack.h"
#include "timer_if.h"
#include "gpio.h"
#include "osi.h"

//===============================================================

u08_t g_ui8TrfBuffer[NFC_FIFO_SIZE];

static u08_t	g_ui8CollisionPosition = 0;

static u08_t	g_ui8FifoOffset = 0;
static u08_t	g_ui8FifoRxLength = 0;

static bool 	g_bRfFieldOn = false;
static u08_t 	g_ui8IsoControlValue = 0x00;

static tTRF797x_Status g_sTrf797xStatus = TRF_IDLE;

static volatile u08_t g_ui8IrqFlag = 0x00;
static volatile u08_t g_ui8TimeoutFlag = 0x00;

u08_t    command[2];

//===============================================================
//
// Trf797xCommunicationSetup - Initialize Communication to TRF797x
//
// Call the appropriate serial communication initialize based on
// hardware setup
//
// \return None.
//
//===============================================================

void
Trf797xCommunicationSetup(void)
{
	A2CounterInit(A2CNTIntHandler);
	RESET_COUNTER;
	STOP_COUNTER;
	command[0] = SOFT_INIT;
	Trf797xDirectCommand(command);

	command[0] = IDLE;
	Trf797xDirectCommand(command);

	command[0] = TRF797x_MODULATOR_CONTROL;
	command[1] = 0x01;                    // ASK 100%, no SYS_CLK output
	Trf797xWriteSingle(command, 2);

	command[0] = TRF797x_NFC_TARGET_LEVEL;
	command[1] = 0x00;
	Trf797xWriteSingle(command, 2);
}

//===============================================================
//
// Trf797xDirectCommand - Send a Direct Command to TRF797x.
//
// \param ui8Value is the direct command to be issued
//
//  Function is used to transmit a Direct Command to TRF797x.
//
// \return None.
//
//===============================================================

void
Trf797xDirectCommand(u08_t *ui8Value)
{
	SpiDirectCommand(ui8Value);
}

//===============================================================
//
// Trf797xDisableSlotCounter - Disables interrupts from 15693
// slot counter function.
//
// Turns off the firing of the IRQ interrupt when a 15693 slot
// hits the No Response timeout
//
// \return None.
//
//===============================================================

void
Trf797xDisableSlotCounter(void)
{
	u08_t pui8ModControl[2];

	pui8ModControl[0] = TRF797x_IRQ_MASK;              // next slot counter
	pui8ModControl[1] = TRF797x_IRQ_MASK;
	Trf797xReadSingle(&pui8ModControl[1], 1);
	pui8ModControl[1] &= 0xFE;                // clear BIT0 in register 0x01
	Trf797xWriteSingle(pui8ModControl, 2);
}

//===============================================================
//
// Trf797xEnableSlotCounter - Enables interrupts from 15693
// slot counter function.
//
// Turns on the firing of the IRQ interrupt when a 15693 slot
// hits the No Response timeout
//
// \return None.
//
//===============================================================

void
Trf797xEnableSlotCounter(void)
{
	u08_t pui8ModControl[2];

	pui8ModControl[0] = TRF797x_IRQ_MASK;              // next slot counter
	pui8ModControl[1] = TRF797x_IRQ_MASK;
	Trf797xReadSingle(&pui8ModControl[1],1);
	pui8ModControl[1] |= BIT0;                // set BIT0 in register 0x01
	Trf797xWriteSingle(pui8ModControl,2);
}

//===============================================================
//
// Trf797xEnableSlotCounter - Initialize TRF797x
//
// Handles setting up the clock register and writes register 0x18
// as per TRF7970A Errata
//
// \return None.
//
//===============================================================

void
Trf797xInitialSettings(void)
{
	command[0] = SOFT_INIT;
	Trf797xDirectCommand(command);
	command[0] = IDLE;
	Trf797xDirectCommand(command);

	command[0] = TRF797x_MODULATOR_CONTROL;
	command[1] = 0x01;                    // ASK 100%, no SYS_CLK output
	Trf797xWriteSingle(command,2);

	command[0] = TRF797x_NFC_TARGET_LEVEL;
	command[1] = 0x00;
    Trf797xWriteSingle(command,2);
}

//===============================================================
//
// Trf797xISR - Services TRF797x IRQ interrupts
//
// \param pui8_IrqStatus is the received IRQ Status flags
//
// The Interrupt Service Routine determines how the IRQ should
// be handled. The Trf797x IRQ status register is read to
// determine the cause of the IRQ. Conditions are checked and
// appropriate actions taken.
//
// \return None.
//
//===============================================================

void
Trf797xISR(u08_t * pui8_IrqStatus)
{
	u08_t	ui8DummyRead;
	u08_t	ui8Length;

	if(*pui8_IrqStatus == (TRF797x_IRQ_STATUS_TX_COMPLETE | TRF797x_IRQ_STATUS_FIFO_HIGH_OR_LOW))			// BIT5 and BIT7
	{								// TX active and 32 bytes left in FIFO
		g_sTrf797xStatus = TX_COMPLETE;
	}

	else if(*pui8_IrqStatus == TRF797x_IRQ_STATUS_TX_COMPLETE)
	{								// TX complete
		g_sTrf797xStatus = TX_COMPLETE;
		Trf797xReset();				// reset the FIFO after TX
	}

	else if((*pui8_IrqStatus & BIT1) == TRF797x_IRQ_STATUS_COLLISION_ERROR)
	{								// Collision error
		g_sTrf797xStatus = PROTOCOL_ERROR;

		if ((g_ui8IsoControlValue == 0x08) || (g_ui8IsoControlValue == 0x88))
		{
			g_ui8CollisionPosition = TRF797x_COLLISION_POSITION;
			Trf797xReadSingle(&g_ui8CollisionPosition, 1);

			if (g_ui8CollisionPosition > 0x20)
			{
				ui8Length = g_ui8CollisionPosition - 0x20;		// number of valid bytes in FIFO

				if((ui8Length & 0x0F) != 0x00)
				{
					ui8Length = ui8Length + 0x10;	// add 1 byte if broken byte recieved
				}
				ui8Length = ui8Length >> 4;

				if(ui8Length != 0x00)
				{
					g_ui8TrfBuffer[g_ui8FifoOffset] = FIFO;		// write the recieved bytes to the correct place of the buffer

					Trf797xReadCont(&g_ui8TrfBuffer[g_ui8FifoOffset], ui8Length);
					g_ui8FifoOffset = g_ui8FifoOffset + ui8Length;
				}
			}
			else
			{
				g_ui8FifoRxLength = TRF797x_FIFO_STATUS;
				Trf797xReadSingle(&g_ui8FifoRxLength,1);	// determine the number of bytes left in FIFO

				g_ui8FifoRxLength = 0x7F & g_ui8FifoRxLength;
				g_ui8TrfBuffer[g_ui8FifoOffset] = FIFO;				// write the recieved bytes to the correct place of the buffer

				Trf797xReadCont(&g_ui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);
				g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;
			}
		}

		Trf797xStopDecoders();

		Trf797xReset();		// reset the FIFO after TX

		Trf797xResetIrqStatus();

		IRQ_CLR;
	}
	else if(*pui8_IrqStatus == TRF797x_IRQ_STATUS_RX_COMPLETE || *pui8_IrqStatus == 0xC0)
	{
		//10-20-2016 Fix
		//TRF7970A returns C0 (TX_IRQ = 1, RX_IRQ = 1) during selection command
		//Really strange, but reading from fifo right afterwards does work for some reason...

		// RX flag means that EOF has been recieved
		// and the number of unread bytes is in FIFOstatus regiter

		g_ui8FifoRxLength = TRF797x_FIFO_STATUS;
		Trf797xReadSingle(&g_ui8FifoRxLength, 1);	// determine the number of bytes left in FIFO

		g_ui8FifoRxLength = 0x7F & g_ui8FifoRxLength;
		g_ui8TrfBuffer[g_ui8FifoOffset] = FIFO;				// write the recieved bytes to the correct place of the buffer

		Trf797xReadCont(&g_ui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);
		g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;

		Trf797xReset();			// reset the FIFO after last byte has been read out

		if (g_sTrf797xStatus == RX_WAIT_EXTENSION)
		{
			g_ui8FifoRxLength = g_ui8FifoOffset;
		}

		g_sTrf797xStatus = RX_COMPLETE;
	}
	else if(*pui8_IrqStatus == (TRF797x_IRQ_STATUS_RX_COMPLETE | TRF797x_IRQ_STATUS_FIFO_HIGH_OR_LOW))
	{									// RX active and 96 bytes allready in FIFO
		g_sTrf797xStatus = RX_WAIT;

		// Read FIFO Status to determine how many bytes are in the FIFO
		g_ui8FifoRxLength = TRF797x_FIFO_STATUS;
		Trf797xReadSingle(&g_ui8FifoRxLength, 1);
		g_ui8FifoRxLength = 0x7F & g_ui8FifoRxLength;

		if (NFC_FIFO_SIZE > (g_ui8FifoOffset+g_ui8FifoRxLength))
		{
			// Read from the FIFO to empty it
			g_ui8TrfBuffer[g_ui8FifoOffset] = FIFO;
			Trf797xReadCont(&g_ui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);	// read all received bytes from FIFO
			g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;					// Adjust buffer index
		}
		else
		{
			g_sTrf797xStatus = PROTOCOL_ERROR;
			return;
		}

		// Read FIFO Status again to determine if more bytes have been received
		g_ui8FifoRxLength = TRF797x_FIFO_STATUS;
		Trf797xReadSingle(&g_ui8FifoRxLength, 1);	// determine the number of bytes left in FIFO
		g_ui8FifoRxLength = 0x7F & g_ui8FifoRxLength;

		if (g_ui8FifoRxLength > 0)
		{
			g_sTrf797xStatus = RX_WAIT_EXTENSION;
		}
		else
		{
			g_ui8FifoRxLength = g_ui8FifoOffset;
			g_sTrf797xStatus = RX_COMPLETE;
			return;
		}
	}
	else if (*pui8_IrqStatus == (TRF797x_IRQ_STATUS_RX_COMPLETE | TRF797x_IRQ_STATUS_NO_RESPONSE))
	{
		// RX has begun but as not completed, space exists in FIFO still, just wait longer to receive full reply.
		g_sTrf797xStatus = RX_WAIT_EXTENSION;
	}
	else if((*pui8_IrqStatus & BIT4) == TRF797x_IRQ_STATUS_CRC_ERROR)		// CRC error
	{
		if((*pui8_IrqStatus & BIT5) == TRF797x_IRQ_STATUS_FIFO_HIGH_OR_LOW)
		{
			g_sTrf797xStatus = RX_WAIT;
		}
		if((*pui8_IrqStatus & BIT6) == TRF797x_IRQ_STATUS_RX_COMPLETE)		// 4 Bit receive
		{
			ui8DummyRead = FIFO;		// write the recieved bytes to the correct place of the buffer

			Trf797xReadCont(&ui8DummyRead, 1);

			Trf797xReset();

			g_sTrf797xStatus = PROTOCOL_ERROR;
		}
		else
		{
			g_sTrf797xStatus = PROTOCOL_ERROR;
		}
	}
	else if((*pui8_IrqStatus & BIT2) == TRF797x_IRQ_STATUS_FRAMING_ERROR)	// byte framing error
	{
		if((*pui8_IrqStatus & BIT5) == TRF797x_IRQ_STATUS_FIFO_HIGH_OR_LOW)
		{
			g_sTrf797xStatus = RX_WAIT;
		}
		else
		{
			g_sTrf797xStatus = PROTOCOL_ERROR;
		}
	}
	else if(*pui8_IrqStatus == TRF797x_IRQ_STATUS_IDLE)
	{						// No response interrupt
		//g_sTrf797xStatus = NO_RESPONSE_RECEIVED;
	}
	else if(*pui8_IrqStatus == TRF797x_IRQ_STATUS_NO_RESPONSE)
	{						// No response interrupt
		g_sTrf797xStatus = NO_RESPONSE_RECEIVED_15693;
		g_ui8FifoOffset = 0;
	}
	else
	{						// Interrupt register not properly set
		g_sTrf797xStatus = PROTOCOL_ERROR;

		Trf797xStopDecoders();	// reset the FIFO after TX
		Trf797xReset();
		Trf797xResetIrqStatus();

		IRQ_CLR;
	}
}							// Interrupt Service Routine

//===============================================================
//
// Trf797xIRQ - Interrupt handler for IRQ interrupts
//
// Handles receiving IRQ's, getting IRQ status, and maintaining
// timers/global variables
//
// \return None.
//
//===============================================================

void
Trf797xIRQ(void)							// interrupt handler
{
	u08_t ui8IrqStatus, ui8IsoControl;

	RESET_COUNTER;							// stop timer mode
	STOP_COUNTER;


	g_ui8IrqFlag = 0x01;

	do
	{
		IRQ_CLR;							// PORT2 interrupt flag clear

		// IRQ status register has to be read
		Trf797xReadIrqStatus(&ui8IrqStatus);

		//UartSendCString("IRQ Status: ");
		//UartPutByteHex(ui8IrqStatus);
		//UartPutCrlf();

		if(ui8IrqStatus == 0xA0)				// TX active and only 3 bytes left in FIFO
		{
			g_sTrf797xStatus = TX_WAIT;
			//StatusPrint(g_sTrf797xStatus);
			break;
		}
		else
		{
			ui8IsoControl = TRF797x_ISO_CONTROL;
			Trf797xReadSingle(&ui8IsoControl, 1);
			if((ui8IsoControl & BIT5) != BIT5)			// RFID mode
			{
				Trf797xISR(&ui8IrqStatus);
				//StatusPrint(g_sTrf797xStatus);
			}
			else										// NFC mode
			{
				// Do Nothing
			}
		}
	} while(GPIOIntStatus(GPIOA1_BASE,1) & GPIO_PIN_4);
}

//===============================================================
//
// Trf797xRawWrite - Write data to TRF797x
//
// \param pui8Payload is the buffer with data packet contents
// \param ui8Length is the size of the data packet
//
// Function is used to write data directly to the TRF797x.
//
// \return None.
//
//===============================================================

void
Trf797xRawWrite(u08_t * pui8Payload, u08_t ui8Length)
{
	u08_t ui8TxBytesRemaining;
	u08_t ui8TxIndex;
	u08_t ui8FifoTxLength;
	u08_t ui8TxBytesAvailable;

	if (127 > ui8Length)
	{
		SpiRawWrite(pui8Payload, ui8Length);
	}
	else
	{
		ui8TxBytesRemaining = ui8Length;
		ui8TxIndex = 0;
		ui8TxBytesAvailable = 127;

		while(ui8TxBytesRemaining > 0)
		{
			if (ui8TxBytesRemaining > 127)
			{
				SpiRawWrite(&pui8Payload[ui8TxIndex], ui8TxBytesAvailable);
				ui8TxBytesRemaining = ui8TxBytesRemaining - ui8TxBytesAvailable;
				ui8TxIndex = ui8TxIndex + ui8TxBytesAvailable;
			}
			else
			{
				SpiRawWrite(&pui8Payload[ui8TxIndex], ui8TxBytesRemaining);
			}

			Trf797xIrqWaitTimeoutTxOnly(35);

			g_sTrf797xStatus = Trf797xGetTrfStatus();

			if ((g_sTrf797xStatus == TX_COMPLETE) || (g_sTrf797xStatus == TX_WAIT))
			{
				ui8FifoTxLength = TRF797x_FIFO_STATUS;
				Trf797xReadSingle(&ui8FifoTxLength, 1);	// determine the number of bytes left in FIFO

				ui8TxBytesAvailable = 127-ui8FifoTxLength;
			}
			else
			{
				// Error occurred, break
				g_sTrf797xStatus = TX_ERROR;
				break;
			}
		}
	}
}

//===============================================================
//
// Trf797xReadCont - Read out multiple TRF797x registers
//
// \param pui8Payload is the address of the first register as
// well as the pointer for buffer where the results will be
// \param ui8Length is the number of registers to read
//
// Function used to read a specified number of TRF797x registers
// from a specified address.
//
// \return None.
//
//===============================================================

void
Trf797xReadCont(u08_t * pui8Payload, u08_t ui8Length)
{
	SpiReadCont(pui8Payload, ui8Length);
}

//===============================================================
//
// Trf797xReadIsoControl - Read the ISO Control Register
//
// Function used to read the ISO Control Register of the TRF797x
//
// \return pui8Value returns the value of the ISO Control
// Register
//
//===============================================================

u08_t
Trf797xReadIsoControl(void)
{
	u08_t pui8Value[1];
	*pui8Value = TRF797x_ISO_CONTROL;

	SpiReadSingle(pui8Value, 1);

	g_ui8IsoControlValue = pui8Value[0];	// Update the ISO Control Register variable

	return pui8Value[0];
}

//===============================================================
//
// Trf797xReadIrqStatus - Read out the IRQ Status Register
//
// \param pui8Value is the pointer to the buffer where the
// result will be
//
// Function used to read the IRQ Status register of the TRF797x
// and store the result into the location pointed to by the input
//
// \return None.
//
//===============================================================

void
Trf797xReadIrqStatus(u08_t * pui8Value)
{
	*pui8Value = TRF797x_IRQ_STATUS;

	SpiReadSingle(pui8Value, 1);

/*
	  *pui8Value = TRF797x_IRQ_STATUS;
	  *(pui8Value + 1) = TRF797x_IRQ_MASK;
	  Trf797xReadCont(pui8Value, 2);           // read second reg. as dummy read
*/
}

//===============================================================
//
// Trf797xReadSingle - Read out a single TRF797x registers
//
// \param pui8Value is the address of the register to read as
// well as pointer for the buffer where the result will be
//
// Function used to read a specific TRF797x register
//
// \return None.
//
//===============================================================

void
Trf797xReadSingle(u08_t *pbuf, u08_t number)
{
	SpiReadSingle(pbuf, number);
}

//===============================================================
//
// Trf797xReadStatusControl - Read the Chip Status Control
// Register
//
// Function used to read the Chip Status Control Register of the
// TRF797x
//
// \return pui8Value returns the value of the Chip Status Control
// Register
//
//===============================================================

u08_t
Trf797xReadStatusControl(void)
{
	u08_t pui8Value[1];
	*pui8Value = TRF797x_STATUS_CONTROL;

	SpiReadSingle(pui8Value, 1);

	if ((pui8Value[0] & BIT5) == BIT5)	// Check for RF field bit and update variable
	{
		g_bRfFieldOn = true;
	}
	else
	{
		g_bRfFieldOn = false;
	}

	return pui8Value[0];
}

//===============================================================
//
// Trf797xReset - Resets TRF797x
//
// Function used to reset the TRF797x
//
// \return None.
//
//===============================================================

void
Trf797xReset(void)
{
	command[0] = RESET;
	Trf797xDirectCommand(command);
}

//===============================================================
//
// Trf797xReset - Resets the IRQ Status Register of the TRF797x
//
// Function used to reset the TRF797x IRQ Status Register
//
// \return None.
//
//===============================================================

void
Trf797xResetIrqStatus(void)
{
	u08_t puiIrqStatus[2];

	puiIrqStatus[0] = TRF797x_IRQ_STATUS;
	puiIrqStatus[1] = TRF797x_IRQ_MASK;

	Trf797xReadCont(puiIrqStatus, 2);		// read second reg. as dummy read
}

//===============================================================
//
// Trf797xRunDecoders - Direct command to enable TRF797x receivers
//
// Issue direct command 0x17 - Enable Receivers to the TRF797x
//
// \return None.
//
//===============================================================
void
Trf797xRunDecoders(void)
{
	command[0] = RUN_DECODERS;
	Trf797xDirectCommand(command);
}

//===============================================================
//
// Trf797xStopDecoders - Direct command to disable TRF797x
// receivers
//
// Issue direct command 0x16 - Block Receivers to the TRF797x
//
// \return None.
//
//===============================================================

void
Trf797xStopDecoders(void)
{
	command[0] = STOP_DECODERS;
	Trf797xDirectCommand(command);
}

//===============================================================
//
// Trf797xTransmitNextSlot - Direct command to transmit next slot
// for ISO15693 End of Frame
//
// Issue direct command 0x14 - End of Frame/Transmit Next Time Slot
// (ISO15693) to the TRF797x
//
// \return None.
//
//===============================================================

void
Trf797xTransmitNextSlot(void)
{
	command[0] = TRANSMIT_NEXT_SLOT;
	Trf797xDirectCommand(command);
}

//===============================================================
//
// Trf797xTurnRfOff - Turn off the transmission of the TRF797x
// RF Field
//
// Function used stop the TRF797x transmitting an RF field
//
// \return None.
//
//===============================================================

void
Trf797xTurnRfOff(void)
{
	u08_t	pui8Command[2];

	pui8Command[0] = TRF797x_STATUS_CONTROL;
	pui8Command[1] = 0x00;			// 3.3VDC, full power out
//	pui8Command[1] = 0x10;			// 3.3VDC, half power out
	Trf797xWriteSingle(pui8Command, 2);

	g_bRfFieldOn = false;	// Update RF Field variable
}

//===============================================================
//
// Trf797xTurnRfOn - Turns on the transmission of the TRF797x
// RF Field
//
// Function used make the TRF797x transmit an RF field
//
// \return None.
//
//===============================================================

void
Trf797xTurnRfOn(void)
{
	u08_t	pui8Command[2];

	pui8Command[0] = TRF797x_STATUS_CONTROL;
	pui8Command[1] = 0x20;			// 3.3VDC, full power out
//	pui8Command[1] = 0x30;			// 3.3VDC, half power out
	Trf797xWriteSingle(pui8Command, 2);

	g_bRfFieldOn = true;	// Update RF Field variable
}

//===============================================================
//
// Trf797xWriteCont - Write to consecutive TRF797x registers
//
// \param pui8Payload is the address of the first register
// followed by the contents to write for each register
// \param ui8Length is the number of registers to write + 1
// Minimum value of ui8Length allowed = 2 (a write to 1 register)
//
// Function used to write to a specific number of TRF797x
// registers from a specific address.
//
// \return None.
//
//===============================================================

void
Trf797xWriteCont(u08_t * pui8Payload, u08_t ui8Length)
{
	if (ui8Length > 1) // Cannot specify a length of 1
	{
		if (*pui8Payload == 0x00)	// If the write starts at the Chip Status Control Register
		{
			if (((*pui8Payload+1) & BIT5) == BIT5)	// Check for RF field bit and update variable
			{
				g_bRfFieldOn = true;
			}
			else
			{
				g_bRfFieldOn = false;
			}
			if (ui8Length > 2)		// Check if the write length includes the ISO Control Register being written to (0x01)
			{
				g_ui8IsoControlValue = (*pui8Payload+2);	// If so, update the Iso Control Register variable
			}
		}
		else if (*pui8Payload == 0x01)	// If the write starts at the ISO Control Register
		{
			g_ui8IsoControlValue = *pui8Payload+1;	// Update the ISO Control Register variable
		}

		// Call continuous write function
		SpiWriteCont(pui8Payload, ui8Length);
	}
	else
	{
		// Error, cannot specify a length of 1
		return;
	}
}

//===============================================================
//
// Trf797xWriteIsoControl - Write to TRF797x ISO Control Register
//
// \param ui8IsoControl is the value to write to the ISO control
// register of the TRF797x
//
// Function used to write a new value into the ISO Control
// register of the TRF797x.
//
// \return None.
//
//===============================================================

void
Trf797xWriteIsoControl(u08_t ui8IsoControl)
{
	u08_t pui8Write[2];

	if((ui8IsoControl & BIT5) == BIT5)
	{
		// Attempt to enable Card Emulation/Peer to Peer which is not supported by firmware
		// Exit function to avoid issues with that
		return;
	}

	pui8Write[0] = TRF797x_ISO_CONTROL;
	pui8Write[1] = ui8IsoControl;
	pui8Write[1] &= ~BIT5;
	Trf797xWriteSingle(pui8Write, 2);

	g_ui8IsoControlValue = ui8IsoControl;	// Update the ISO Control Register variable
}

//===============================================================
//
// Trf797xWriteRegister - Write single to a TRF797x Register
//
// \param ui8TrfRegister is the register address for the write
// \param ui8Value is the value to write to the specified
// register
//
// Function used to write a new value into a single TRF797x
// register.
//
// \return None.
//
//===============================================================

void
Trf797xWriteRegister(u08_t ui8TrfRegister, u08_t ui8Value)
{
	u08_t pui8Write[2];

	if (ui8TrfRegister == TRF797x_ISO_CONTROL)
	{
		// Attempt to enable Card Emulation/Peer to Peer which is not supported by firmware
		// Exit function to avoid issues with that
		if ((ui8Value & BIT5) == BIT5)
		{
			return;
		}

		g_ui8IsoControlValue = ui8Value;	// Update the ISO Control Register variable
	}
	if (ui8TrfRegister == TRF797x_STATUS_CONTROL)
	{
		if ((ui8Value & BIT5) == BIT5)	// Check for RF field bit and update variable
		{
			g_bRfFieldOn = true;
		}
		else
		{
			g_bRfFieldOn = false;
		}
	}

	pui8Write[0] = ui8TrfRegister;
	pui8Write[1] = ui8Value;
	Trf797xWriteSingle(pui8Write, 2);
}

//===============================================================
//
// Trf797xWriteInitiatorSetup - Write the initial settings for
// a set of TRF797x registers based on which protocol is to be
// enabled.
//
// \param ui8IsoControl is the value to write to the ISO Control
// register of the TRF797x
//
// Function used to write to a series of TRF797x registers based
// on which technology will be enabled in the ISO control register
// This function currently only enables 1 technology at a time
//
// \return None.
//
//===============================================================

void
Trf797xWriteInitiatorSetup(u08_t ui8IsoControl)
{
	u08_t pui8Write[2];
	u08_t write[16];

	g_ui8IsoControlValue = ui8IsoControl;	// Update the ISO Control Register variable

	if (ui8IsoControl == 0x88) // ISO14443A
	{


		// Register 0x00
		pui8Write[0] = TRF797x_STATUS_CONTROL;
		pui8Write[1] = 0x20;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x01; // Sys Clock Output = 13.56MHz, OOK = 100%
		Trf797xWriteSingle(pui8Write, 2);
	}

	if (ui8IsoControl == 0x0C) // ISO14443B
	{
		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x00; // Sys Clock Output = 13.56MHz, ASK 10%
		Trf797xWriteSingle(pui8Write, 2);
	}

	if (ui8IsoControl == 0x02) // ISO15693
	{

		pui8Write[0] = TRF797x_STATUS_CONTROL;
		pui8Write[1] = 0x20;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Resgister 0x07 - No Response Wait Time
		pui8Write[0] = TRF797x_RX_NO_RESPONSE_WAIT_TIME;
		pui8Write[1] = 0x15;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x01; // Sys Clock Output = 13.56MHz, OOK = 100%
		Trf797xWriteSingle(pui8Write, 2);




		/*		write[0] = 0x20;                  //Continuous Write, starting with register 0x00
			    write[1] = 0x20;                  //Value for Chip Status Control Register 0x00, 0x20 = +3.3VDC, full power, etc.
			    write[2] = 0x02;                  //Value for ISO Control Register 0x01, 0x02 = high tag data rate, etc.
			    write[3] = 0x00;				  //0x02
			    write[4] = 0x00;
			    write[5] = 0xC1;
			    write[6] = 0xBB;
			    write[7] = 0x00;
			    write[8] = 0x30;
			    write[9] = 0x1F;
			    write[10] = 0x01;
			    write[11] = 0x40;
			    write[12] = 0x03;*/

	/*	// Register 0x00
		pui8Write[0] = TRF797x_STATUS_CONTROL;
		pui8Write[1] = 0x20;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x02
		pui8Write[0] = TRF797x_ISO_14443_TX_OPTIONS;
		pui8Write[1] = 0x00;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x03
		pui8Write[0] = TRF797x_ISO_14443_BITRATE_OPTIONS;
		pui8Write[1] = 0x00;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x04
		pui8Write[0] = TRF797x_TX_TIMER_EPC_HIGH;
		pui8Write[1] = 0xC1;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x05
		pui8Write[0] = TRF797x_TX_TIMER_EPC_LOW;
		pui8Write[1] = 0xBB;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x06
		pui8Write[0] = TRF797x_TX_PULSE_LENGTH_CONTROL;
		pui8Write[1] = 0x00;
		Trf797xWriteSingle(pui8Write, 2);

		// Resgister 0x07 - No Response Wait Time
		pui8Write[0] = TRF797x_RX_NO_RESPONSE_WAIT_TIME;
		pui8Write[1] = 0x30;
		Trf797xWriteSingle(pui8Write, 2);

		// Resgister 0x08 -
		pui8Write[0] = TRF797x_RX_WAIT_TIME;
		pui8Write[1] = 0x1F;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x01; // Sys Clock Output = 13.56MHz, OOK = 100%
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x0A
		pui8Write[0] = TRF797x_RX_SPECIAL_SETTINGS;
		pui8Write[1] = 0x40;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x0B
		pui8Write[0] = TRF797x_REGULATOR_CONTROL;
		pui8Write[1] = 0x03;
		Trf797xWriteSingle(pui8Write, 2);
*/


/*		write[0] = 0x20;                  //Continuous Write, starting with register 0x00
	    write[1] = 0x20;                  //Value for Chip Status Control Register 0x00, 0x20 = +3.3VDC, full power, etc.
	    write[2] = 0x02;                  //Value for ISO Control Register 0x01, 0x02 = high tag data rate, etc.
	    write[3] = 0x00;				  //0x02
	    write[4] = 0x00;
	    write[5] = 0xC1;
	    write[6] = 0xBB;
	    write[7] = 0x00;
	    write[8] = 0x30;
	    write[9] = 0x1F;
	    write[10] = 0x01;
	    write[11] = 0x40;
	    write[12] = 0x03;*/

	    //Trf797xWriteCont(write, 13);      //writes registers 0x00:0x0B

	}

	if (ui8IsoControl == 0x1A) // FeliCa
	{
		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x00; // Sys Clock Output = 13.56MHz, ASK 10%
		Trf797xWriteSingle(pui8Write, 2);
	}

	if (ui8IsoControl == 0x86) // PicoPass
	{
		// Register 0x01
		pui8Write[0] = TRF797x_ISO_CONTROL;
		pui8Write[1] = ui8IsoControl;
		Trf797xWriteSingle(pui8Write, 2);

		// Register 0x09 - System Clock Output, Modulation Scheme
		pui8Write[0] = TRF797x_MODULATOR_CONTROL;
		pui8Write[1] = 0x00; // Sys Clock Output = 13.56MHz, ASK 10%
		Trf797xWriteSingle(pui8Write, 2);
	}

	// Register 0x14 - Adjustable FIFO Level
	pui8Write[0] = TRF797x_FIFO_IRQ_LEVELS;
	if (ui8IsoControl == 0x86)
	{
		// TX water level set to 32 bytes for PicoPass
		pui8Write[1] = 0x0F;
	}
	else
	{
		pui8Write[1] = 0x0C;
	}
	Trf797xWriteSingle(pui8Write, 2);

}

//===============================================================
//
// Trf797xWriteSingle - Write single to a TRF797x Register
//
// \param pui8Value is a pointer to a buffer which has the
// Register address for the write followed by the data to be
// written to that register
//
// Function used to write a new value into a single TRF797x
// register.
//
// \return None.
//
//===============================================================

void
Trf797xWriteSingle(u08_t *pbuf, u08_t length)
{
	SpiWriteSingle(pbuf, length);
}

//===============================================================
//
// Trf797xIrqWaitTimeout - Timeout sequence for both TX and RX
//
// \param ui8TxTimeout is the TX timeout in milliseconds
// \param ui8RxTimeout is the RX timeout in milliseconds
//
// Function is used to ensure data is transmitted correct as well
// as determine if data has been received prior to the RX timeout
// When the RX timeout occurs before data is received, then mark
// the TRF797x status as a No Response Received status.
//
// \return None.
//
//===============================================================

void Trf797xIrqWaitTimeout(long ui8TxTimeout, long ui8RxTimeout)
{
	g_ui8FifoOffset = 0; // Reset the FIFO Offset prior to receiving data

	g_sTrf797xStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;
	while((g_sTrf797xStatus != TX_COMPLETE) && (g_sTrf797xStatus != TX_ERROR))
	{	// Wait for end of TX
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;
		// Setup for the Timer
		// Calculate the timeout value for the timer
		A2CounterLoad(COUNT_1ms * ui8TxTimeout);
		// Start the Timeout
		START_COUNTER;
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
		{
			// Do Nothing
		}
		RESET_COUNTER;
		STOP_COUNTER;
		if (g_sTrf797xStatus != TX_COMPLETE)
		{
			if (g_sTrf797xStatus == TX_WAIT)	// Wait longer since we received an 0xA0
			{
				UartSendCString("Received TX_WAIT\n\r");
				Trf797xIrqWaitTimeoutTxOnly(ui8TxTimeout);	// Wait longer for transmission to complete
			}
			else	// Failed to send packet properly - Exit TX Timeout
			{
				g_sTrf797xStatus = TX_ERROR;	// Set status to error
			}
		}
	}

	//StatusPrint(g_sTrf797xStatus);
	if (g_sTrf797xStatus != TX_ERROR)
	{
		g_sTrf797xStatus = RX_WAIT;
		g_ui8TimeoutFlag = 0x00;
		while(g_sTrf797xStatus == RX_WAIT || g_sTrf797xStatus == TX_COMPLETE)		// Wait for end of RX or timeout
		{
			// Clear the IRQ Flag
			g_ui8IrqFlag = 0x00;
			// Setup for the Timer
			// Calculate the timeout value for the timer
			A2CounterLoad(COUNT_1ms * ui8RxTimeout);
			//A2CounterLoad(COUNT_1ms * 10000);
			// Start the Timeout
			START_COUNTER;
			while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
			{
				// Do Nothing
			}
			//StatusPrint(g_sTrf797xStatus);
			while (g_sTrf797xStatus == RX_WAIT_EXTENSION || g_sTrf797xStatus == TX_WAIT)
			{
				RESET_COUNTER;
				STOP_COUNTER;

				g_ui8IrqFlag = 0x00;
				if ((g_ui8IsoControlValue & 0x1F) > 0x07)
				{
					A2CounterLoad(COUNT_1ms * 7);
				}
				else
				{
					A2CounterLoad(COUNT_1ms * 50);
				}
				START_COUNTER;
				while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
				{
					// Do Nothing
				}
			}
			RESET_COUNTER;
			STOP_COUNTER;

			if (g_sTrf797xStatus == RX_WAIT)
			{
				// Exit the while loop
				g_sTrf797xStatus = NO_RESPONSE_RECEIVED;
			}
		}
	}

}

//===============================================================
//
// Trf797xIrqWaitTimeoutTxOnly - Timeout sequence for just TX
//
// \param ui8TxTimeout is the TX timeout in milliseconds
//
// Function is used to handle delays for transmit only, which
// is helpful when a response is not expected (such as with sleep
// or halt commands)
// This function will not delay to receive any responses
//
// \return None.
//
//===============================================================

void Trf797xIrqWaitTimeoutTxOnly(u08_t ui8TxTimeout)
{
	g_sTrf797xStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;
	while((g_sTrf797xStatus != TX_COMPLETE) && (g_sTrf797xStatus != NO_RESPONSE_RECEIVED) && (g_sTrf797xStatus != TX_ERROR))
	{										// Wait for end of TX
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;
		// Setup for the Timer
		// Calculate the timeout value for the timer
		A2CounterLoad(COUNT_1ms * ui8TxTimeout);
		// Start the Timeout
		START_COUNTER;
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
		{
			// Do Nothing
		}
		RESET_COUNTER;
		STOP_COUNTER;

		if (g_sTrf797xStatus != TX_COMPLETE)
		{
			if (g_sTrf797xStatus == TX_WAIT)	// Wait longer since we received an 0xA0
			{
				Trf797xIrqWaitTimeoutTxOnly(ui8TxTimeout);	// Wait longer for transmission to complete
			}
			else	// Failed to send packet properly - Exit TX Timeout
			{
				g_sTrf797xStatus = TX_ERROR;	// Set status to error
			}
		}
	}
}


//===============================================================
//
// Trf797xIrqWaitTimeoutRxOnly - Timeout sequence for just RX
//
// \param ui8RxTimeout is the RX timeout in milliseconds
//
// Function is used to handle delays for receive only, such as
// when waiting for a response without sending a data package
// during anticollision slot procedures.
//
// \return None.
//
//===============================================================

void Trf797xIrqWaitTimeoutRxOnly(u08_t ui8RxTimeout)
{
	g_ui8FifoOffset = 0; // Reset the FIFO Offset prior to receiving data

	g_sTrf797xStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;
	while(g_sTrf797xStatus == RX_WAIT)		// Wait for end of RX or timeout
	{
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;
		// Setup for the Timer
		// Calculate the timeout value for the timer
		A2CounterLoad(COUNT_1ms * ui8RxTimeout);
		// Start the Timeout
		START_COUNTER;
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
		{
			// Do Nothing
		}
		while (g_sTrf797xStatus == RX_WAIT_EXTENSION)
		{
			RESET_COUNTER;
			STOP_COUNTER;

			g_ui8IrqFlag = 0x00;
			if ((g_ui8IsoControlValue & 0x1F) > 0x07)
			{
				A2CounterLoad(COUNT_1ms * 7);
			}
			else
			{
				A2CounterLoad(COUNT_1ms * 50);
			}
			START_COUNTER;
			while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
			{
				// Do Nothing
			}
		}
		RESET_COUNTER;
		STOP_COUNTER;

		if (g_sTrf797xStatus == RX_WAIT)
		{
			// Exit the while loop
			g_sTrf797xStatus = NO_RESPONSE_RECEIVED;
		}
	}
}

//===============================================================
//
// Trf797xIrqWaitTimeoutFeliCa - Timeout sequence for just RX
// specifically for FeliCa slot timings
//
// Function is used to handle receive delays for FeliCa
// anticollision only.
//
// \return None.
//
//===============================================================

void Trf797xIrqWaitTimeoutFeliCa(void)
{
	g_ui8FifoOffset = 0; // Reset the FIFO Offset prior to receiving data

	g_sTrf797xStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;
	while(g_sTrf797xStatus == RX_WAIT)		// Wait for end of RX or timeout
	{
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;
		// Setup for the Timer
		// Calculate the timeout value for the timer
		A2CounterLoad(2);

		// Start the Timeout
		START_COUNTER;
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00))	// Wait for an interrupt
		{
			// Do Nothing
		}
		RESET_COUNTER;
		STOP_COUNTER;

		if (g_sTrf797xStatus == RX_WAIT)
		{
			// Exit the while loop
			g_sTrf797xStatus = NO_RESPONSE_RECEIVED;
		}
	}
}


//===============================================================
//
// Trf797xGetTrfStatus - Returns current TRF797x driver status
//
// Function is used to pass the current TRF797x driver status
// up to higher layers of the firmware
//
// \return g_sTrf797xStatus returns the current TRF797x drive
// status
//
//===============================================================

tTRF797x_Status Trf797xGetTrfStatus(void)
{
	return g_sTrf797xStatus;
}

//===============================================================
//
// Trf797xSetTrfStatus - Set the TRF797x driver status
//
// \param sTrfStatus is the new TRF797x driver status
//
// Function is used to allow higher layers of the firmware to
// set the TRF797x driver status without an IRQ event. Use with
// caution.
//
// \return None.
//
//===============================================================

void Trf797xSetTrfStatus(tTRF797x_Status sTrfStatus)
{
	g_sTrf797xStatus = sTrfStatus;
}

//===============================================================
//
// Trf797xGetCollisionPosition - Return the current Collision
// Position value
//
// Function is used to pass the current Collision Position value
// up to higher layers of the firmware. Used for ISO14443 Type A
// anti-collision process.
//
// \return g_ui8CollisionPosition returns the current Collision
// Position value
//
//===============================================================

u08_t Trf797xGetCollisionPosition(void)
{
	return g_ui8CollisionPosition;
}

//===============================================================
//
// Trf797xSetCollisionPosition - Set the Collision Position
// variable
//
// \param ui8ColPos is the new Collision Position value
//
// Function is used to allow higher layers of the firmware to
// set the Collision Position variable. Used for ISO14443 Type A
// anti-collision process
//
// \return None.
//
//===============================================================

void Trf797xSetCollisionPosition(u08_t ui8ColPos)
{
	g_ui8CollisionPosition = ui8ColPos;
}

//===============================================================
//
// Trf797xGetRxBytesReceived - Returns the Number of RX Bytes
// received by the TRF797x FIFO
//
// Function is used to pass the number of RX bytes received
// during the last packet reception up to higher layers of the
// firmware.
// This can be used by the application layer to check for the
// length of bytes received and ensure packets were correctly
// received.
//
// \return g_ui8FifoRxLength returns the current FIFO RX Length
//
//===============================================================

u08_t Trf797xGetRxBytesReceived(void)
{
	return g_ui8FifoRxLength;
}

//===============================================================
//
// Trf797xGetIsoControlValue - Fetch the latest Iso Control
// Register value
//
// Function is used to pass the current Iso Control Register
// value up to higher layers of the firmware.
//
// The Iso Control Register value is updated whenever a read or
// write to the Iso Control Register occurs.
//
// \return g_ui8IsoControlValue returns the current ISO Control
// Register value
//
//===============================================================

u08_t Trf797xGetIsoControlValue(void)
{
	return g_ui8IsoControlValue;
}

//===============================================================
//
// Trf797xReadRssiLevels - Reads the RSSI Levels register and then
// returns the value of the latest RSSI measurement
//
// Function is used to determine the latest RSSI Level reading.
// The RSSI Level register is updated when RF data is received
// by the TRF797x. For most accurate reading, this function should
// be called directly following a reception of RF data.
//
// \return pui8Read returns the latest RSSI Register value
//
//===============================================================

u08_t Trf797xReadRssiLevels(void)
{
	u08_t pui8Read[1];

	pui8Read[0] = TRF797x_RSSI_LEVELS;		// read RSSI levels
	Trf797xReadSingle(pui8Read, 1);

	return pui8Read[0];
}

//===============================================================
//
// Trf797xCheckRfField - Fetch the RfFieldOn variable
//
// Function is used to pass the current RfFieldOn value up to
// higher layers of the firmware
//
// \return g_bRfFieldOn retuns the current RF field status
//
//===============================================================

bool Trf797xCheckRfField(void)
{
	return g_bRfFieldOn;
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

	u08_t ui8IrqStatus;
	ui8IrqStatus = 0x00;

	STOP_COUNTER;
	RESET_COUNTER;

	g_ui8TimeoutFlag = 0x01;

	Trf797xReadIrqStatus(&ui8IrqStatus);

	ui8IrqStatus = ui8IrqStatus & 0xF7;		// Set the parity flag to 0

	if(ui8IrqStatus == TRF797x_IRQ_STATUS_TX_COMPLETE)
	{
		g_sTrf797xStatus = TX_COMPLETE;
	}
	else if(ui8IrqStatus == TRF797x_IRQ_STATUS_IDLE)
	{
		g_sTrf797xStatus = NO_RESPONSE_RECEIVED;
	}
	else
	{
		g_sTrf797xStatus = RX_WAIT;
	}
}

void StatusPrint(tTRF797x_Status sTrfStatus) {
	switch (sTrfStatus) {
	case TRF_IDLE:
		UartSendCString("Status: TRF_IDLE\n\r");
		break;
	case TX_COMPLETE:
			UartSendCString("Status: TX_COMPLETE\n\r");
			break;
	case RX_COMPLETE:
			UartSendCString("Status: RX_COMPLETE\n\r");
			break;
	case TX_ERROR:
			UartSendCString("Status: TX_ERROR\n\r");
			break;
	case RX_WAIT:
			UartSendCString("Status: RX_WAIT\n\r");
			break;
	case RX_WAIT_EXTENSION:
			UartSendCString("Status: RX_WAIT_EXTENSION\n\r");
			break;
	case TX_WAIT:
			UartSendCString("Status: TX_WAIT\n\r");
			break;
	case PROTOCOL_ERROR:
			UartSendCString("Status: PROTOCOL_ERROR\n\r");
			break;
	case NO_RESPONSE_RECEIVED:
			UartSendCString("Status: NO_RESPONSE_RECEIVED\n\r");
			break;
	case NO_RESPONSE_RECEIVED_15693:
			UartSendCString("Status: NO_RESPONSE_RECEIVED_15693\n\r");
			break;
	default:
		UartSendCString("Status: Unknown\n\r");
		break;
	}
}
