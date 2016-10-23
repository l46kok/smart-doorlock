/*
 * File Name: iso14443a.c
 *
 * Description: ISO14443A Specific Functions
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

#include "hw_memmap.h"
#include "iso14443a.h"
#include "trf7970BoosterPack.h"
#include "uart_if.h"
#include "gpio.h"
#include "gpio_if.h"

//===============================================================
//		Global Variables
//===============================================================

extern u08_t g_ui8TrfBuffer[NFC_FIFO_SIZE];
static volatile tTRF797x_Status g_sTrfStatus;

static tISO14443A_UidSize g_sUidSize = ISO14443A_UID_UNKNOWN;
static u08_t g_pui8CompleteUid[10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static u08_t g_pui8PartialUid[5] = {0x00,0x00,0x00,0x00,0x00};
static u08_t g_ui8UidPos = 0;
static u08_t g_ui8ValidUidByteCount = 0;
static u08_t g_ui8ValidUidBitCount = 0;
static u08_t g_ui8ValidBits;

static u08_t g_ui8RecursionCount = 0;
static u08_t g_ui8MaxRecurviseCalls = 5;

static u08_t g_ui8Iso14443aSAK;
static bool g_bType4ACompliant = false;
static u08_t g_ui8AtsSupportedBitrates = 0x00; // This is used to store the ATS reply for TA(1) which contains the Tags supported bitrates - needed to determine PPS request parameters.

//===============================================================
//
// Iso14443a_TagSelection - Process to detect and select
// ISO14443A/NFC Type 2/4A Tag Platform compliant tags.
//
// \param ui8Command is the Polling command to be issued by
// the TRF797x for ISO14443A tag detection.
//
// This function issues polling command, processes the ATQA, and
// handles sending correct anticollision and selection commands.
//
// When a collision occurs, this function will call the
// anticollision function to handle the collision.
//
// \return ui8Status returns whether or not an ISO14443A
// compliant tag has been successfully selected.
//
//===============================================================

u08_t Iso14443a_TagSelection(u08_t ui8Command)
{
	u08_t ui8Index = 0;
	u08_t ui8LoopCount = 0;
	u08_t ui8Status = STATUS_FAIL;
	bool bSendCT = false;
	tISO14443A_UidStatus sUidProgress = CASCADE1;
	tCollisionStatus sCollisionStatus = NO_COLLISION;

	// Clear UID to store new one
	for(ui8Index = 0; ui8Index < 10; ui8Index++)
	{
		g_pui8CompleteUid[ui8Index] = 0x00;
	}
	// Clear partial UID buffer
	for (ui8LoopCount = 0; ui8LoopCount < 5; ui8LoopCount++)
	{
		g_pui8PartialUid[ui8LoopCount] = 0x00;
	}

	g_ui8UidPos = 0;				// Reset UID Position Marker
	g_ui8ValidUidByteCount = 0;		// Reset Valid Bytes Received Counter
	g_ui8ValidUidBitCount = 0; 		// Reset Valid Bits Received Counter
	g_ui8Iso14443aSAK = 0;			// Reset the SAK
	g_bType4ACompliant = false; 	// Reset Type 4A Compliance
	g_ui8AtsSupportedBitrates = 0;	// Reset the ATS Reply for TA(1)
	g_ui8ValidBits = 0; 			// Clear Valid Bit global

	// Poll for a ISO14443A tag
	if (Iso14443a_PollingCommand(ui8Command))
	{
		if (g_sTrfStatus == RX_COMPLETE)
		{
			// Check ATQA Response for UID size
			if ((g_ui8TrfBuffer[0] & 0xC0) == 0x00)
			{
				g_sUidSize = ISO14443A_UID_SINGLE;
			}
			else if ((g_ui8TrfBuffer[0] & 0xC0) == 0x40)
			{
				g_sUidSize = ISO14443A_UID_DOUBLE;
			}
			else if ((g_ui8TrfBuffer[0] & 0xC0) == 0x80)
			{
				g_sUidSize = ISO14443A_UID_TRIPLE;
			}
			else
			{
				g_sUidSize = ISO14443A_UID_UNKNOWN;
			}
		}
		else
		{
			// Collision occurred, UID size not known
			g_sUidSize = ISO14443A_UID_UNKNOWN;
		}
	}
	else
	{
		// No response to polling command, exit function
		ui8Status = STATUS_FAIL;
		g_ui8RecursionCount = 0; // Reset the recursion count for the anticollision loops
		return ui8Status;
	}

	while (sUidProgress != UID_COMPLETE)
	{

		sCollisionStatus = Iso14443a_AnticollisionCommand(sUidProgress, NVB_INIT, &g_pui8CompleteUid[0]);	// Call anticollision loop function

		// Process the response
		if (sCollisionStatus == NO_COLLISION)
		{
			// Store the UID and keep track if the CT byte needs to be sent
			bSendCT = Iso14443a_StoreUid(sUidProgress,&g_ui8TrfBuffer[0]);

			// Issue Select command
			if (Iso14443a_SelectCommand(sUidProgress,&g_pui8CompleteUid[g_ui8UidPos],bSendCT)) 	// Issue the Select Command
			{
				// If successful, use SAK information to determine if the UID is complete
				if ((g_ui8Iso14443aSAK & BIT2) == 0x00)
				{
					// UID complete, set status to success
					ui8Status = STATUS_SUCCESS;
					sUidProgress = UID_COMPLETE;

					if (g_sUidSize == ISO14443A_UID_UNKNOWN)
					{
						if (sUidProgress == CASCADE1)
						{
							g_sUidSize = ISO14443A_UID_SINGLE;
						}
						else if (sUidProgress == CASCADE2)
						{
							g_sUidSize = ISO14443A_UID_DOUBLE;
						}
						else if (sUidProgress == CASCADE3)
						{
							g_sUidSize = ISO14443A_UID_TRIPLE;
						}
					}
				}
				else
				{
					// UID is not Complete, increase cascade level, update UidSize as well
					if (sUidProgress == CASCADE1)
					{
						sUidProgress = CASCADE2;
						if (g_sUidSize == ISO14443A_UID_UNKNOWN)
						{
							g_sUidSize = ISO14443A_UID_DOUBLE;
						}
					}
					else if (sUidProgress == CASCADE2)
					{
						sUidProgress = CASCADE3;
						if (g_sUidSize == ISO14443A_UID_UNKNOWN)
						{
							g_sUidSize = ISO14443A_UID_TRIPLE;
						}
					}
					else
					{
						// Either Cascade was already CASCADE3 or an error occured, so break to hit re-try loop
						sUidProgress = UID_INCOMPLETE;
						break;
					}

				}
			}
			else
			{
				// Break to hit the re-try loop
				sUidProgress = UID_INCOMPLETE;
				break;
			}
		}
		else if (sCollisionStatus == COLLISION)
		{
			Report("COLLISION Occurred\n\r");
			// If a collision occurs, call the Anticollision loop to handle tag collisions
			sCollisionStatus = Iso14443a_AnticollisionLoop(sUidProgress);

			// Check if the anticollision loop is successful
			if (sCollisionStatus == NO_COLLISION)
			{
				// If successful, use SAK information to determine if the UID is complete
				if ((g_ui8Iso14443aSAK & BIT2) == 0x00)
				{
					// UID complete, set status to success
					ui8Status = STATUS_SUCCESS;
					sUidProgress = UID_COMPLETE;
				}
				else
				{
					// UID is not Complete, increase cascade level, update UidSize as well
					if (sUidProgress == CASCADE1)
					{
						sUidProgress = CASCADE2;
					}
					else if (sUidProgress == CASCADE2)
					{
						sUidProgress = CASCADE3;
					}
					else
					{
						// Either Cascade was already CASCADE3 or an error occured, so break to hit re-try loop
						sUidProgress = UID_INCOMPLETE;
						break;
					}
				}
			}
			else
			{
				break;
			}
		}
		else
		{
			// Other error occurred, do not proceed
			sUidProgress = UID_INCOMPLETE;
			g_ui8RecursionCount = 0; // Reset the recursion count for the anticollision loops
			return ui8Status;
		}
	}

	if (sUidProgress == UID_INCOMPLETE)
	{	// Some error occurred, attempt to find the tag again

		if (g_ui8RecursionCount < g_ui8MaxRecurviseCalls)
		{
			Report("UID Incomplete: Recursively calling TAG find\n\r");
			g_ui8RecursionCount++;
			ui8Status = Iso14443a_TagSelection(ui8Command);
		}
		else
		{
			g_ui8RecursionCount = 0; // Reset the recursion count for the anticollision loops
			return ui8Status;
		}
	}

	// This won't repetively trigger after the recursive call of Iso14443a_TagSelection since the sUidProgress will not change
	if (sUidProgress == UID_COMPLETE)
	{

#ifdef ENABLE_HOST
		// UID Completed
		UartPutCrlf();
		Report("Anticollison Completed");
		UartPutCrlf();

		// Output UID to UART Terminal
		Report("ISO14443A UID:  ");
		UartPutChar('[');
		if (g_sUidSize == ISO14443A_UID_UNKNOWN)	// Assume ID is a single if it has not been defined to this point.
		{
			g_sUidSize = ISO14443A_UID_SINGLE;
		}
		for (ui8LoopCount=0; ui8LoopCount<g_sUidSize; ui8LoopCount++)
		{
			UartPutByte(g_pui8CompleteUid[ui8LoopCount]);
		}
		UartPutChar(']');
		UartPutCrlf();
#endif
		// Output compliance to ISO14443-4
		if (g_ui8Iso14443aSAK & BIT5)
		{
#ifdef ENABLE_HOST
			Report("Tag is ISO14443-4 Compliant");
			UartPutCrlf();
#endif
			g_bType4ACompliant = true;
		}
		else
		{
#ifdef ENABLE_HOST
			Report("Tag is not ISO14443-4 Compliant");
			UartPutCrlf();
#endif
			g_bType4ACompliant = false;
		}
#ifdef ENABLE_HOST
		UartPutCrlf();
#endif
	}

	g_ui8RecursionCount = 0; // Reset the recursion count for the anticollision loops after anticollision is finished
	return ui8Status;
}

//===============================================================
//
// Iso14443a_AnticollisionLoop - Issue the polling command for
// ISO14443A compliant tags.
//
// \param sCascade is the current anticollision cascade.
//
// This function handles the ISO14443A anticollision procedures
// including dealing with receiving broken bytes and issuing
// anticollisions based on those broken bytes. It will run until
// a complete UID is received unless an error occurs.
//
// This function uses a recursive call for the anticollision
// process.
//
// \return sStatus returns a status based on what tag response is
// received following the most recently issued Anticollision
// command.
//
//===============================================================

tCollisionStatus Iso14443a_AnticollisionLoop(tISO14443A_UidStatus sCascade)
{
	u08_t ui8NVB = NVB_INIT;
	u08_t	ui8NVBytes = 0;
	u08_t	ui8NVBits = 0;
	u08_t	ui8NVBitCount = 0;
	u08_t	ui8CollisionPosition = 0;
	u08_t	ui8LoopCount = 0;
	tCollisionStatus sStatus = COLLISION;

	// Note: The g_ui8UidPos will be used differently in this function in that it will track where to place received
	// valid UID bytes rather than mark the location of the first byte as it does in all other functions.
	// When a full UID is received, the Iso14443a_StoreUid function will be called which will restore the g_ui8UidPos
	// value to what is expected by the rest of the firmware. If a microcontroller with larger RAM reserves is used,
	// a seperate global variable could be used instead.

	ui8CollisionPosition = Trf797xGetCollisionPosition();		// Get the collision position information from the TRF driver

	ui8NVBytes = (ui8CollisionPosition >> 4) - 2;	// This represents the number of known valid bytes of the UID
	ui8NVBitCount = ui8CollisionPosition & 0x07;	// This represents the number of known valid bits of the UID (can't be more than 8 or else it would be a valid byte)

	g_ui8ValidUidBitCount = ui8NVBitCount;			// Set the valid bit count to be equal to the received value from the TRF

	// Use the number of bits received to generate the value of the bits received so far
	for(ui8LoopCount = 0; ui8LoopCount < ui8NVBitCount; ui8LoopCount++)
	{
		ui8NVBits = (ui8NVBits << 1) + 1;			// Store the info for the valid bits
	}

	if (g_ui8ValidUidByteCount < ui8NVBytes)
	{
		if ((ui8NVBytes-g_ui8ValidUidByteCount) > 5)
		{
			sStatus = COLLISION_ERROR;
			return sStatus;
		}

		// Store the received bytes of the UID in a storage buffer
		for (ui8LoopCount = 0; ui8LoopCount < (ui8NVBytes-g_ui8ValidUidByteCount); ui8LoopCount++)
		{
			g_pui8PartialUid[ui8LoopCount+g_ui8UidPos] |= g_ui8TrfBuffer[ui8LoopCount];
		}

		g_ui8UidPos = ui8LoopCount+g_ui8UidPos;		// Set the UID Position indicator to the next array index

		if (g_ui8UidPos > 4)
		{
			sStatus = COLLISION_ERROR;
			return sStatus;
		}

		g_ui8ValidUidByteCount = ui8NVBytes;		// Set the Valid byte count equal to what was received by the TRF

		// Store the received bits of the UID in a storage buffer
				// "The valid bits shall be part of the UID CLn that was received before a collision occurred
				// followed by a (0)b or (1)b, decided by the PCD. A typical implementation adds a (1)b."
		if (g_ui8ValidUidBitCount < 7)
		{
			// Since the valid bits are not at the maximum amount of bits allowed, add the extra bit at the end of the UID
			g_pui8PartialUid[g_ui8UidPos] = ((g_ui8TrfBuffer[g_ui8ValidUidByteCount] & ui8NVBits));
			g_ui8ValidBits = g_pui8PartialUid[g_ui8UidPos];		// Save the current valid bits in a variable

			// NVB is equivalent to the value received in the TRF797x Collision Position Register plus one for the extra bit added per the standard.
			ui8NVB = ui8CollisionPosition+1; 		// "The PCD shall assign NVB with a value that specifies the number of valid bits of UID CLn."
			g_ui8ValidUidBitCount++;				// Increment the valid bit count by one to mark the extra bit which was added per specifications.
		}
		else
		{
			g_pui8PartialUid[g_ui8UidPos] = (g_ui8TrfBuffer[g_ui8ValidUidByteCount] & ui8NVBits);
			g_ui8ValidBits = g_pui8PartialUid[g_ui8UidPos];		// Save the current valid bits in a variable

			// NVB is equivalent to the value received in the TRF797x Collision Position Register plus one for the extra bit added per the standard.
			ui8NVB = ui8CollisionPosition; 			// "The PCD shall assign NVB with a value that specifies the number of valid bits of UID CLn."
		}
	}
	else
	{
		// Update the valid bits based on the current valid bits as well as the newly received valid bits from the tag response
				// "The valid bits shall be part of the UID CLn that was received before a collision occurred
				// followed by a (0)b or (1)b, decided by the PCD. A typical implementation adds a (1)b."
		if (g_ui8ValidUidBitCount < 7)
		{
			// Since the valid bits are not at the maximum amount of bits allowed, add the extra bit at the end of the UID
			g_pui8PartialUid[g_ui8UidPos] = (g_ui8ValidBits | ((g_ui8TrfBuffer[0] & ui8NVBits) << (g_ui8ValidUidBitCount-ui8NVBitCount)));
			g_ui8ValidBits = g_pui8PartialUid[g_ui8UidPos];		// Save the current valid bits in a variable

			ui8NVB = ui8CollisionPosition+1; 		// "The PCD shall assign NVB with a value that specifies the number of valid bits of UID CLn."
			g_ui8ValidUidBitCount++;				// Increment the valid bit count by one to mark the extra bit which was added per specifications.
		}
		else
		{
			g_pui8PartialUid[g_ui8UidPos] = (g_ui8ValidBits | ((g_ui8TrfBuffer[0] & ui8NVBits) << (g_ui8ValidUidBitCount-ui8NVBitCount)));
			g_ui8ValidBits = g_pui8PartialUid[g_ui8UidPos];		// Save the current valid bits in a variable

			ui8NVB = ui8CollisionPosition; 			// "The PCD shall assign NVB with a value that specifies the number of valid bits of UID CLn."
		}
	}

	McuDelayMillisecond(1);							// Small delay prior to sending out packet.

	sStatus = Iso14443a_AnticollisionCommand(sCascade,ui8NVB,&g_pui8PartialUid[0]);				// Issue anti-collision command with the partial UID

	if (sStatus == NO_COLLISION)
	{
		// No Collision means the anticollision command was successful and the remaining bytes were received

		g_pui8PartialUid[g_ui8UidPos] = g_pui8PartialUid[g_ui8UidPos] | g_ui8TrfBuffer[0]; 		// Combine broken byte with 1st received byte to finalize the 1st byte of the UID
		g_ui8UidPos++;								// Increment the UID Position Indicator
		g_ui8ValidUidBitCount = 0;					// Reset the valid bit counter
		g_ui8ValidBits = 0;							// Reset the valid bit variable

		// Store the other bytes
		for (ui8LoopCount = 0; ui8LoopCount < (5-g_ui8UidPos); ui8LoopCount++)
		{
			g_pui8PartialUid[ui8LoopCount+g_ui8UidPos] = g_ui8TrfBuffer[(ui8LoopCount+1)];	// Store remaining received UID bytes into the partial UID buffer
		}

		// Issue the Select Command with the fully received UID
		if (Iso14443a_SelectCommand(sCascade,&g_pui8PartialUid[0],false))
		{
			// Received the SAK from the Select Command (This is stored in a global within the Select function)

			// If the UID Size is not known yet
			if (g_sUidSize == ISO14443A_UID_UNKNOWN)
			{
				// Use SAK information to determine if the UID is complete
				if ((g_ui8Iso14443aSAK & BIT2) == 0x00)
				{
					if (sCascade == CASCADE1)
					{
						g_sUidSize = ISO14443A_UID_SINGLE;
					}
					else if (sCascade == CASCADE2)
					{
						g_sUidSize = ISO14443A_UID_DOUBLE;
					}
					else if (sCascade == CASCADE3)
					{
						g_sUidSize = ISO14443A_UID_TRIPLE;
					}
				}
				else
				{
					// UID is not complete, update UidSize as well
					if (sCascade == CASCADE1)
					{
						g_sUidSize = ISO14443A_UID_DOUBLE;
					}
					else if (sCascade == CASCADE2)
					{
						g_sUidSize = ISO14443A_UID_TRIPLE;
					}
				}
			}

			// Store the partial UID into the global UID buffer
			Iso14443a_StoreUid(sCascade,&g_pui8PartialUid[0]);

			// Clear partial UID buffer in order to handle future collisions
			for (ui8LoopCount = 0; ui8LoopCount < 5; ui8LoopCount++)
			{
				g_pui8PartialUid[ui8LoopCount] = 0x00;
			}

			// Set status to NO_COLLISION and exit function
			sStatus = NO_COLLISION;
			return sStatus;
		}
		else
		{
			sStatus = COLLISION_ERROR;
		}
	}
	else if (sStatus == COLLISION)
	{
		// If a collision occurred, check the Recursion Counter and then call Anticollision Loop function again if the condition is met
		if (g_ui8RecursionCount < g_ui8MaxRecurviseCalls)
		{
			g_ui8RecursionCount++;	// Increment Recursion Counter
			sStatus = Iso14443a_AnticollisionLoop(sCascade);	// Recursive call of Anticollision Loop
		}
		else
		{
			sStatus = COLLISION_ERROR;
		}
	}
	else
	{
		// For all other statuses, return the status that was received
	}

	return sStatus;
}


//===============================================================
//
// Iso14443a_PollingCommand - Issue the polling command for
// ISO14443A compliant tags.
//
// \param ui8Command is the polling command to be issued.
//
// This function sends the Polling command based on the inputted
// command (either REQA or WUPA).
//
// \return ui8Status returns whether or not an ISO14443A
// compliant tag has responded to the Polling command.
//
//===============================================================

u08_t Iso14443a_PollingCommand(u08_t ui8Command)
{
	u08_t ui8Offset = 0;
	u08_t ui8Status = STATUS_FAIL;

	if (Trf797xGetIsoControlValue() != 0x88)
	{
		// Trf797x has not been properly configured for ISO14443A
		Trf797xWriteIsoControl(0x88);				// Configure the TRF797x for ISO14443A @ 106kbps and Receive without CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();							// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;				// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x90;				// Send without CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;				// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;				// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x0F;				// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = ui8Command;		// Send the polling command from function input - either REQA (0x26) or WUPA (0x52)

    Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the ISO14443A Polling Command

	Trf797xIrqWaitTimeout(3,30);		// 3 millisecond TX timeout, 10 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	//Report("Querying Iso14443a_PollingCommand status\n\r");
	//StatusPrint(g_sTrfStatus);

	if (g_sTrfStatus == RX_COMPLETE)	// Tag detected - could be either a single or collided tag
	{
		ui8Status = STATUS_SUCCESS;
	}
	else if (g_sTrfStatus == PROTOCOL_ERROR)
	{
		ui8Status = STATUS_SUCCESS;		// "A PCD detecting a collision in any bit of (b16 to b1) shall commence with the first step of the anticollision loop."
	}

	return ui8Status;
}


//===============================================================
//
// Iso14443a_AnticollisionCommand - Issue Anticollsion command
// for ISO14443A compliant tags.
//
// \param sCascade is the current anticollision cascade
// \param ui8NVB is the NVB value for the UID bytes and bits
// \param pui8UID is the location of the UID bytes/bits to send.
//
// This function sends the Anticollision command based on the
// current cascade level.
//
// \return sStatus returns a status based on what tag response is
// received following the Anticollision command.
//
//===============================================================

tCollisionStatus Iso14443a_AnticollisionCommand(tISO14443A_UidStatus sCascade, u08_t ui8NVB, u08_t * pui8UID)
{
	u08_t ui8Offset = 0;
	u08_t ui8LoopCount = 0;
	u08_t ui8UidLength = 0;
	u08_t ui8RxLength = 0;
	u08_t ui8Select = SEL_CASCADE1;
	tCollisionStatus sStatus = NO_COLLISION;

	if (sCascade == CASCADE1)
	{
		ui8Select = SEL_CASCADE1;
	}
	else if (sCascade == CASCADE2)
	{
		ui8Select = SEL_CASCADE2;
	}
	else if (sCascade == CASCADE3)
	{
		ui8Select = SEL_CASCADE3;
	}
	else
	{
		return sStatus = COLLISION_ERROR;
	}

	if (Trf797xGetIsoControlValue() != 0x88)
	{
		// Trf797x has not been properly configured for ISO14443A with no RX CRC
		Trf797xWriteIsoControl(0x88);	// Configure the TRF797x for ISO14443A @ 106kbps and Receive without CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();				// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x90;		// Transmit without CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if((ui8NVB & 0x07) != 0x00)				// Length of packet in bytes - lower nibble and broken bits of transmit byte length
	{
		g_ui8TrfBuffer[ui8Offset++] = (ui8NVB & 0xF0) | (((ui8NVB & 0x07) << 1) + 1); 	// Set the number of broken bits, last bit is 1 means broken byte
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = ui8NVB & 0xF0;	// No broken bits
	}
	g_ui8TrfBuffer[ui8Offset++] = ui8Select;			// Select Command; can be 0x93, 0x95 or 0x97
	g_ui8TrfBuffer[ui8Offset++] = ui8NVB;				// Number of valid bits

	ui8UidLength = (ui8NVB >> 4) - 2;
	if ((ui8NVB & 0x0F) != 0x00)
	{
		ui8UidLength++;
	}

	for (ui8LoopCount = 0; ui8LoopCount < ui8UidLength; ui8LoopCount++)
	{
		g_ui8TrfBuffer[ui8Offset++] = pui8UID[ui8LoopCount];	// UID Bytes
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Select Command

    Trf797xIrqWaitTimeout(25,50);							// 5 millisecond TX timeout, 15 millisecond RX timeout

    g_sTrfStatus = Trf797xGetTrfStatus();

    if (g_sTrfStatus == RX_COMPLETE)
    {
    	ui8RxLength = Trf797xGetRxBytesReceived();

    	if (ui8RxLength > 1)
    	{
			sStatus = NO_COLLISION;
    	}
    	else
    	{
    		sStatus = COLLISION_ERROR;
    	}
    }
    else if (g_sTrfStatus == PROTOCOL_ERROR)
    {
    	sStatus = COLLISION;
    }
    else if (g_sTrfStatus == NO_RESPONSE_RECEIVED)
    {
    	sStatus = NO_RESPONSE;
    }
    else
    {
    	// Do nothing
    	sStatus = COLLISION_ERROR;
    }

    return sStatus;
}


//===============================================================
//
// Iso14443a_SelectCommand - Issue Select command for ISO14443A
// compliant tags.
//
// \param sCascade is the current anticollision cascade.
// \param pui8UID is the location of the UID bytes to send.
// \param bSendCT determines if the CT byte must be sent.
//
// This function issues the Select command based on the current
// cascade level.
//
// \return ui8Status returns whether or not an ISO14443A
// compliant tag has responded to the Select command.
//
//===============================================================

u08_t Iso14443a_SelectCommand(tISO14443A_UidStatus sCascade, u08_t * pui8UID, bool bSendCT)
{
	u08_t ui8Offset = 0;
	u08_t ui8Status = STATUS_FAIL;
	u08_t ui8Select = SEL_CASCADE1;

	// Sending select command and will receive a SAK response which has a CRC.
	if (Trf797xGetIsoControlValue() != 0x08)
	{
		// Trf797x has not been properly configured for ISO14443A with RX CRC
		Trf797xWriteIsoControl(0x08);			// Configure the TRF797x for ISO14443A @ 106kbps and Receive with CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(10);
	}

	if (sCascade == CASCADE1)
	{
		ui8Select = SEL_CASCADE1;
	}
	else if (sCascade == CASCADE2)
	{
		ui8Select = SEL_CASCADE2;
	}
	else if (sCascade == CASCADE3)
	{
		ui8Select = SEL_CASCADE3;
	}
	else
	{
		return ui8Status = STATUS_FAIL;
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;				// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;				// Transmit with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;				// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;				// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x70;				// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = ui8Select;		// Select Command; can be 0x93, 0x95 or 0x97
	g_ui8TrfBuffer[ui8Offset++] = NVB_FULL;			// Number of valid bits
	if (bSendCT == true)
	{
		g_ui8TrfBuffer[ui8Offset++] = 0x88;			// CT
		g_ui8TrfBuffer[ui8Offset++] = *pui8UID;		// UID Bytes
		g_ui8TrfBuffer[ui8Offset++] = *(pui8UID + 1);
		g_ui8TrfBuffer[ui8Offset++] = *(pui8UID + 2);
		g_ui8TrfBuffer[ui8Offset++] = ( 0x88 ^ *pui8UID ^ *(pui8UID + 1) ^ *(pui8UID + 2) );	// Calculate BCC Byte
	}
	else
	{
		g_ui8TrfBuffer[ui8Offset++] = *pui8UID;		// UID Bytes
		g_ui8TrfBuffer[ui8Offset++] = *(pui8UID + 1);
		g_ui8TrfBuffer[ui8Offset++] = *(pui8UID + 2);
		g_ui8TrfBuffer[ui8Offset++] = *(pui8UID + 3);
		g_ui8TrfBuffer[ui8Offset++] = ( *pui8UID ^ *(pui8UID + 1) ^ *(pui8UID + 2) ^ *(pui8UID + 3) );	// Calculate BCC Byte
	}

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);	// Issue the Select Command

	Trf797xIrqWaitTimeout(20,80);					// 5 millisecond TX timeout, 15 millisecond RX timeout

    g_sTrfStatus = Trf797xGetTrfStatus();
    //Report("Iso14443a_SelectCommand trfStatus: ");
    //StatusPrint(g_sTrfStatus);

    if (g_sTrfStatus == RX_COMPLETE)
    {
    	ui8Status = STATUS_SUCCESS;

   		g_ui8Iso14443aSAK = g_ui8TrfBuffer[0];
    }
    else
    {
    	// Do nothing
    }

    return ui8Status;
}


//===============================================================
//
// Iso14443a_Halt - Issue the Halt command to the currently
// selected ISO14443A compliant tag.
//
// This function sends the Polling command based on the inputted
// command (either REQA or WUPA).
//
// \return ui8Status returns STATUS_FAIL if the tag erroneously
// responded to the Halt command.
//
//===============================================================

u08_t Iso14443a_Halt(void)
{
	u08_t ui8Offset = 0;

	if (Trf797xGetIsoControlValue() != 0x88)
	{
		// Trf797x has not been properly configured for ISO14443A
		Trf797xWriteIsoControl(0x88);			// Configure the TRF797x for ISO14443A @ 106kbps and Receive without CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x90;		// Send without CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x50;		// Halt Command
	g_ui8TrfBuffer[ui8Offset++] = 0x00;

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Halt Command

	Trf797xIrqWaitTimeout(3,10);	// 3 millisecond TX timeout, 10 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if (g_sTrfStatus != NO_RESPONSE_RECEIVED)	// If PICC gives a response to the command, this means the Halt command failed or had an error
	{
		g_sTrfStatus = PROTOCOL_ERROR;
		Trf797xSetTrfStatus(g_sTrfStatus);
#ifdef ENABLE_HOST
		Report("Halt command error \n \r");
#endif
		return STATUS_FAIL;
	}
	else
	{
		return STATUS_SUCCESS;
	}
}


//===================================================================
//
// Iso14443a_Rats - Issue the RATS command to the currently selected
// ISO14443A compliant tag.
//
// This function sends the RATS command to activate an ISO14443A
// compliant tag for data exchange.
//
// \return ui8Status returns whether or not the selected ISO14443A
// compliant tag responded to the RATS command.
//
//===================================================================

u08_t Iso14443a_Rats(void)
{
	u08_t ui8Offset = 0;
	u08_t ui8RxLength = 0;
	u08_t ui8Status = STATUS_FAIL;
#ifdef ENABLE_HOST
	u08_t ui8LoopCount = 0;
#endif

	if (Trf797xGetIsoControlValue() != 0x08)
	{
		// Trf797x has not been properly configured for ISO14443A
		Trf797xWriteIsoControl(0x08);			// Configure the TRF797x for ISO14443A @ 106kbps and Receive with CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	// Buffer setup for FIFO writing
	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = RATS_CMD;		//RATS Command
	g_ui8TrfBuffer[ui8Offset++] = RATS_PARAM;	//RATS Parameters: 128 byte max receive and CID = 0

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the RATS command

	Trf797xIrqWaitTimeout(3,10);	// 3 millisecond TX timeout, 10 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	// If data received
	if(g_sTrfStatus == RX_COMPLETE)
	{
		ui8RxLength = Trf797xGetRxBytesReceived();

		if (g_ui8TrfBuffer[0] == ui8RxLength)
		{
			ui8Status = STATUS_SUCCESS;

			ui8Offset = 0;

			// Print out TL
#ifdef ENABLE_HOST
			Report("ISO14443A ATS Response - TL: ");
			UartPutByte(g_ui8TrfBuffer[ui8Offset++]);
			UartPutCrlf();

			// If TL is greater than 1, at minimum Format Byte T0 will be present
			if (g_ui8TrfBuffer[0] > 1)
			{
				// Print out the value of T0
				Report("ISO14443A ATS Response - T0: ");
				UartPutByte(g_ui8TrfBuffer[ui8Offset++]);
				UartPutCrlf();
				if (g_ui8TrfBuffer[1] & 0x10)
				{
					g_ui8AtsSupportedBitrates = g_ui8TrfBuffer[ui8Offset++];

					// TA(1) has been received, print it out
					Report("ISO14443A ATS Response - TA(1): ");
					UartPutByte(g_ui8AtsSupportedBitrates);
					UartPutCrlf();
				}
				if (g_ui8TrfBuffer[1] & 0x20)
				{
					// TB(1) has been received, print it out
					Report("ISO14443A ATS Response - TB(1): ");
					UartPutByte(g_ui8TrfBuffer[ui8Offset++]);
					UartPutCrlf();
				}
				if (g_ui8TrfBuffer[1] & 0x40)
				{
					// TC(1) has been received, print it out
					Report("ISO14443A ATS Response - TC(1): ");
					UartPutByte(g_ui8TrfBuffer[ui8Offset++]);
					UartPutCrlf();
				}
				if (ui8RxLength > ui8Offset)
				{
					// Historical Bytes have been received, print out all of them
					Report("ISO14443A ATS Response - Historical Bytes: ");
					for (ui8LoopCount = ui8Offset; ui8LoopCount < ui8RxLength; ui8LoopCount++)
					{
						UartPutByte(g_ui8TrfBuffer[ui8LoopCount]);
					}
					UartPutCrlf();
				}
			}
			UartPutCrlf();
#endif
		}
		else
		{
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		ui8Status = STATUS_FAIL;
	}
	return ui8Status;
}



//===================================================================
//
// Iso14443a_Pps - Issue the PPS command to the currently selected
// ISO14443A compliant tag.
//
// This function sends the PPS command to modify the over the air
// data rate of the selected ISO14443A compliant tag.
//
// \return ui8Status returns whether or not the selected ISO14443A
// compliant tag responded to the PPS command.
//
//===================================================================

u08_t Iso14443a_Pps(void)
{
	u08_t ui8Offset = 0;
	u08_t ui8Status = STATUS_FAIL;
	u08_t ui8PPSBitrate;

	// Check if PPS is supported based on last received ATS reply
	if ((g_ui8AtsSupportedBitrates == 0x00) || (g_ui8AtsSupportedBitrates == 0x80))
	{
#ifdef ENABLE_HOST
		Report("Tag does not support data rates above 106kbps. No PPS issued.");
		UartPutCrlf();
		UartPutCrlf();
#endif

		return ui8Status = STATUS_SUCCESS;
	}

	if (Trf797xGetIsoControlValue() != 0x08)
	{
		// Trf797x has not been properly configured for ISO14443A
		Trf797xWriteIsoControl(0x08);			// Configure the TRF797x for ISO14443A @ 106kbps and Receive with CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	ui8PPSBitrate = PPS1_106; 				// Set the PPS bit rate to 106kbps for best range performance
											// It is recommended to keep the data rate low to get better transmission ranges

	// Buffer setup for FIFO writing
	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x30;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = PPSS;		// PPS Command
	g_ui8TrfBuffer[ui8Offset++] = PPS0;
	g_ui8TrfBuffer[ui8Offset++] = ui8PPSBitrate;	// Send PPS Bit Rate

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the PPS Command

	Trf797xIrqWaitTimeout(3,10);		// 3 millisecond TX timeout, 10 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	// If data received
	if(g_sTrfStatus == RX_COMPLETE)
	{
		// Check PPS response
		if (g_ui8TrfBuffer[0] == PPSS)
		{
			ui8Status = STATUS_SUCCESS;

			// Execute Bitrate Change
			McuDelayMillisecond(1);

			if (ui8PPSBitrate == PPS1_106)
			{
				Trf797xWriteIsoControl(0x08);	// Configure the TRF797x for ISO14443A @ 106kbps and Receive with CRC

#ifdef ENABLE_HOST
				Report("PPS Successful: ISO14443A Bit Rate Set to 106kpbs");
				UartPutCrlf();
				UartPutCrlf();
#endif
			}
			else if (ui8PPSBitrate == PPS1_212)
			{
				Trf797xWriteIsoControl(0x09);	// Configure the TRF797x for ISO14443A @ 212kbps and Receive with CRC

#ifdef ENABLE_HOST
				Report("PPS Successful: ISO14443A Bit Rate Set to 212kpbs");
				UartPutCrlf();
				UartPutCrlf();
#endif
			}
			else if (ui8PPSBitrate == PPS1_424)
			{
				Trf797xWriteIsoControl(0x0A);	// Configure the TRF797x for ISO14443A @ 424kbps and Receive with CRC

#ifdef ENABLE_HOST
				Report("PPS Successful: ISO14443A Bit Rate Set to 424kpbs");
				UartPutCrlf();
				UartPutCrlf();
#endif
			}
			else if (ui8PPSBitrate == PPS1_848)
			{
				Trf797xWriteIsoControl(0x0B);	// Configure the TRF797x for ISO14443A @ 848kbps and Receive with CRC

#ifdef ENABLE_HOST
				Report("PPS Successful: ISO14443A Bit Rate Set to 848kpbs");
				UartPutCrlf();
				UartPutCrlf();
#endif
			}
			else
			{
				// Do Nothing
			}
			McuDelayMillisecond(6);
		}
		else
		{
			ui8Status = STATUS_FAIL;

#ifdef ENABLE_HOST
			Report("Error: PPS Reply Does Not Match Transmitted Frame");
			UartPutCrlf();
#endif
		}
	}
	else
	{
		ui8Status = STATUS_FAIL;

#ifdef ENABLE_HOST
		Report("Error: PPS Reply Not Received");
		UartPutCrlf();
#endif
	}

	return ui8Status;
}



//===============================================================
//
// Iso14443a_StoreUid - Store the received UID bytes into the
// global g_pui8CompleteUid buffer.
//
// \param sCascade is the current anticollision cascade.
// \param pui8UID is the location of the UID bytes to store.
//
// This function stores the received UID bytes into the global
// UID buffer. It will also parse out the CT byte and set a flag
// to indicate if the CT byte was present.
//
// \return bSendCT returns whether or not the next transmission
// will require the CT byte to be sent in addition to the UID
// bytes.
//
//===============================================================

bool Iso14443a_StoreUid(tISO14443A_UidStatus sCascade, u08_t * pui8UID)
{
	bool bSendCT = false;
	u08_t ui8Offset = 0;

	if ((g_sUidSize == ISO14443A_UID_SINGLE) && (sCascade == CASCADE1))
	{
		// UID has no CT, so store all bytes normally.
		ui8Offset = 0;
		bSendCT = false;
	}
	else if ((g_sUidSize == ISO14443A_UID_DOUBLE) && (sCascade == CASCADE2))
	{
		// UID has no CT, so store all bytes normally.
		ui8Offset = 0;
		bSendCT = false;
	}
	else if ((g_sUidSize == ISO14443A_UID_TRIPLE) && (sCascade == CASCADE3))
	{
		// UID has no CT, so store all bytes normally.
		ui8Offset = 0;
		bSendCT = false;
	}
	else
	{
		if (pui8UID[0] == 0x88)
		{
			// UID has a CT, set bool to return that a CT must be sent
			ui8Offset = 1;		// Set offset to account for the location of the CT byte
			bSendCT = true;		// Set variable to tell Select Command to include a CT in addition to UID bytes
		}
		else
		{
			// UID has no CT, so store all bytes normally.
			ui8Offset = 0;
			bSendCT = false;
		}
	}

	// Store UID based on the current Cascade level
	if (sCascade == CASCADE1)
	{
		g_pui8CompleteUid[0] = pui8UID[0+ui8Offset];
		g_pui8CompleteUid[1] = pui8UID[1+ui8Offset];
		g_pui8CompleteUid[2] = pui8UID[2+ui8Offset];
		g_pui8CompleteUid[3] = pui8UID[3+ui8Offset];	// BCC Byte or last byte of UID
		g_ui8UidPos = 0;	// Update the UID Position indicator to the first byte of the newly stored UID for when the Select command is issued
	}
	else if (sCascade == CASCADE2)
	{
		g_pui8CompleteUid[3] = pui8UID[0+ui8Offset];	// Override the BCC from prior Cascade as it is no longer needed
		g_pui8CompleteUid[4] = pui8UID[1+ui8Offset];
		g_pui8CompleteUid[5] = pui8UID[2+ui8Offset];
		g_pui8CompleteUid[6] = pui8UID[3+ui8Offset];	// BCC Byte or last byte of UID
		g_ui8UidPos = 3;	// Update the UID Position indicator to the first byte of the newly stored UID for when the Select command is issued
	}
	else if (sCascade == CASCADE3)
	{
		g_pui8CompleteUid[6] = pui8UID[0];	// Override the BCC from prior Cascade as it is no longer needed
		g_pui8CompleteUid[7] = pui8UID[1];
		g_pui8CompleteUid[8] = pui8UID[2];
		g_pui8CompleteUid[9] = pui8UID[3];
		bSendCT = false; 	// Ensure no accidental sending of the CT occurs incase uid6 for a Triple Size UID = 0x88 (which is permitted per ISO14443-3 specifications)
		g_ui8UidPos = 6;	// Update the UID Position indicator to the first byte of the newly stored UID for when the Select command is issued
	}

	return bSendCT;
}

//===============================================================
//
// Iso14443a_Type2_Read4Blocks - Reads out four blocks of data
// from NFC Type 2 Tag Platforms.
//
// \param ui8StartBlock is the block number to start reading the
// tag data from.
//
// This function will issue a Read Block command with the
// provided starting block number. The Read Block command
// automatically will read out four blocks of data from the Type
// 2 Tag.
//
// \return ui8Status returns whether or not the tag data was
// successfully read.
//
//===============================================================

u08_t Iso14443a_Type2_Read4Blocks(u08_t ui8StartBlock)
{
	u08_t ui8Offset = 0;
	u08_t ui8Status = STATUS_FAIL;
#ifdef ENABLE_HOST
	u08_t	ui8LoopCount1 = 1;
	u08_t	ui8LoopCount2 = 0;
#endif

	if (Trf797xGetIsoControlValue() != 0x08)
	{
		// Trf797x has not been properly configured for ISO14443A
		Trf797xWriteIsoControl(0x08);			// Configure the TRF797x for ISO14443A @ 106kbps and Receive with CRC
	}
	if (Trf797xCheckRfField() == false)
	{
		// RF field is not enabled, VICC will not receive the command
		Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

		// When a PICC is exposed to an unmodulated operating field
		// it shall be able to accept a quest within 5 ms.
		// PCDs should periodically present an unmodulated field of at least
		// 5.1 ms duration. (ISO14443-3)
		McuDelayMillisecond(6);
	}

	g_ui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_ui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_ui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_ui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_ui8TrfBuffer[ui8Offset++] = 0x30;				// Read Command
	g_ui8TrfBuffer[ui8Offset++] = ui8StartBlock;	// Starting from Block # (called Bno)

	Trf797xRawWrite(&g_ui8TrfBuffer[0], ui8Offset);		// Issue the Type 2 Read Command

	Trf797xIrqWaitTimeout(5,30);		// 5 millisecond TX timeout, 30 millisecond RX timeout

	g_sTrfStatus = Trf797xGetTrfStatus();

	if(g_sTrfStatus == RX_COMPLETE)		// If block data has been received
	{
		ui8Status = STATUS_SUCCESS;		// Mark tag has been successfully read

#ifdef ENABLE_HOST
		for(ui8LoopCount2 = 0; ui8LoopCount2 < 4; ui8LoopCount2++)
		{
			Report("NFC Type 2 Block ");
			UartPutByte(ui8StartBlock++);
			Report(":  [");
			for(ui8LoopCount1 = (ui8LoopCount2*4); ui8LoopCount1 < 4+(ui8LoopCount2*4); ui8LoopCount1++)
			{
				UartPutByte(g_ui8TrfBuffer[ui8LoopCount1]);		// Print out the received data
			}
			UartPutChar(']');
			UartPutCrlf();
		}
#endif
	}
	else
	{
		// Otherwise return a fail
		ui8Status = STATUS_FAIL;
	}

	return ui8Status;
}

//===============================================================
//
// Iso14443a_Get_Type4ACompliance - Fetches g_bType4ACompliant value
//
// This function allows for higher layers to fetch the current
// Type 4A NDEF compliance information.
//
// \return g_bType4ACompliant returns the current compliance for
// Type 4A NDEF.
//
//===============================================================

bool Iso14443a_Get_Type4ACompliance(void)
{
	return g_bType4ACompliant;
}

//===============================================================
//
// Iso14443a_Set_RecursionCount - Sets the g_ui8RecursionCount
// variable
//
// \param
//
// This function allows for higher layers to adjust the global
// recursion count. Useful for resetting it prior to running
// anticollision routines.
//
// \return None.
//
//===============================================================

void Iso14443a_Set_RecursionCount(u08_t ui8RecursionCount)
{
	g_ui8RecursionCount = ui8RecursionCount;
}

//===============================================================
//
// Iso14443a_Get_Uid - Fetches the ISO14443A Tag UID.
//
// This function allows for higher layers to fetch the tag UID of
// an ISO14443A tag. In the current implementation, the UID
// stored is from the most recent tag which finished the
// anticollision procedure.
//
// \return g_pui8CompleteUid returns the currently stored UID.
//
//===============================================================

u08_t * Iso14443a_Get_Uid(void)
{
	return g_pui8CompleteUid;
}

//===============================================================
//
// Iso14443a_Get_UidSize - Fetches the UID size of the most
// recently read ISO14443A tag.
//
// This function allows for higher layers to fetch the size of
// the current UID for an ISO14443A tag.
//
// \return g_sUidSize returns the current UID size.
//
//===============================================================

tISO14443A_UidSize Iso14443a_Get_UidSize(void)
{
	return g_sUidSize;
}

//===============================================================
//
// Nfc_Iso14443a_Type4NdefApp - Customizeable application to read
// NDEF data from a Type 4A NDEF Formatted tag.
//
// Tags which are Type 4 compliant are activated via RATS.
// If the tag contains an NDEF message, then the NDEF data is
// read from it.
//
// \return None.
//
//===============================================================

void Nfc_Iso14443a_Type4NdefApp(void)
{
	if (Iso14443a_Rats() == STATUS_SUCCESS)
	{
		Ndef_SetBlockNumberBit(0);

		if (NDEF_ApplicationSelect() == STATUS_SUCCESS) // Selects NDEF Application
		{
			Report("Payload Received\n\r");
		}
		else
		{
			Report("ISO14443a: Select AID failure\n\r");
		}
	}
	else
	{
		Report("ISO14443a: RATS Failure\n\r");
	}
}

void ISO14443aFindTag(void)
{
	Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

	Trf797xWriteInitiatorSetup(0x88);		// Configure the TRF797x for ISO14443A @ 106kbps and Receive no CRC

	IRQ_CLR;									// PORT2 interrupt flag clear
	IRQ_ON;

	// When a PICC is exposed to an unmodulated operating field
	// it shall be able to accept a quest within 5 ms.
	// PCDs should periodically present an unmodulated field of at least
	// 5.1 ms duration. (ISO14443-3)
	McuDelayMillisecond(6);

	Iso14443a_Set_RecursionCount(0); 		// Clear the recursion count for anticollision loops

	if (Iso14443a_TagSelection(REQA) == STATUS_SUCCESS)	//  Do a complete anticollision sequence as described in ISO14443-3 standard for type A
	{
		if (Iso14443a_Get_Type4ACompliance() == true)
		{
			//Report("Type 4A Tag\n\r");
			Nfc_Iso14443a_Type4NdefApp();	// For a Type 4A compliant tag, the tag is put into Layer 4, and in order to attempt to read/write NDEF contents
		}
	}

	Trf797xTurnRfOff();
}
