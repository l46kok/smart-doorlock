/*
 * {spi_for_trf.c}
 *
 * {SPI Interface Functions}
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
#include "spi.h"
#include "spi_for_trf.h"
#include "gpio_if.h"
#include "trf797x.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "interrupt.h"
#include "pinmux.h"
#include "uart.h"
#include "uart_if.h"
#include "types.h"
#include "trf7970BoosterPack.h"



#define ASSERT_CS()          (SPI_CS_ON)
#define DEASSERT_CS()        (SPI_CS_OFF)

typedef unsigned int Fd_t;

//===============================================================

u08_t    temp = 0;

extern u08_t direct_mode;

//===============================================================

void SpiStartCondition(void);
void SpiStopCondition(void);
void SpiUsciSet(void);
void SpiUsciDisable(void);

//===============================================================
// NAME: void SpiDirectCommand (u08_t *pbuf)
//
// BRIEF: Is used in SPI mode to transmit a Direct Command to
// reader chip.
//
// INPUTS:
//    Parameters:
//        u08_t        *pbuf        Direct Command
//
// OUTPUTS:
//
// PROCESS:    [1] transmit Direct Command
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiDirectCommand(u08_t *pbuf)
{
  unsigned char ucDummy;
	// set Address/Command Word Bit Distribution to command
  *pbuf = (0x80 | *pbuf);           // command
  *pbuf = (0x9f & *pbuf);            // command code
  SPI_CS_OFF;
  SPITransfer(GSPI_BASE, pbuf, &ucDummy, 1, SPI_CS_ENABLE);
  SPI_CS_ON;
}


//===============================================================
// NAME: void SpiDirectMode (void)
//
// BRIEF: Is used in SPI mode to start Direct Mode.
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:    [1] start Direct Mode
//
// NOTE: No stop condition
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiDirectMode(void)
{
  u08_t command [2];
  
  command[0] = TRF797x_STATUS_CONTROL;
  command[1] = TRF797x_STATUS_CONTROL;
  // read byte to command[1]
  SPI_CS_OFF;
  SPITransfer(GSPI_BASE, 0, &command[1], 1, SPI_CS_ENABLE);


  command[1] |= 0x60;                        // RF on and BIT 6 in Chip Status Control Register set
  // write command[0] byte
  SPITransfer(GSPI_BASE, command, 0, 1, SPI_CS_ENABLE);
  SpiWriteSingle(command, 2);
  SPI_CS_ON;
}

//===============================================================
// NAME: void SpiRawWrite (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write direct to the reader chip.
//
// INPUTS:
//    Parameters:
//        u08_t        *pbuf        raw data
//        u08_t        length        number of data bytes
//
// OUTPUTS:
//
// PROCESS:    [1] send raw data to reader chip
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiRawWrite(u08_t *pbuf, u08_t length)
{
  SPI_CS_OFF;
  SPITransfer(GSPI_BASE, pbuf, 0, length, SPI_CS_ENABLE);
  SPI_CS_ON;
}

//===============================================================
// NAME: void SpiReadCont (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to read a specified number of
// reader chip registers from a specified address upwards.
//
// INPUTS:
//    Parameters:
//        u08_t        *pbuf        address of first register
//        u08_t        length        number of registers
//
// OUTPUTS:
//
// PROCESS:    [1] read registers
//            [2] write contents to *pbuf
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiReadCont(u08_t *pbuf, u08_t length)
{
  
  // Address/Command Word Bit Distribution
  *pbuf = (0x60 | *pbuf);                     // address, read, continuous
  *pbuf = (0x7f &*pbuf);                        // register address

  // write command address/word
  SPI_CS_OFF;
  SPITransfer(GSPI_BASE, pbuf, 0, 1, SPI_CS_ENABLE);

  // read length bytes into pbuf[length]
  SPITransfer(GSPI_BASE, 0, pbuf, length, SPI_CS_ENABLE);
  SPI_CS_ON;
  
 }

//===============================================================
// NAME: void SpiReadSingle (u08_t *pbuf, u08_t number)
//
// BRIEF: Is used in SPI mode to read specified reader chip
// registers.
//
// INPUTS:
//    Parameters:
//        u08_t        *pbuf        addresses of the registers
//        u08_t        number        number of the registers
//
// OUTPUTS:
//
// PROCESS:    [1] read registers
//            [2] write contents to *pbuf
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiReadSingle(u08_t *pbuf, u08_t number)
{

  SPI_CS_OFF;

  while(number > 0)
  {
    // Address/Command Word Bit Distribution
    *pbuf = (0x40 | *pbuf);             // address, read, single
    *pbuf = (0x5f & *pbuf);                // register address

    
    // write address/command byte into pbuf
    SPITransfer(GSPI_BASE, pbuf, 0, 1, SPI_CS_ENABLE);

    // read single byte into pbuf
    SPITransfer(GSPI_BASE, 0, pbuf, 1, SPI_CS_ENABLE);
    
    pbuf++;
    number--;
    
  }
  SPI_CS_ON;


}






//===============================================================
// NAME: void SpiWriteCont (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write to a specific number of
// reader chip registers from a specific address upwards.
//
// INPUTS:
//    u08_t    *pbuf    address of first register followed by the
//                    contents to write
//    u08_t    length    number of registers + 1
//
// OUTPUTS:
//
// PROCESS:    [1] write to the registers
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiWriteCont(u08_t *pbuf, u08_t length)
{


  SPI_CS_OFF;

  // Address/Command Word Bit Distribution
  *pbuf = (0x20 | *pbuf);                 // address, write, continuous
  *pbuf = (0x3f &*pbuf);                    // register address

  // write command address/word + number of registers
  SPITransfer(GSPI_BASE, pbuf, 0, length, SPI_CS_ENABLE);

  SPI_CS_ON;

}

//===============================================================
// NAME: void SpiWriteSingle (u08_t *pbuf, u08_t length)
//
// BRIEF: Is used in SPI mode to write to a specified reader chip
// registers.
//
// INPUTS:
//    u08_t    *pbuf    addresses of the registers followed by the
//                    contends to write
//    u08_t    length    number of registers * 2
//
// OUTPUTS:
//
// PROCESS:    [1] write to the registers
//
// CHANGE:
// DATE          WHO    DETAIL
// 24Nov2010    RP    Original Code
// 07Dec2010    RP    integrated wait while busy loops
//===============================================================

void
SpiWriteSingle(u08_t *pbuf, u08_t length)
{
  
  SPI_CS_OFF;
  while(length > 0)
  {
    // Address/Command Word Bit Distribution
    // address, write, single (fist 3 bits = 0)
    *pbuf = (0x1f &*pbuf);                // register address
	SPITransfer(GSPI_BASE, pbuf, 0, 2, SPI_CS_ENABLE);
    pbuf=pbuf + 2;
    length=length - 2;
  }
  SPI_CS_ON;

}





