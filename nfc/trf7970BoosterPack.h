/*
 * {trf7970BoosterPack.h}
 *
 * {Header File}
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

#ifndef _Trf7970BOOSTERPACK_H_
#define _Trf7970BOOSTERPACK_H_

//================================================================


#include <stdint.h>
#include "types.h"


#define	_BOARD_H



/*  This macro is for use by other macros to form a fully valid C statement.  */
#define st(x)      do { x } while (__LINE__ == -1)


#define COUNT_1ms 80000
#define IRQ_CLR GPIOIntClear(GPIOA1_BASE, GPIO_PIN_4); //GPIO 7
#define IRQ_ON  GPIOIntEnable(GPIOA1_BASE,GPIO_PIN_4);
#define IRQ_OFF GPIOIntDisable(GPIOA1_BASE,GPIO_PIN_4);
#define START_COUNTER A2CounterEnable();
#define RESET_COUNTER A2CounterDisable();
#define TRF_OFF GPIO_IF_Set(11,0);
#define TRF_ON GPIO_IF_Set(11,1);
#define SPI_CS_ON MAP_SPICSEnable(GSPI_BASE);
#define SPI_CS_OFF MAP_SPICSDisable(GSPI_BASE);


void Delay(unsigned long interval);
void McuDelayMillisecond(unsigned int ui_delay_in_msec);






//===============================================================

#define TRIGGER        0                        // if TRIGGER 1 trigger-point at LED 5

//=====MCU constants=============================================


// IRQ on P2.0
// rising edge interrupt
#define IRQ_PIN             BIT7
#define IRQ_PORT            HWREG(GPIOA1_BASE + GPIO_O_GPIO_RIS)





// Hard code to only SPI mode
#define SPIMODE             1

// CSn

//-----Counter-timer constants-----------------------------------


//---------------------------------------------------------------
// Set timer capture register value based on clock source


//---------------------------------------------------------------

// 25MHz clock


//===============================================================












#endif
