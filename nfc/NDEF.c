/* {NDEF.c}
 *
 * {NDEF specific functions file}
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
*/
#include "hw_types.h"
#include "NDEF.h"
#include "timer.h"
#include "timer_if.h"
#include "trf7970.h"
#include "trf7970BoosterPack.h"
#include "hw_types.h"
#include "uart_if.h"

#include "utils.h"
#include "gpio.h"
#include "gpio_if.h"
#include "hw_memmap.h"
#include "uart_if.h"
#include "utils.h"
//===============================================================

extern u08_t	buf[140];
extern u08_t	i_reg;
extern u08_t	irq_flag;
extern u08_t	stand_alone_flag;
extern s08_t	rxtx_state;
extern u08_t 	rx_error_flag;

//===============================================================

unsigned char NDEFApplicationSelect(void)
{
	u08_t NDEF_Support = 0;

rx_error_flag = 0x00;

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0xE0;  // length
  buf[5] = 0x02;
  buf[6] = 0x00;  
  buf[7] = 0xA4;
  buf[8] = 0x04;
  buf[9] = 0x00;
  buf[10] = 0x07;  
  buf[11] = 0xD2;  
  buf[12] = 0x76;
  buf[13] = 0x00;  
  buf[14] = 0x00;  
  buf[15] = 0x85;  
  buf[16] = 0x01;
  buf[17] = 0x01;
  buf[18] = 0x00; 

  
  Trf7970RawWrite(&buf[0], 19);	//writing to FIFO
  
  //Trf7970ResetIrqStatus();
  IRQ_ON;
  
  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  // COUNT_VALUE = COUNT_1ms * 20;              /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */
  
  while(i_reg == 0x01);   // Wait for end of TX

  i_reg = 0x01;
  
  RESET_COUNTER;


  // COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */
  
  if( (buf[1] == 0x90) && (buf[2] == 0x00)){
	  NDEF_Support = 1;
  }

  RESET_COUNTER;
  
  Trf7970ResetIrqStatus();
  
  McuDelayMillisecond(1);

  return NDEF_Support;
}

void CapabilityContainerSelect(void)
{
	rx_error_flag = 0x00;

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0x80;  // length
  buf[5] = 0x03;
  buf[6] = 0x00;  
  buf[7] = 0xA4;
  buf[8] = 0x00;
  buf[9] = 0x0C;
  buf[10] = 0x02;  
  buf[11] = 0xE1;  
  buf[12] = 0x03;
  
  Trf7970RawWrite(&buf[0], 13);
  
  IRQ_CLR;											// PORT2 interrupt flag clear (inside MSP430)
  IRQ_ON;
  
  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  //COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */
  
  while(i_reg == 0x01);   // Wait for end of TX

  i_reg = 0x01;
  
  RESET_COUNTER;


  A2CounterLoad(COUNT_1ms * 20);
//  COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  START_COUNTER;                            /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */
  
  Trf7970ResetIrqStatus();
  
  //stopCounter;
  
  McuDelayMillisecond(1);
}

unsigned char ReadBinary(unsigned char Offset, unsigned char Read_Length)
{
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  unsigned char Nlen=0; 
//  extern unsigned char Tag_found;
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

  rx_error_flag = 0x00;

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0x60;  // length
  buf[5] = 0x02;
  buf[6] = 0x00;  
  buf[7] = 0xB0;
  buf[8] = 0x00;
  buf[9] = Offset;              // offset
  buf[10] = Read_Length;       // Read Length
  
  Trf7970RawWrite(&buf[0], 11);
  
  Trf7970ResetIrqStatus();
  IRQ_ON;
  
  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  //COUNT_VALUE = COUNT_1ms * 20;
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */
  
  while(i_reg == 0x01);   // Wait for end of TX
  

  i_reg = 0x01;

  RESET_COUNTER;
  

  A2CounterLoad(COUNT_1ms * 20);
//  COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */
  
  RESET_COUNTER;

  //Trf7970ResetIrqStatus();
   
  // might need test case here, like if(i_reg == 0xFF)
   Nlen = buf[2]; 
   McuDelayMillisecond(1);
   
   return Nlen;
}

unsigned char ReadBinary2(unsigned char Offset, unsigned char Read_Length)
{
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  unsigned char Nlen=0;
//  extern unsigned char Tag_found;
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

  rx_error_flag = 0x00;

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0x60;  // length
  buf[5] = 0x03;
  buf[6] = 0x00;
  buf[7] = 0xB0;
  buf[8] = 0x00;
  buf[9] = Offset;              // offset
  buf[10] = Read_Length;       // Read Length

  Trf7970RawWrite(&buf[0], 11);

  Trf7970ResetIrqStatus();
  IRQ_ON;

  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  //COUNT_VALUE = COUNT_1ms * 20;
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);   // Wait for end of TX


  i_reg = 0x01;

  RESET_COUNTER;


//  COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */

  RESET_COUNTER;

  //Trf7970ResetIrqStatus();

  // might need test case here, like if(i_reg == 0xFF)
   Nlen = buf[2];
   McuDelayMillisecond(1);

   return Nlen;
}


void SelectNDEF(void)
{
	rx_error_flag = 0x00;

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0x80;  // length
  buf[5] = 0x03;
  buf[6] = 0x00;  
  buf[7] = 0xA4;
  buf[8] = 0x00;
  buf[9] = 0x0C;
  buf[10] = 0x02;  
  buf[11] = 0xE1;  
  buf[12] = 0x04;
  
  Trf7970RawWrite(&buf[0], 13);
  
  Trf7970ResetIrqStatus();
  IRQ_ON;
  
  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  // COUNT_VALUE = COUNT_1ms * 20;
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                            /* start timer up mode */
  
  while(i_reg == 0x01);   // Wait for end of TX

  i_reg = 0x01;
  
  RESET_COUNTER;


  // COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */
  
  Trf7970ResetIrqStatus();
  
  RESET_COUNTER;
    
  McuDelayMillisecond(1);
}

void RATS(void)
{
	u08_t write[4];
	rx_error_flag = 0x00;

	write[0] = ISO_CONTROL;
	write[1] = 0x08;
	Trf7970WriteSingle(write, 2);

  buf[0] = 0x8f;
  buf[1] = 0x91;
  buf[2] = 0x3d;
  buf[3] = 0x00;
  buf[4] = 0x20;  // length
  buf[5] = 0xE0;
  buf[6] = 0x80;

  Trf7970RawWrite(&buf[0], 7);

  Trf7970ResetIrqStatus();
  IRQ_ON;

  i_reg = 0x01;
  rxtx_state = 0;                            /* the response will be stored in buf[0] upwards */


  // COUNT_VALUE = COUNT_1ms * 20;
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                            /* start timer up mode */

  while(i_reg == 0x01);   // Wait for end of TX

  i_reg = 0x01;

  RESET_COUNTER;


  // COUNT_VALUE = COUNT_1ms * 20;               /* 10ms for TIMEOUT */
  A2CounterLoad(COUNT_1ms * 20);
  START_COUNTER;                             /* start timer up mode */

  while(i_reg == 0x01);                     /* wait for RX complete */

  Trf7970ResetIrqStatus();

  RESET_COUNTER;

  McuDelayMillisecond(1);
}

