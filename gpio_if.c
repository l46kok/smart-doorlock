/*
 * gpio_if.c
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */



#include <stdio.h>
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "pin.h"
#include "gpio.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio_if.h"

//****************************************************************************
//                      GLOBAL VARIABLES
//****************************************************************************


static unsigned long ulReg[]=
{
    GPIOA0_BASE,
    GPIOA1_BASE,
    GPIOA2_BASE,
    GPIOA3_BASE
};

//*****************************************************************************
// Variables to store TIMER Port,Pin values
//*****************************************************************************

#define PIN_LCD_RS 0 //4 RS (CS) H / L H=Data, L=Command
#define PIN_LCD_RW 3 //5 R/W (SID) H / L H=Read, L=Write
#define PIN_LCD_E 4 //6 E (SCLK) H Enable (falling edge)
#define PIN_LCD_D0 5 //7 D0 (SOD) H / L Display Data, LSB
#define PIN_LCD_D1 6 //8 D1 H / L Display Data
#define PIN_LCD_D2 7 //9 D2 H / L Display Data
#define PIN_LCD_D3 8 //10 D3 H / L Display Data
#define PIN_LCD_D4 9 //11 D4 (D0) H / L Display Data
#define PIN_LCD_D5 10 //12 D5 (D1) H / L Display Data
#define PIN_LCD_D6 11 //13 D6 (D2) H / L Display Data
#define PIN_LCD_D7 12 //14 D7 (D3) H / L Display Data, MSB


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

void
GPIO_IF_Set(unsigned int gpioNum, unsigned int state) {
	unsigned int portNum = 0;
	unsigned char pinNum;

	GPIO_IF_GetPortNPin(gpioNum,
	                        &portNum,
	                        &pinNum);
	GPIO_IF_SetVal(gpioNum, portNum, pinNum, state);
}

void
GPIO_IF_Toggle(unsigned int gpioNum) {
	unsigned int portNum = 0;
	unsigned char pinNum;
	unsigned int portStatus = -1;

	GPIO_IF_GetPortNPin(gpioNum,
	                        &portNum,
	                        &pinNum);

	portStatus = !GPIO_IF_Get(gpioNum, portNum, pinNum);
	GPIO_IF_SetVal(gpioNum, portNum, pinNum, portStatus);
}


//****************************************************************************
//
//! Get the port and pin of a given GPIO
//!
//! \param ucPin is the pin to be set-up as a GPIO (0:39)
//! \param puiGPIOPort is the pointer to store GPIO port address return value
//! \param pucGPIOPin is the pointer to store GPIO pin return value
//!
//! This function
//!    1. Return the GPIO port address and pin for a given external pin number
//!
//! \return None.
//
//****************************************************************************
void
GPIO_IF_GetPortNPin(unsigned char ucPin,
				unsigned int *puiGPIOPort,
					unsigned char *pucGPIOPin)
{
  //
  // Get the GPIO pin from the external Pin number
  //
  *pucGPIOPin = 1 << (ucPin % 8);

  //
  // Get the GPIO port from the external Pin number
  //
  *puiGPIOPort = (ucPin / 8);
  *puiGPIOPort = ulReg[*puiGPIOPort];
}

//****************************************************************************
//
//! Configures the GPIO selected as input to generate interrupt on activity
//!
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//! \param uiIntType is the type of the interrupt (refer gpio.h)
//! \param pfnIntHandler is the interrupt handler to register
//!
//! This function
//!    1. Sets GPIO interrupt type
//!    2. Registers Interrupt handler
//!    3. Enables Interrupt
//!
//! \return None
//
//****************************************************************************
void
GPIO_IF_ConfigureNIntEnable(unsigned int uiGPIOPort,
                                  unsigned char ucGPIOPin,
                                  unsigned int uiIntType,
                                  void (*pfnIntHandler)(void))
{
  //
  // Set GPIO interrupt type
  //
  MAP_GPIOIntTypeSet(uiGPIOPort,ucGPIOPin,uiIntType);

  //
  // Register Interrupt handler
  //

  MAP_GPIOIntRegister(uiGPIOPort,pfnIntHandler);


  //
  // Enable Interrupt
  //
  MAP_GPIOIntClear(uiGPIOPort,ucGPIOPin);
  MAP_GPIOIntEnable(uiGPIOPort,ucGPIOPin);
}

//****************************************************************************
//
//! Set a value to the specified GPIO pin
//!
//! \param ucPin is the GPIO pin to be set (0:39)
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//! \param ucGPIOValue is the value to be set
//!
//! This function
//!    1. Sets a value to the specified GPIO pin
//!
//! \return None.
//
//****************************************************************************
void
GPIO_IF_SetVal(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin,
             unsigned char ucGPIOValue)
{
  //
  // Set the corresponding bit in the bitmask
  //
  ucGPIOValue = ucGPIOValue << (ucPin % 8);

  //
  // Invoke the API to set the value
  //
  MAP_GPIOPinWrite(uiGPIOPort,ucGPIOPin,ucGPIOValue);
}

//****************************************************************************
//
//! Set a value to the specified GPIO pin
//!
//! \param ucPin is the GPIO pin to be set (0:39)
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//!
//! This function
//!    1. Gets a value of the specified GPIO pin
//!
//! \return value of the GPIO pin
//
//****************************************************************************
unsigned char
GPIO_IF_Get(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin)
{
    unsigned char ucGPIOValue;
    long lGPIOStatus;

    //
    // Invoke the API to Get the value
    //

    lGPIOStatus =  MAP_GPIOPinRead(uiGPIOPort,ucGPIOPin);

    //
    // Set the corresponding bit in the bitmask
    //
    ucGPIOValue = lGPIOStatus >> (ucPin % 8);
    return ucGPIOValue;
}



/*
void GPIO_Set(unsigned char ucGPIONum){

	 int ucGPIOPin, uiGPIOPort;
	 ucGPIOPin = 1 << (ucGPIONum % 8);
	 uiGPIOPort = ulReg[ucGPIONum / 8];

	 MAP_GPIOPinWrite(uiGPIOPort,ucGPIOPin,0xff);
}

void GPIO_Clear(unsigned char ucGPIONum){

	 int ucGPIOPin, uiGPIOPort;
	 ucGPIOPin = 1 << (ucGPIONum % 8);
	 uiGPIOPort = ulReg[ucGPIONum / 8];

	 MAP_GPIOPinWrite(uiGPIOPort,ucGPIOPin,0x0);
}
*/

void GPIOIntInit(unsigned long ulPort, unsigned char ucPin, unsigned long ulInterrupt, void (*pfnHandler)(void), unsigned long ulIntType, unsigned char ucPriority) {

	  IntRegister(ulInterrupt, pfnHandler);
	  IntPrioritySet(ulInterrupt, INT_PRIORITY_LVL_1);


	  GPIOIntTypeSet(ulPort,ucPin,ulIntType);
	  GPIOIntClear(ulPort,0xFF);
	  GPIOIntEnable(ulPort,ucPin);
	  IntEnable(ulInterrupt);
}
