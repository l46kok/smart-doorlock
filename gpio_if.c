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
	unsigned int portStatus;

	GPIO_IF_GetPortNPin(gpioNum,
	                        &portNum,
	                        &pinNum);

	portStatus = !GPIO_IF_GetVal(gpioNum, portNum, pinNum);
	GPIO_IF_SetVal(gpioNum, portNum, pinNum, portStatus);
}

unsigned char
GPIO_IF_Get(unsigned int gpioNum) {
	unsigned int portNum = 0;
	unsigned char pinNum;

	GPIO_IF_GetPortNPin(gpioNum,
	                        &portNum,
	                        &pinNum);

	return GPIO_IF_GetVal(gpioNum, portNum, pinNum);
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
static void
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
GPIO_IF_GetVal(unsigned char ucPin,
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


void GPIOIntInit(unsigned long ulPort, unsigned char ucPin, unsigned long ulInterrupt, void (*pfnHandler)(void), unsigned long ulIntType, unsigned char ucPriority) {

	  IntRegister(ulInterrupt, pfnHandler);
	  IntPrioritySet(ulInterrupt, INT_PRIORITY_LVL_1);


	  GPIOIntTypeSet(ulPort,ucPin,ulIntType);
	  GPIOIntClear(ulPort,0xFF);
	  GPIOIntEnable(ulPort,ucPin);
	  IntEnable(ulInterrupt);
}
