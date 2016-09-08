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
unsigned int g_uiLED1Port = 0,g_uiLED2Port = 0,g_uiLED3Port = 0;
unsigned char g_ucLED1Pin,	g_ucLED2Pin,	g_ucLED3Pin;
extern unsigned char g_heart_beat, g_state;

#define PIN_LED1 9
#define PIN_LED2 10
#define PIN_LED3 11



//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************
//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  ucPins is the bit-pack representation of 3 LEDs
//! 		LSB:GP09-GP10-GP11:MSB
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_LedConfigure(unsigned char ucPins)
{

  if(ucPins & LED1)
  {
    GPIO_IF_GetPortNPin(PIN_LED1,
                        &g_uiLED1Port,
                        &g_ucLED1Pin);
  }

  if(ucPins & LED2)
  {
    GPIO_IF_GetPortNPin(PIN_LED2,
                  &g_uiLED2Port,
          &g_ucLED2Pin);
  }

  if(ucPins & LED3)
  {
    GPIO_IF_GetPortNPin(PIN_LED3,
                      &g_uiLED3Port,
                      &g_ucLED3Pin);

  }

}


//*****************************************************************************
//
//! Turn LED On
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
GPIO_IF_LedOn(char ledNum)
{
  switch(ledNum)
  {
    case MCU_GREEN_LED_GPIO:
    {
      /* Switch ON GREEN LED */
      GPIO_IF_Set(PIN_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
      break;
    }
    case MCU_ORANGE_LED_GPIO:
    {
      /* Switch ON ORANGE LED */
      GPIO_IF_Set(PIN_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
      break;
    }
    case MCU_RED_LED_GPIO:
    {
      /* Switch ON RED LED */
      GPIO_IF_Set(PIN_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
      break;
    }
    case MCU_ALL_LED_IND:
    {
      /* Switch ON ALL LEDs LED */
      GPIO_IF_Set(PIN_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
      GPIO_IF_Set(PIN_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
      GPIO_IF_Set(PIN_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
      break;
    }
    default:
      break;
  }

}

//*****************************************************************************
//
//! Turn LED Off
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
GPIO_IF_LedOff(char ledNum)
{
  switch(ledNum)
  {
    case MCU_GREEN_LED_GPIO:
    {
      /* Switch OFF GREEN LED */
      GPIO_IF_Set(PIN_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
      break;
    }
    case MCU_ORANGE_LED_GPIO:
    {
      /* Switch OFF ORANGE LED */
      GPIO_IF_Set(PIN_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
      break;
    }
    case MCU_RED_LED_GPIO:
    {
      /* Switch OFF RED LED */
      GPIO_IF_Set(PIN_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
      break;
    }
    case MCU_ALL_LED_IND:
    {
      /* Switch OFF ALL LEDs LED */
      GPIO_IF_Set(PIN_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
      GPIO_IF_Set(PIN_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
      GPIO_IF_Set(PIN_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
      break;
    }
    default:
      break;
  }
}

//*****************************************************************************
//
//!  \brief This function returns LED current Status
//!
//!  \param[in] ucGPIONum is the GPIO to which the LED is connected
//!				MCU_GREEN_LED_GPIO\MCU_ORANGE_LED_GPIO\MCU_RED_LED_GPIO
//!
//!
//!  \return 1: LED ON, 0: LED OFF
//
//*****************************************************************************
unsigned char
GPIO_IF_LedStatus(unsigned char ucGPIONum)
{
  unsigned char ucLEDStatus;
  switch(ucGPIONum)
  {
    case MCU_GREEN_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED3Port, g_ucLED3Pin);
      break;
    }
    case MCU_ORANGE_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED2Port, g_ucLED2Pin);
      break;
    }
    case MCU_RED_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED1Port, g_ucLED1Pin);
      break;
    }
    default:
    	ucLEDStatus = 0;
  }
  return ucLEDStatus;

}

//*****************************************************************************
//
//! Toggle the Led state
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Toggles a board LED
//
//*****************************************************************************
void GPIO_IF_LedToggle(unsigned char ucLedNum)
{

  unsigned char ucLEDStatus = GPIO_IF_LedStatus(ucLedNum);
  if(ucLEDStatus == 1)
  {
          GPIO_IF_LedOff(ucLedNum);
  }
  else
  {
          GPIO_IF_LedOn(ucLedNum);
  }
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
GPIO_IF_Set(unsigned char ucPin,
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

void GPIOIntInit(unsigned long ulPort, unsigned char ucPin, unsigned long ulInterrupt, void (*pfnHandler)(void), unsigned long ulIntType, unsigned char ucPriority) {

	  IntRegister(ulInterrupt, pfnHandler);
	  IntPrioritySet(ulInterrupt, INT_PRIORITY_LVL_1);


	  GPIOIntTypeSet(ulPort,ucPin,ulIntType);
	  GPIOIntClear(ulPort,0xFF);
	  GPIOIntEnable(ulPort,ucPin);
	  IntEnable(ulInterrupt);
}
