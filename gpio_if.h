/*
 * gpio_if.h
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#ifndef __GPIOIF_H__
#define __GPIOIF_H__

//GPIO # Definitions

typedef enum
{
	 PIN_LCD_RS = 0, //4 RS (CS) H / L H=Data, L=Command
	 PIN_LCD_RW = 3, //5 R/W (SID) H / L H=Read, L=Write
	 PIN_LCD_E = 4, //6 E (SCLK) H Enable (falling edge)
	 PIN_LCD_D0 = 5, //7 D0 (SOD) H / L Display Data, LSB
	 PIN_LCD_D1 = 6, //8 D1 H / L Display Data
	 PIN_LCD_D2 = 7, //9 D2 H / L Display Data
	 PIN_LCD_D3 = 8, //10 D3 H / L Display Data
	 PIN_LCD_D4 = 9, //11 D4 (D0) H / L Display Data
	 PIN_LCD_D5 = 10, //12 D5 (D1) H / L Display Data
	 PIN_LCD_D6 = 11, //13 D6 (D2) H / L Display Data
	 PIN_LCD_D7 = 12 //14 D7 (D3) H / L Display Data, MSB
} lcdPinEnum;


typedef enum
{
	PIN_KEYPAD_B1 = 3,
	PIN_KEYPAD_B2 = 4,
	PIN_KEYPAD_B3 = 5,
	PIN_KEYPAD_B4 = 6,
} keypadPinEnum;

#define PIN_BUZZER 7


//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern void GPIO_IF_GetPortNPin(unsigned char ucPin,
                     unsigned int *puiGPIOPort,
                     unsigned char *pucGPIOPin);

extern void GPIO_IF_ConfigureNIntEnable(unsigned int uiGPIOPort,
                                  unsigned char ucGPIOPin,
                                  unsigned int uiIntType,
                                  void (*pfnIntHandler)(void));
extern void GPIO_IF_Set(unsigned int gpioNum, unsigned int state);
extern void GPIO_IF_Toggle(unsigned int gpioNum);

extern unsigned char GPIO_IF_GetVal(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin);
static void GPIO_IF_SetVal(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin,
             unsigned char ucGPIOValue);
extern void GPIO_Set(unsigned char ucGPIONum);
extern void GPIO_Clear(unsigned char ucGPIONum);
extern void GPIO_Toggle(unsigned char ucGPIONum);
extern unsigned char GPIO_IF_Get(unsigned int gpioNum);
void GPIOIntInit(	unsigned long ulPort,
					unsigned char ucPin,
					unsigned long ulInterrupt,
					void (*pfnHandler)(void),
					unsigned long ulIntType,
					unsigned char ucPriority);
void GPIOs3IntHandler(void);



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __GPIOIF_H__

