/*
 * gpio_if.h
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#ifndef __GPIOIF_H__
#define __GPIOIF_H__

typedef enum
{
    NO_LED,
    LED1 = 0x1, /* RED LED D7/GP9/Pin64 */
    LED2 = 0x2, /* ORANGE LED D6/GP10/Pin1 */
    LED3 = 0x4  /* GREEN LED D5/GP11/Pin2 */

} ledEnum;

typedef enum
{
    MCU_RED_LED_GPIO = 9,	/* GP09 for LED RED as per LP 3.0 */
    MCU_ORANGE_LED_GPIO = 10,/* GP10 for LED ORANGE as per LP 3.0 */
    MCU_GREEN_LED_GPIO = 11, /* GP11 for LED GREEN as per LP 3.0 */
    MCU_ALL_LED_IND = 12
} ledNames;

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
extern void GPIO_IF_Set(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin,
             unsigned char ucGPIOValue);

extern unsigned char GPIO_IF_Get(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin);
extern void GPIO_IF_LedConfigure(unsigned char ucPins);
extern void GPIO_IF_LedOn(char ledNum);
extern void GPIO_IF_LedOff(char ledNum);
extern unsigned char GPIO_IF_LedStatus(unsigned char ucGPIONum);
extern void GPIO_IF_LedToggle(unsigned char ucLedNum);
extern void GPIO_Set(unsigned char ucGPIONum);
extern void GPIO_Clear(unsigned char ucGPIONum);
extern void GPIO_Toggle(unsigned char ucGPIONum);
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

