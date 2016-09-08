/*
 * Smart Doorlock
 *
 * pinmux.c
 *
 *  Created on: 2016. 8. 27.
 *      Author: Sokwhan
 */

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
void
PinMuxConfig(void)
{
    // Enable Peripheral Clocks
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    // Reserved for Future Use (Probably IRQ)
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    // Used for LED1,2,3
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);

    // Configure PIN_55 for UART0 UART0_TX
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);

    // Configure PIN_57 for UART0 UART0_RX
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);

    // Configure PIN_64 for LED1 (GPIO9)
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    // Configure PIN_01 for LED2
	MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

    // Configure PIN_02 for LED3
	MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);
}
