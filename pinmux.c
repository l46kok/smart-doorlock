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
	//
	// Enable Peripheral Clocks
	//
	MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);

	//
	// Configure PIN_55 for UART0 UART0_TX
	//
	MAP_PinTypeUART(PIN_55, PIN_MODE_3);

	//
	// Configure PIN_57 for UART0 UART0_RX
	//
	MAP_PinTypeUART(PIN_57, PIN_MODE_3);

	//
	// Configure PIN_50 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_50, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x1, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_58 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_59 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_59, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_60 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_60, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x20, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_61 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_61, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x40, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_62 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_63 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_63, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x1, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_64 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_01 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_02 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_03 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_04 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_05 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_05, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x40, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_06 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_06, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x80, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_08 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_08, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA2_BASE, 0x2, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_18 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_18, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_21 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_21, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA3_BASE, 0x2, GPIO_DIR_MODE_OUT);

	//
	// Configure PIN_53 for GPIO Output
	//
	MAP_PinTypeGPIO(PIN_53, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA3_BASE, 0x40, GPIO_DIR_MODE_OUT);
}
