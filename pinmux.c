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
	MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);

	//
	// Configure PIN_05 for SPI0 GSPI_CLK
	//
	MAP_PinTypeSPI(PIN_05, PIN_MODE_7);

	//
	// Configure PIN_06 for SPI0 GSPI_MISO
	//
	MAP_PinTypeSPI(PIN_06, PIN_MODE_7);

	//
	// Configure PIN_07 for SPI0 GSPI_MOSI
	//
	MAP_PinTypeSPI(PIN_07, PIN_MODE_7);

	//
	// Configure PIN_08 for SPI0 GSPI_CS
	//
	MAP_PinTypeSPI(PIN_08, PIN_MODE_7);

    //
    // Configure PIN_03 for TRF IRQ Input (GPIO 12)
    //
    MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x10, GPIO_DIR_MODE_IN);

	//
    // Configure PIN_02 for TRF enable Output (GPIO11, LED D5)
	//
	MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);


	// GPIO Definitions for keypad
	// Pins 58/59/60/61 - GPIO 3,4,5,6 (Rows, Output)
	// Pins 62/63/64 - GPIO 7,8,9 (Columns, Input)

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
	// Configure PIN_62 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_63 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_63, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x1, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_64 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_04 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_15 for GPIO Input
	//
	MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
	MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);

	//
	// Configure PIN_55 for UART0 UART0_TX
	//
	MAP_PinTypeUART(PIN_55, PIN_MODE_3);

	//
	// Configure PIN_57 for UART0 UART0_RX
	//
	MAP_PinTypeUART(PIN_57, PIN_MODE_3);

}
