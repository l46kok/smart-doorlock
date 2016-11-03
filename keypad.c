/*
 * keypad.c
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#include "keypad.h"
#include "gpio_if.h"

buttonEnum getPressedButton() {
	if (GPIO_IF_Get(PIN_KEYPAD_B1) == 0)
		return CANCEL;
	if (GPIO_IF_Get(PIN_KEYPAD_B2) == 0)
		return ENTER;
	if (GPIO_IF_Get(PIN_KEYPAD_B3) == 0)
		return DOWN_ARROW;
	if (GPIO_IF_Get(PIN_KEYPAD_B4) == 0)
		return UP_ARROW;

	return NONE;
}
