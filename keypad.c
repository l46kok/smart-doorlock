/*
 * keypad.c
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#include "keypad.h"
#include "gpio_if.h"

buttonEnum getPressedButton() {
	static unsigned int rowEnum = 0;
	GPIO_IF_Set(PIN_KEYPAD_R1,1);
	GPIO_IF_Set(PIN_KEYPAD_R2,1);
	GPIO_IF_Set(PIN_KEYPAD_R3,1);
	GPIO_IF_Set(PIN_KEYPAD_R4,1);

	rowEnum++;
	if (rowEnum >= MAX_KEYPAD_ROWS)
		rowEnum = 0;

	switch (rowEnum) {
		case 0:
			GPIO_IF_Set(PIN_KEYPAD_R1,0);
			return GPIO_IF_Get(PIN_KEYPAD_C2) == 0 ? UP_ARROW : NONE;
		case 1:
			GPIO_IF_Set(PIN_KEYPAD_R2,0);
			if (GPIO_IF_Get(PIN_KEYPAD_C1) == 0)
				return LEFT_ARROW;
			else if (GPIO_IF_Get(PIN_KEYPAD_C2) == 0)
				return DOWN_ARROW;
			else if (GPIO_IF_Get(PIN_KEYPAD_C3) == 0)
				return RIGHT_ARROW;
			return NONE;
		case 2:
			return NONE;
		case 3:
			GPIO_IF_Set(PIN_KEYPAD_R4,0);//
			if (GPIO_IF_Get(PIN_KEYPAD_C1) == 0)
				return ENTER;
			if (GPIO_IF_Get(PIN_KEYPAD_C3) == 0)
				return CANCEL;
			return NONE;
	}



	return NONE;
}
