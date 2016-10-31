/*
 * menu.h
 *
 *  Created on: Oct 31, 2016
 *      Author: shuh
 */

#ifndef MENU_H_
#define MENU_H_

#include "keypad.h"

extern void MoveMenu(int menuOption);
extern void MoveConfigMenu(int menuOption);
extern void MenuProcessMain(buttonEnum pressedBtn);
extern void MenuProcessConfig(buttonEnum pressedBtn);
extern void MenuProcessConfigInner(buttonEnum pressedBtn);

#endif /* MENU_H_ */
