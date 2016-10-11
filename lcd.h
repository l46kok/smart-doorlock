/*
 * lcd.h
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#ifndef LCD_H_
#define LCD_H_

extern void lcdInit();
extern void lcdClearScreen(void);
extern void lcdPutString(unsigned char* str);
extern void lcdPutChar(unsigned char lcdChar);
extern void lcdSetPosition(unsigned int position);
#endif /* LCD_H_ */
