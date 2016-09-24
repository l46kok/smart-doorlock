/*
 * lcd.h
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#ifndef LCD_H_
#define LCD_H_

#define SPI_IF_BIT_RATE  100000

extern void lcdInit(void);
extern void lcdPutChar(char lcdChar);
#endif /* LCD_H_ */
