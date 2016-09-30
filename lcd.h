/*
 * lcd.h
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#ifndef LCD_H_
#define LCD_H_

#define CMD_START_BIT 0xF8 //5 1s for start bit
#define CMD_RW 2
#define CMD_RS 1
#define CMD_DATA_SHIFT_BIT 4

typedef enum
{
	 CLEAR_SCREEN,
	 WRITE_CHAR
} lcdCommandEnum;

extern void lcdPutChar(unsigned char lcdChar);
#endif /* LCD_H_ */
