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
	LCD_INIT,
	DISPLAY_ON,
	CLEAR_SCREEN,
	RETURN_HOME
} lcdCommandEnum;

extern void lcdInit();
extern void lcdPutChar(unsigned char lcdChar);
extern void lcdDisplayOn();
extern void lcdClearScreen(void);
extern void lcdPutString(unsigned char* str);
#endif /* LCD_H_ */
