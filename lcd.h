/*
 * lcd.h
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#ifndef LCD_H_
#define LCD_H_

#define LCD_LINE1	0
#define LCD_LINE2	LCD_LINE1+0x20
#define LCD_LINE3	LCD_LINE1+0x40
#define	LCD_LINE4 	LCD_LINE1+0x60

typedef enum
{
	LCD_INIT,
	DISPLAY_ON,
	CLEAR_SCREEN,
	RETURN_HOME
} lcdCommandEnum;

extern void lcdInit();
extern void lcdReset();
extern void lcdDisplayOn();
extern void lcdClearScreen(void);
extern void lcdPutString(unsigned char* str);
extern void lcdPutChar(unsigned char lcdChar);
extern void lcdSetPosition(unsigned char position);
#endif /* LCD_H_ */
