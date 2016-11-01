/*
 * lcd.h
 *
 *  Created on: Sep 23, 2016
 *      Author: shuh
 */

#ifndef LCD_H_
#define LCD_H_



typedef enum
{
    LCD_DISP_INIT,
	LCD_DISP_CONNECT_AP,
	LCD_DISP_CONNECT_MQTT,
	LCD_DISP_ACTIVE,
	LCD_DISP_REGISTER_ACTIVE,
	LCD_DISP_OPENING_DOOR,
	LCD_DISP_UNREGISTERED_PHONE_TAPPED,
	LCD_DISP_REGISTERING_PHONE,
	LCD_DISP_EXITING_APP,
	LCD_DISP_NFC_DISABLED,
	LCD_DISP_NO_PHONE_REGISTERED,
	LCD_DISP_FACTORY_RESET,
	LCD_DISP_REBOOTING,
	LCD_DISP_UNREGISTER_PHONE_SUCCESS,
	LCD_DISP_FIRST_TIME_SETUP
} sdLcdEnum;

extern void lcdInit();
extern void lcdClearScreen(void);
extern void lcdPutString(unsigned char* str);
extern void lcdPutChar(unsigned char lcdChar);
extern void lcdSetPosition(unsigned int position);
extern void SmartDoorlockLCDDisplay(sdLcdEnum lcdEnum);

#endif /* LCD_H_ */
