/*
 * sd_globals.h
 *
 *  Created on: Oct 31, 2016
 *      Author: shuh
 */

#ifndef SD_GLOBALS_H_
#define SD_GLOBALS_H_

typedef enum
{
	MODE_INITIALIZING,
	MODE_INITIALIZING_IOT,
	MODE_INITIALIZING_NFC,
	MODE_INITIALIZE_COMPLETE,
	MODE_MENU,
	MODE_ACTIVE,
	MODE_CONFIG,
	MODE_OPENING_DOOR,
	MODE_OPERATION_SETUP,
	MODE_REGISTER_ACTIVE,
	MODE_REGISTERING_PHONE,
	MODE_UNREGISTER_PHONE,
	MODE_EXIT
} appModeEnum;

typedef enum
{
	MENU_ACTIVE,
	MENU_CONFIG,
	MENU_EXIT
} appMenuEnum;

typedef enum
{
	MENU_OPERATION_SETUP,
	MENU_REGISTER_PHONE,
	MENU_UNREGISTER_PHONE,
	MENU_WIFI_CONFIG,
	MENU_WIFI_TEST,
	MENU_FACTORY_RESET
} configMenuEnum;

typedef enum
{
	OPER_NFC_ONLY,
	OPER_IOT_ONLY,
	OPER_NFC_IOT
} operEnum;

//Globals
extern unsigned int g_appMode;
extern unsigned int g_currMenuOption;

#endif /* SD_GLOBALS_H_ */
