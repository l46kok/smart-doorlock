/*
 * s_flash.h
 *
 *  Created on: Oct 23, 2016
 *      Author: shuh
 */

#ifndef S_FLASH_H_
#define S_FLASH_H_

/* Configuration Data Access options */
#define SF_DELETE_DATA_RECORD		(1)
#define SF_CREATE_DATA_RECORD		(2)
#define SF_WRITE_DATA_RECORD		(3)
#define SF_READ_DATA_RECORD 		(4)
#define SF_TEST_DATA_RECORD 		(5)

typedef struct
{
	unsigned char iotEnabled;
	unsigned char nfcEnabled;
	char SSID[40];
	char Password[40];
	int regDoorlockCount;
	char doorlockPhoneId[10][40];
}ConfigData_t;

// initialize configuration data
extern ConfigData_t g_ConfigData;

extern long ManageConfigData(unsigned char Mode);




#endif /* S_FLASH_H_ */
