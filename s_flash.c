/*
 * s_flash.c
 *
 *  Created on: Oct 23, 2016
 *      Author: shuh
 */

#include "simplelink.h"
#include "s_flash.h"

#define CONFIG_FILE_NAME        	"SD_Config.txt"

long g_isFileHandle;								// Configuration Record File Handler

// initialize configuration data
ConfigData_t g_ConfigData =
{
		1, \
		"A", \
		"A", \
		"A", \
		"A"
};


//****************************************************************************
//
//! Manages the configuration data in the s-flash
//!
//! \param Mode indicates type of requested operation
//!
//!  Possible Mode values: SF_TEST_DATA_RECORD, SF_CREATE_DATA_RECORD, SF_WRITE_DATA_RECORD, SF_READ_DATA_RECORD, SF_DELETE_DATA_RECORD
//!
//!
//! \return 0 on success.
//
//****************************************************************************
long ManageConfigData(unsigned char Mode)
{

	long lRetVal;
	unsigned long ulToken;

	switch (Mode)
	{
		case SF_TEST_DATA_RECORD:
			lRetVal = sl_FsOpen((unsigned char *)CONFIG_FILE_NAME, FS_MODE_OPEN_READ, &ulToken, &g_isFileHandle);
			sl_FsClose(g_isFileHandle,0,0,NULL);

			break;

		case SF_CREATE_DATA_RECORD:
			sl_FsOpen((unsigned char *) CONFIG_FILE_NAME, FS_MODE_OPEN_CREATE(1024,_FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE), &ulToken,&g_isFileHandle);
			sl_FsWrite(g_isFileHandle, 0, (unsigned char *) &g_ConfigData, sizeof(g_ConfigData));
			sl_FsClose(g_isFileHandle,0,0,NULL);

			break;


		case SF_WRITE_DATA_RECORD:
			sl_FsOpen((unsigned char *) CONFIG_FILE_NAME, FS_MODE_OPEN_WRITE, &ulToken,&g_isFileHandle);
			sl_FsWrite(g_isFileHandle, 0, (unsigned char *) &g_ConfigData, sizeof(g_ConfigData));
			sl_FsClose(g_isFileHandle,0,0,NULL);

			break;


		case SF_READ_DATA_RECORD:
			sl_FsOpen((unsigned char *) CONFIG_FILE_NAME,FS_MODE_OPEN_READ, &ulToken,&g_isFileHandle);
			sl_FsRead(g_isFileHandle,0, (unsigned char *) &g_ConfigData, sizeof(g_ConfigData));
			sl_FsClose(g_isFileHandle,0,0,NULL);

			break;


		case SF_DELETE_DATA_RECORD:
			sl_FsDel((unsigned char *) CONFIG_FILE_NAME,0);

			break;


		default:
			lRetVal = -1;

			break;

	}

	return lRetVal;

}

