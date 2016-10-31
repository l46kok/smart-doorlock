/*
 * nfc.h
 *
 *  Created on: 2016. 10. 12.
 *      Author: Sokwhan
 */

#ifndef NFC_H_
#define NFC_H_

typedef enum
{
	NFC_NONE,
	NFC_INVALID_PAYLOAD,
    NFC_REG_PHONE,
	NFC_UNREGISTERED_PHONE,
	NFC_OPEN_DOORLOCK,
	NFC_WIFI_CONFIG
} nfcCmdEnum;

extern void NFCInit();
extern nfcCmdEnum readNFCTag();

extern char nfcCmdPayload[100];
extern char nfcCmdPayload2[100];

#endif /* NFC_H_ */
