/*
 * network.h
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#ifndef NETWORK_H_
#define NETWORK_H_

extern unsigned long  g_ulStatus;

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

int ConnectAP(const char* ssidName, const char* securityKey);
long Network_IF_DeInitDriver(void);
long Network_IF_DisconnectFromAP();



#endif /* NETWORK_H_ */
