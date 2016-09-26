/*
 * network.h
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#ifndef NETWORK_H_
#define NETWORK_H_

extern unsigned char   g_ulStatus;

int ConnectAP(const char* ssidName, const char* securityKey);
long Network_IF_DeInitDriver(void);
long Network_IF_DisconnectFromAP();

#endif /* NETWORK_H_ */
