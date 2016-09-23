/*
 * globals.h
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#define SSID_LEN_MAX        32
#define BSSID_LEN_MAX       6


//Simplelink (Wi-Fi) related globals

extern char 			g_APSsidName[33];    // AP SSID name
extern unsigned char   g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
extern unsigned char   g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

extern unsigned char   g_ulStatus;
extern unsigned long  	g_ulStaIp; 						// STA IP number
extern unsigned long  	g_ulGatewayIP; 					//Network Gateway IP address
extern unsigned long  	g_ulAcquiredIP;
extern unsigned long  	g_ulPingPacketsRecv;

//Timer (Keypad) global
extern volatile unsigned long g_ulBase;


#endif /* GLOBALS_H_ */
