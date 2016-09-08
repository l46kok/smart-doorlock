/*
 * globals.c
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

#include "globals.h"

char 			g_APSsidName[33];    // AP SSID name
unsigned char   g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char   g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

unsigned char   g_ulStatus = 0;
unsigned long  	g_ulStaIp = 0; 						// STA IP number
unsigned long  	g_ulGatewayIP = 0; 					//Network Gateway IP address
unsigned long  	g_ulAcquiredIP = 0;
unsigned long  	g_ulPingPacketsRecv = 0;
