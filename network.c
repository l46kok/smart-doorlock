/*
 * network.c
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */

// Simplelink includes
#include "simplelink.h"

//Common interface includes
#include "uart_if.h"
#include "common.h"
#include "globals.h"


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

//*****************************************************************************
//
//! On Successful completion of Wlan Connect, This function triggers Connection
//! status to be set.
//!
//! \param  pSlWlanEvent pointer indicating Event type
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
	switch(pSlWlanEvent->Event)
	{
	case SL_WLAN_CONNECT_EVENT:
	{
		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		UART_PRINT("SL_WLAN_CONNECT_EVENT");
		//
		// Information about the connected AP (like name, MAC etc) will be
		// available in 'slWlanConnectAsyncResponse_t'-Applications
		// can use it if required
		//
		//  slWlanConnectAsyncResponse_t *pEventData = NULL;
		// pEventData = &pSlWlanEvent->EventData.STAandP2PModeWlanConnected;
		//
		//

        // Copy new connection SSID and BSSID to global parameters
         memcpy(g_ucConnectionSSID,pSlWlanEvent->EventData.
                STAandP2PModeWlanConnected.ssid_name,
                pSlWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
         memcpy(g_ucConnectionBSSID,
                pSlWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                SL_BSSID_LENGTH);
	}
	break;

	case SL_WLAN_DISCONNECT_EVENT:
	{
		slWlanConnectAsyncResponse_t*  pEventData = NULL;

		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		UART_PRINT("SL_WLAN_DISCONNECT_EVENT");

		pEventData = &pSlWlanEvent->EventData.STAandP2PModeDisconnected;

		// If the user has initiated 'Disconnect' request,
		//'reason_code' is SL_USER_INITIATED_DISCONNECTION
		if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
		{
			UART_PRINT("Device disconnected from the AP on application's "
					"request \n\r");
		}
		else
		{
			UART_PRINT("Device disconnected from the AP on an ERROR..!! \n\r");
		}

	}
	break;

	case SL_WLAN_STA_CONNECTED_EVENT:
	{
		// when device is in AP mode and any client connects to device cc3xxx
		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		UART_PRINT("SL_WLAN_STA_CONNECTED_EVENT");

		//
		// Information about the connected client (like SSID, MAC etc) will be
		// available in 'slPeerInfoAsyncResponse_t' - Applications
		// can use it if required
		//
		// slPeerInfoAsyncResponse_t *pEventData = NULL;
		// pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
		//

	}
	break;

	case SL_WLAN_STA_DISCONNECTED_EVENT:
	{
		UART_PRINT("SL_WLAN_STA_DISCONNECTED_EVENT");
		// when client disconnects from device (AP)
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

		//
		// Information about the connected client (like SSID, MAC etc) will
		// be available in 'slPeerInfoAsyncResponse_t' - Applications
		// can use it if required
		//
		// slPeerInfoAsyncResponse_t *pEventData = NULL;
		// pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
		//
	}
	break;

	default:
	{
		UART_PRINT("[WLAN EVENT] Unexpected event \n\r");
	}
	break;
	}
}


//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
	switch(pNetAppEvent->Event)
	{
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
	case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
	{
		SlIpV4AcquiredAsync_t *pEventData = NULL;

		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
		//Ip Acquired Event Data
		pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
		g_ulAcquiredIP = pEventData->ip;

		//Gateway IP address
		g_ulGatewayIP = pEventData->gateway;


	}
	break;

	case SL_NETAPP_IP_LEASED_EVENT:
	{
		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

		g_ulStaIp = (pNetAppEvent)->EventData.ipLeased.ip_address;

		UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d \n\r",
				SL_IPV4_BYTE(g_ulStaIp,3), SL_IPV4_BYTE(g_ulStaIp,2),
				SL_IPV4_BYTE(g_ulStaIp,1), SL_IPV4_BYTE(g_ulStaIp,0));
	}
	break;

	case SL_NETAPP_IP_RELEASED_EVENT:
	{
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

		UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d \n\r",
				SL_IPV4_BYTE(g_ulStaIp,3), SL_IPV4_BYTE(g_ulStaIp,2),
				SL_IPV4_BYTE(g_ulStaIp,1), SL_IPV4_BYTE(g_ulStaIp,0));

	}
	break;

	default:
	{
		UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
				pNetAppEvent->Event);
	}
	break;
	}
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
	//
	// Most of the general errors are not FATAL are are to be handled
	// appropriately by the application
	//
	UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
			pDevEvent->EventData.deviceEvent.status,
			pDevEvent->EventData.deviceEvent.sender);
}



//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
	//
	// This application doesn't work w/ socket - Events are not expected
	//
	switch( pSock->Event )
	{
	case SL_SOCKET_TX_FAILED_EVENT:
#if 0
		switch( pSock->EventData.status )
#else
		switch( pSock->socketAsyncEvent.SockTxFailData.status)
#endif
		{
		case SL_ECLOSE:
			break;

		default:
			UART_PRINT("[SOCK ERROR] - TX FAILED : socket %d , reason"
					"(%d) \n\n",
#if 0
					pSock->EventData.sd, pSock->EventData.status);
#else
					pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
#endif
		}
		break;

		default:
			UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
	}
}



//*****************************************************************************
//
//! This function gets triggered when HTTP Server receives Application
//! defined GET and POST HTTP Tokens.
//!
//! The project does not use any HTTP calls, so we just leave an empty body here (for linking)
//! \return None
//!
//*****************************************************************************
void sl_HttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, SlHttpServerResponse_t *pSlHttpServerResponse)
{
}



//****************************************************************************
//
//! Connects to an access point
//
//****************************************************************************
int ConnectAP(char* ssid, char* security_key)
{
	int status;
	long  lRetVal = -1;

	lRetVal = sl_Start(NULL,NULL,NULL);

	lRetVal = sl_WlanSetMode(ROLE_STA);
	ASSERT_ON_ERROR(lRetVal);


    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lRetVal);

/*
	// setting AP SSID
	lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(ssid), (unsigned char*)ssid);
	ASSERT_ON_ERROR(lRetVal);

	lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1, SL_SEC_TYPE_WPA_WPA2);
	ASSERT_ON_ERROR(lRetVal);
*/


    SlSecParams_t secParams = {0};

    secParams.Key = (signed char *)security_key;
    secParams.KeyLen = strlen(security_key);
    secParams.Type = SL_SEC_TYPE_WPA_WPA2;

	// reset status bits
	CLR_STATUS_BIT_ALL(g_ulStatus);

    status = sl_WlanConnect((signed char *)ssid, strlen(ssid), 0, &secParams, 0);

	return status;
}
