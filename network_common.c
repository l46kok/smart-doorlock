/*
 * network_common.c
 *
 *  Created on: 2016. 9. 7.
 *      Author: Sokwhan
 */



#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function serves as first level handler for HTTP GET/POST tokens
//!        It runs under driver context and performs only operation that can run
//!        from this context. For operations that can't is sets an indication of
//!        received token and preempts the provisioning context.
//!
//! \param pSlHttpServerEvent Pointer indicating http server event
//! \param pSlHttpServerResponse Pointer indicating http server response
//!
//! \return None
//!
//*****************************************************************************
_SlEventPropogationStatus_e sl_Provisioning_HttpServerEventHdl(
                            SlHttpServerEvent_t    *apSlHttpServerEvent,
                            SlHttpServerResponse_t *apSlHttpServerResponse)
{
	// Unused in this application
	return EVENT_PROPAGATION_CONTINUE;
}

//*****************************************************************************
//
//! \brief This function serves as first level network application events handler.
//!        It runs under driver context and performs only operation that can run
//!        from this context. For operations that can't is sets an indication of
//!        received token and preempts the provisioning context.
//!
//! \param apEventInfo Pointer to the net app event information
//!
//! \return None
//!
//*****************************************************************************
_SlEventPropogationStatus_e sl_Provisioning_NetAppEventHdl(SlNetAppEvent_t *apNetAppEvent)
{
	// Unused in this application
	return EVENT_PROPAGATION_CONTINUE;
}

//*****************************************************************************
//
//! \brief This function serves as first level WLAN events handler.
//!        It runs under driver context and performs only operation that can run
//!        from this context. For operations that can't is sets an indication of
//!        received token and preempts the provisioning context.
//!
//! \param apEventInfo Pointer to the WLAN event information
//!
//! \return None
//!
//*****************************************************************************
_SlEventPropogationStatus_e sl_Provisioning_WlanEventHdl(SlWlanEvent_t *apEventInfo)
{
	// Unused in this application
	return EVENT_PROPAGATION_CONTINUE;
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************
