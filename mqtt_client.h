/*
 * mqtt_client.h
 *
 *  Created on: Sep 26, 2016
 *      Author: shuh
 */

#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_

typedef enum
{
    PUSH_BUTTON_SW2_PRESSED,
    PUSH_BUTTON_SW3_PRESSED,
    BROKER_DISCONNECTION
}events;

typedef struct
{
	void * hndl;
	events event;
}event_msg;

extern int initMqtt();

extern OsiMsgQ_t g_PBQueue;


#endif /* MQTT_CLIENT_H_ */
