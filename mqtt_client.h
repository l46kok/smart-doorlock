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
    BROKER_DISCONNECTION,
	DOORLOCK_OPEN
}events;

typedef struct
{
	void * hndl;
	events event;
}event_msg;

extern int initMqtt();
extern int mqttConnect();
extern int attemptReconnect();
extern void Mqtt_ClientExit();
extern void MqttPublishLockAccess(unsigned char *data);

extern OsiMsgQ_t g_PBQueue;


#endif /* MQTT_CLIENT_H_ */
