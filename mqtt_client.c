/*
 * mqtt_client.c
 *
 *  Created on: Sep 26, 2016
 *      Author: shuh
 */


// Standard includes
#include <stdlib.h>

// Common Interface Includes
#include "common.h"
#include "sl_mqtt_client.h"
#include "uart_if.h"
#include "network.h"

// Project Includes
#include "mqtt_client.h"


/*Operate Lib in MQTT 3.1 mode.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS2
#define WILL_RETAIN             false

/*Background receive task priority*/
#define TASK_PRIORITY           3

#define UART_PRINT              Report

/*Defining Number of topics*/
#define TOPIC_COUNT             1

/* Keep Alive Timer value*/
#define KEEP_ALIVE_TIMER        25

/*Retain Flag. Used in publish message. */
#define RETAIN                  1

/*Defining Broker IP address and port Number*/
#define SERVER_ADDRESS          "192.168.2.2"
#define PORT_NUMBER             1883

#define SERVER_MODE             MQTT_3_1
/*Specifying Receive time out for the Receive task*/
#define RCV_TIMEOUT             30

/*Defining QOS levels*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Defining Subscription Topic Values*/
#define TOPIC1                  "/SmartDoorlock/DoorControl"

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
          long pay_len, bool dup,unsigned char qos, bool retain);
static void sl_MqttEvt(void *app_hndl,long evt, const void *buf,
                       unsigned long len);
static void sl_MqttDisconnect(void *app_hndl);
int initMqtt();

/* library configuration */
SlMqttClientLibCfg_t Mqtt_Client={
    1882,
    TASK_PRIORITY,
    30,
    true,
    (long(*)(const char *, ...))UART_PRINT
};

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    unsigned char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[TOPIC_COUNT];
    unsigned char qos[TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connect_config;


/* connection configuration */
connect_config usr_connect_config[] =
{
    {
        {
            {
                SL_MQTT_NETCONN_URL,
                SERVER_ADDRESS,
                PORT_NUMBER,
                0,
                0,
                0,
                NULL
            },
            SERVER_MODE,
            true,
        },
        NULL,
        "user1",
        NULL,
        NULL,
        true,
        KEEP_ALIVE_TIMER,
        {Mqtt_Recv, sl_MqttEvt, sl_MqttDisconnect},
        TOPIC_COUNT,
        {TOPIC1},
        {QOS2},
        {WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
        false
    }
};


/*Message Queue*/
OsiMsgQ_t g_PBQueue;

void *app_hndl = (void*)usr_connect_config;

int mqttConnect() {
    int iCount = 0;

    connect_config *local_con_conf = (connect_config *)app_hndl;

    //create client context
	local_con_conf[iCount].clt_ctx =
	sl_ExtLib_MqttClientCtxCreate(&local_con_conf[iCount].broker_config,
								  &local_con_conf[iCount].CallBAcks,
								  &(local_con_conf[iCount]));

	//
	// Set Client ID
	//
	sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
						SL_MQTT_PARAM_CLIENT_ID,
						local_con_conf[iCount].client_id,
						strlen((char*)(local_con_conf[iCount].client_id)));

	//
	// Set will Params
	//
	if(local_con_conf[iCount].will_params.will_topic != NULL)
	{
		sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
								SL_MQTT_PARAM_WILL_PARAM,
								&(local_con_conf[iCount].will_params),
								sizeof(SlMqttWill_t));
	}

	//
	// setting username and password
	//
	if(local_con_conf[iCount].usr_name != NULL)
	{
		sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
							SL_MQTT_PARAM_USER_NAME,
							local_con_conf[iCount].usr_name,
							strlen((char*)local_con_conf[iCount].usr_name));

		if(local_con_conf[iCount].usr_pwd != NULL)
		{
			sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
							SL_MQTT_PARAM_PASS_WORD,
							local_con_conf[iCount].usr_pwd,
							strlen((char*)local_con_conf[iCount].usr_pwd));
		}
	}

	//
	// connectin to the broker
	//
	if((sl_ExtLib_MqttClientConnect((void*)local_con_conf[iCount].clt_ctx,
						local_con_conf[iCount].is_clean,
						local_con_conf[iCount].keep_alive_time) & 0xFF) != 0)
	{
		UART_PRINT("\n\rBroker connect fail for conn no. %d \n\r",iCount+1);

		//delete the context for this connection
		sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);

		return -1;
	}
	else
	{
		UART_PRINT("\n\rSuccess: conn to Broker no. %d\n\r ", iCount+1);
		local_con_conf[iCount].is_connected = true;
	}

	//
	// Subscribe to topics
	//

	if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
							   local_con_conf[iCount].topic,
							   local_con_conf[iCount].qos, TOPIC_COUNT) < 0)
	{
		UART_PRINT("\n\r Subscription Error for conn no. %d\n\r", iCount+1);
		UART_PRINT("Disconnecting from the broker\r\n");
		sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
		local_con_conf[iCount].is_connected = false;

		//delete the context for this connection
		sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
		return -1;
	}
	else
	{
		int iSub;
		UART_PRINT("Client subscribed on following topics:\n\r");
		for(iSub = 0; iSub < local_con_conf[iCount].num_topics; iSub++)
		{
			UART_PRINT("%s\n\r", local_con_conf[iCount].topic[iSub]);
		}
	}
	return 0;
}

int attemptReconnect() {
	if(!IS_CONNECTED(g_ulStatus))
	{
		UART_PRINT("device has disconnected from AP \n\r");

		UART_PRINT("retry connection to the AP\n\r");

		while(!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
		{
			osi_Sleep(10);
		}
	}
	return mqttConnect();
}

int initMqtt() {
	long lRetVal = -1;
    lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
    if(lRetVal != 0)
    {
        // lib initialization failed
        UART_PRINT("MQTT Client lib initialization failed\n\r");
        return -1;
    }

/*
	//
	// Deinitializating the client library
	//
	sl_ExtLib_MqttClientExit();
	UART_PRINT("\n\r Exiting the Application\n\r");
*/

	return 0;
}


void Mqtt_ClientExit() {
	sl_ExtLib_MqttClientExit();
}


//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init
//! API. Background receive task invokes this handler whenever MQTT Client
//! receives a Publish Message from the broker.
//!
//!\param[out]     topstr => pointer to topic of the message
//!\param[out]     top_len => topic length
//!\param[out]     payload => pointer to payload
//!\param[out]     pay_len => payload length
//!\param[out]     retain => Tells whether its a Retained message or not
//!\param[out]     dup => Tells whether its a duplicate message or not
//!\param[out]     qos => Tells the Qos level
//!
//!\return none
//****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
                       long pay_len, bool dup,unsigned char qos, bool retain)
{

    char *topic_str=(char*)malloc(top_len+1);
    memset(topic_str,'\0',top_len+1);
    strncpy(topic_str, (char*)topstr, top_len);
    topic_str[top_len]='\0';

    char *data_str=(char*)malloc(pay_len+1);
    memset(data_str,'\0',pay_len+1);
    strncpy(data_str, (char*)payload, pay_len);
    data_str[pay_len]='\0';


    if(strncmp(topic_str, TOPIC1, top_len) == 0)
    {
        event_msg msg;
        msg.hndl = app_hndl;
        msg.event = DOORLOCK_OPEN;

        // write message indicating publish message
        osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
    }

    UART_PRINT("\n\rPublish Message Received");
    UART_PRINT("\n\rTopic: %s\n\r", topic_str);
    free(topic_str);

    UART_PRINT(" [Qos: %d] ",qos);
    if(retain)
      UART_PRINT(" [Retained]");
    if(dup)
      UART_PRINT(" [Duplicate]");

    UART_PRINT("\n\rData is: %s\n\r", data_str);
    free(data_str);

    return;
}


//****************************************************************************
//! Defines sl_MqttEvt event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init
//! API. Background receive task invokes this handler whenever MQTT Client
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//! param[out]      evt => Event that invokes the handler. Event can be of the
//!                        following types:
//!                        MQTT_ACK - Ack Received
//!                        MQTT_ERROR - unknown error
//!
//!
//! \param[out]     buf => points to buffer
//! \param[out]     len => buffer length
//!
//! \return none
//****************************************************************************
static void
sl_MqttEvt(void *app_hndl, long evt, const void *buf,unsigned long len)
{
    int i;
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        UART_PRINT("PubAck:\n\r");
        UART_PRINT("%s\n\r",buf);
        break;

      case SL_MQTT_CL_EVT_SUBACK:
        UART_PRINT("\n\rGranted QoS Levels are:\n\r");

        for(i=0;i<len;i++)
        {
          UART_PRINT("QoS %d\n\r",((unsigned char*)buf)[i]);
        }
        break;

      case SL_MQTT_CL_EVT_UNSUBACK:
        UART_PRINT("UnSub Ack \n\r");
        UART_PRINT("%s\n\r",buf);
        break;

      default:
        break;

    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection
//!
//! \param app_hndl is the handle for the disconnected connection
//!
//! return none
//
//****************************************************************************
static void
sl_MqttDisconnect(void *app_hndl)
{
    connect_config *local_con_conf;
    event_msg msg;
    local_con_conf = app_hndl;
    msg.hndl = app_hndl;
    msg.event = BROKER_DISCONNECTION;

    UART_PRINT("disconnect from broker %s\r\n",
           (local_con_conf->broker_config).server_info.server_addr);
    local_con_conf->is_connected = false;
	//Derive the value of the local_con_conf or clt_ctx from the message
	sl_ExtLib_MqttClientCtxDelete(local_con_conf->clt_ctx);

    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}
