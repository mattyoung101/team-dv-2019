#pragma once
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "defines.h"

// Quick note about the watchdog timer: while most other tasks are watched, the WiFi isn't because its intended
// to block for long periods of time (and this is OK by design since its a thread - hopefully!)

// Handles robot to robot communication over wifi using a TCP client and server

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_BUF_SIZE 32

typedef enum {
    /** One device is notifying the device of a thing, response not required **/
    WTYPE_NOTIFY = 1,
    /** Client sends sensor data to host, host is to send its own sensor data back **/
    WTYPE_SENSORDATA_XCHG,
    /** One device is asking another device a question, response expected **/
    WTYPE_QUERY
} wifi_packet_type_t;

typedef enum {
    WMSG_OK = 1,
    WMSG_YES,
    WMSG_NO,
    WMSG_HELLO,
    WMSG_GOODBYE,
    WMSG_PING,
} wifi_message_t;

/** Initialises the TCPIP stack as the host (i.e. makes an access point and starts the host thread) **/
void comms_wifi_init_host(void);
/** Initialises the TCPIP stack as the client (i.e. tries to connect to a host) **/
void comms_wifi_init_client(void);