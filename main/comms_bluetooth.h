#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "defines.h"
#include "utils.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "bluetooth.pb.h"
#include "math.h"
#include "freertos/task.h"

/** received packets are pushed into this queue and read by the BT manager task in bluetooth_manager.c */
extern QueueHandle_t packetQueue;
extern TaskHandle_t receiveTaskHandle;
extern TaskHandle_t sendTaskHandle;

/** Initialises the Bluetooth stack as a slave, or initiator **/
void comms_bt_init_slave();
/** Initialises the Bluetooth stack as a master, or acceptor **/
void comms_bt_init_master();
/** Bluetooth receive task */
void comms_bt_receive_task(void *pvParameter);
/** Bluetooth send task */
void comms_bt_send_task(void *pvParameter);
/** stops the send and receive tasks */
void comms_bt_stop_tasks(void);
/** reinits bluetooth **/
void comms_bt_reinit(void);