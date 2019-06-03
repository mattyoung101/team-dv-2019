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

typedef enum {
    BT_MSG_ACK = 0,
    BT_MSG_OK,
    BT_MSG_DENIED,
    BT_MSG_UPDATE,
    BT_MSG_REQUEST_SWITCH,
    BT_MSG_SWITCH_COMPLETED,
    BT_MSG_ENQUIRE_SEM_STATUS,
    BT_MSG_TAKE_SEM,
    BT_MSG_GIVE_SEM
} bt_message_t;

/** Initialises the Bluetooth stack as a slave, or initiator **/
void comms_bt_init_slave();
/** Initialises the Bluetooth stack as a master, or acceptor **/
void comms_bt_init_master();
/** task which controls bluetooth slave logic **/
void comms_bt_slave_controller_task(void *pvParameter);
/** task which controls bluetooth master logic **/
void comms_bt_master_controller_task(void *pvParameter);