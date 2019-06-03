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

/** Initialises the Bluetooth stack as a slave, or initiator **/
void comms_bt_init_slave();
/** Initialises the Bluetooth stack as a master, or acceptor **/
void comms_bt_init_master();
/** task which controls bluetooth slave logic **/
void comms_bt_slave_controller_task(void *pvParameter);
/** task which controls bluetooth master logic **/
void comms_bt_master_controller_task(void *pvParameter);