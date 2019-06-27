#pragma once
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "string.h"
#include "utils.h"

// VL53L0X UART driver

extern uint16_t lrfDistance;

/** Creates the LRF task */
void lrf_driver_install();