/*
 * Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
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