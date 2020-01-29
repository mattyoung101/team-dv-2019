/*
 * Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#include "esp_timer.h"

// Ported from PID.cpp 

#ifndef constrain
    #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

typedef struct {
    float kp, ki, kd, absMax, lastInput;
} pid_config_t;

float pid_update(pid_config_t *conf, float input, float setpoint, float modulus);
