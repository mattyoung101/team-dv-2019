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
