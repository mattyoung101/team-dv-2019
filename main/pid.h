#pragma once
#include "esp_timer.h"
#include "utils.h"

// Ported from PID.cpp 

typedef struct {
    float kp, ki, kd, absMax;
} pid_config_t;

float pid_update(pid_config_t* conf, float input, float setpoint, float modulus);