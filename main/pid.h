#pragma once
#include "esp_timer.h"
#include "utils.h"

// Ported from PID.cpp 

typedef struct {
    float kp;
    float ki;
    float kd;
    float absMax;
} pid_config_t;

int64_t lastTime;
float integral;
float lastInput;

float pid_update(pid_config_t* conf, float input, float setpoint, float modulus);