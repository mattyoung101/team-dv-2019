#pragma once
#include "esp_timer.h"

// Ported from PID.cpp 

typedef struct {
    float ki;
    float kp;
    float kd;
    float absMax;
} pid_config;

int64_t lastTime;
float integral;
float lastInput;

float pid_update(pid_config* conf, float input, float setpoint, float modulus);