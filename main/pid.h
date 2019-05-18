#pragma once
#include "esp_timer.h"

// Ported from PID.cpp 

#ifndef constrain
    // So don't get me wrong, C is a decent fucking language right
    // But holy fucking shit, I fucking hate it's god awful fucking include system like seriously what the fuck
    // This is the type of shit I have to do to get around it I mean look at this shit
    #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

typedef struct {
    float kp, ki, kd, absMax, lastInput;
} pid_config_t;

float pid_update(pid_config_t *conf, float input, float setpoint, float modulus);