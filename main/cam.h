#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include "defines.h"
#include "esp_err.h"
#include <math.h>
#include "utils.h"
#include <string.h>
#include <alloca.h>

typedef struct {
    /** if false: the values of x, y, angle and length are undefined **/
    bool exists;
    int16_t x;
    int16_t y;
    int16_t angle;
    int16_t length;
} cam_goal;

cam_goal goalBlue;
cam_goal goalYellow;
int16_t robotX;
int16_t robotY;

void cam_init(void);
void cam_update(void);
void cam_calc(void);