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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_task_wdt.h"

#define CAM_BUF_SIZE 8

typedef struct {
    /** if false: the values of x, y, angle and length are undefined **/
    bool exists;
    int16_t x;
    int16_t y;
    int16_t angle;
    float length;
    int16_t distance;
} cam_goal;

extern SemaphoreHandle_t goalDataSem;

extern cam_goal goalBlue;
extern cam_goal goalYellow;
extern int16_t robotX;
extern int16_t robotY;

void cam_init(void);
void cam_update(void);
void cam_calc(void);