#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include <math.h>
#include "defines.h"
#include "utils.h"

uint16_t tsopValues[TSOP_NUM];
uint16_t tsopSortedValues[TSOP_NUM];
float tsopAngle;
float tsopStrength;

void tsop_init(void);
void tsop_update_once(void);
void tsop_read(void);
void tsop_calc(uint8_t n);