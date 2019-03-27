#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include <math.h>
#include "defines.h"
#include "utils.h"
#include "mplexer.h"

uint16_t tsopValues[TSOP_NUM];
uint16_t tsopSortedValues[TSOP_NUM];
float tsopAngle;
float tsopStrength;

void tsop_init(void);
void tsop_update(void);
void tsop_process(void);
void tsop_calc(uint8_t n);