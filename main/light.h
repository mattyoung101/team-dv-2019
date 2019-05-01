#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "utils.h"
#include "defines.h"
#include "math.h"
#include "mplexer.h"
#include "esp_adc_cal.h"

// Converted from LightSensor.cpp

typedef struct {
    int32_t thresholdValue;
    gpio_num_t pin;
} light_sensor;

extern float lineAngle = LS_NO_LINE_ANGLE;
extern float lineSize = 0;
extern float lastAngle = LS_NO_LINE_ANGLE;
extern bool isOnLine;
extern bool lineOver;

/** Light sensor func. Receives light sensor and int on which mux its on (0 or 1) **/
typedef void (*ls_func_t)(light_sensor*, uint8_t);

/** Initialises the ADC for light sensor usage **/
void ls_init(void);
/** Reads the value of a light sensor on the given mux **/
uint16_t ls_read(uint8_t mux);
/** Iterates through all the light sensors and calls the provided ls_func appropriately **/
void ls_iterate(ls_func_t func);

/** Reads all light sensors **/
void lsarray_read(void);
/** Calculates the line from the stored readings **/
void lsarray_calc(void);
/** Prints debug values to be read by the script ls_debug.py **/
void lsarray_debug(void);