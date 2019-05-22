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

// Ported and adapted from LJStand's 2018 LightSensorArray.cpp

// --- Light Sensors --- //

typedef struct {
    int32_t thresholdValue;
    gpio_num_t pin;
} light_sensor;

// Variables for line avoidance
extern float lineAngle;
extern float lineSize;
extern float lastAngle;
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

// --- Line Calcs --- //

/** Reads all light sensors **/
void lsarray_read(void);
/** Calculates the line from the stored readings using vectors **/
void lsarray_calc_vec(void);
/** Line calculations with cluster method **/
void lsarray_calc_clusters(bool doneFillInSensors);
void lsarray_calc_line(void);
void lsarray_reset_clusters(void);
/** Prints debug values to be read by the script ls_debug.py **/
void lsarray_debug(void);

uint8_t numClusters;