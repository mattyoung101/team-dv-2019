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
#include "ads1015.h"

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

// --- Clusters --- //

typedef struct {
    float centre;
    int16_t length;
    int16_t leftSensor;
    int16_t rightSensor;
} ls_cluster;

// TODO add more docs

void cluster_update_left_right(ls_cluster *cluster);
void cluster_update_length_centre(ls_cluster *cluster);
/** Adds two clusters, storing the result in cluster1 **/
void cluster_add_cluster(ls_cluster *cluster1, ls_cluster *cluster2);
void cluster_reset(ls_cluster *cluster);
void cluster_add_clockwise(ls_cluster *cluster);
float cluster_get_angle(ls_cluster *cluster);
float cluster_get_left_angle(ls_cluster *cluster);
float cluster_get_right_angle(ls_cluster *cluster);

// --- Line Calcs --- //

/** Reads all light sensors **/
void lsarray_read(void);
/** Calculates the line from the stored readings using vectors **/
void lsarray_calc_vec(void);
/** Line calculations with cluster method **/
void lsarray_calc_clusters(void);
void lsarray_fill_in_sensors(void);
void lsarray_calc_line(void);
void lsarray_reset_clusters(void);
/** Prints debug values to be read by the script ls_debug.py **/
void lsarray_debug(void);

static light_sensor *sensors[LS_NUM];
bool filledInData[LS_NUM];
uint8_t numClusters;