#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "utils.h"
#include "defines.h"
#include "math.h"

// Converted from LightSensor.cpp

typedef struct {
    int32_t thresholdValue;
    gpio_num_t pin;

} light_sensor;

void ls_init_adc(void);
void ls_init(light_sensor *ls);
uint16_t ls_read(light_sensor *ls);
bool ls_on_white(light_sensor *ls);

typedef struct {
    float centre;
    int16_t length;
    int16_t leftSensor;
    int16_t rightSensor;
} ls_cluster;

void cluster_update_left_right(ls_cluster *cluster);
void cluster_update_length_centre(ls_cluster *cluster);
/** Adds two clusters, storing the result in cluster1 **/
void cluster_add_cluster(ls_cluster *cluster1, ls_cluster *cluster2);
void cluster_reset(ls_cluster *cluster);

static light_sensor *sensors[LS_NUM];
// TODO rename this
static bool data[LS_NUM];
bool filledInData[LS_NUM];
static ls_cluster cluster1, cluster2, cluster3;
extern gpio_num_t lsPins[LS_NUM];

void lsarray_init(void);
void lsarray_read(void);
void lsarray_calc_clusters();
void lsarray_fill_in_sensors();
void lsarray_reset_clusters(void);