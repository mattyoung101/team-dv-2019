#include "light.h"

////////// LIGHT SENSOR //////////
void ls_init_adc(void){
    // Teensy ADC is 10 bit
    adc1_config_width(ADC_WIDTH_BIT_12);
    // TODO what does this do? (copied from examples) + verify which bus we're using
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_0); // channel 6, atten_db_0
}

void ls_init(light_sensor *ls){
    gpio_set_direction(ls->pin, GPIO_MODE_INPUT);

    // calibrate the sensor
    float defaultValue = ls_read(ls);
    for (int i = 0; i < LS_CALIBRATION_COUNT; i++){
        defaultValue += ls_read(ls);
    }

    ls->thresholdValue = roundf((defaultValue / LS_CALIBRATION_COUNT) + LS_CALIBRATION_BUFFER);
}

uint16_t ls_read(light_sensor *ls){
    // multi-sampled ADC read, lower samples for increased speed
    uint32_t reading = 0;

    for (int i = 0; i < ADC_SAMPLES; i++){
        reading += adc1_get_raw(ADC_CHANNEL_6);
    }

    return reading / ADC_SAMPLES;
}

bool ls_on_white(light_sensor *ls){
    return ls_read(ls) > ls->thresholdValue;
}

///////// CLUSTER ////////
void cluster_update_left_right(ls_cluster *cluster){
    cluster->leftSensor = mod(cluster->centre - ((cluster->length - 1) / 2.0f), LS_NUM);
    cluster->rightSensor = mod(cluster->centre + ((cluster->length - 1) / 2.0f), LS_NUM);
}

void cluster_update_length_centre(ls_cluster *cluster){
    if (cluster->leftSensor > cluster->rightSensor) {
        cluster->centre = floatMod((-(LS_NUM - cluster->leftSensor) + cluster->rightSensor) / 2.0, LS_NUM);
        cluster->length = (LS_NUM - (cluster->leftSensor - cluster->rightSensor)) + 1;
    } else {
        cluster->centre = (cluster->leftSensor + cluster->rightSensor) / 2.0;
        cluster->length = (cluster->rightSensor - cluster->leftSensor) + 1;
    }
}

void cluster_add_cluster(ls_cluster *cluster1, ls_cluster *cluster2){
    int16_t leftOther = cluster2->leftSensor;
    int16_t rightOther = cluster2->rightSensor;

    if (mod(cluster1->rightSensor + 1, LS_NUM) == leftOther) {
        cluster1->rightSensor = rightOther;
    } else if (mod(rightOther + 1, LS_NUM) == cluster1->leftSensor) {
        cluster1->leftSensor = leftOther;
    } else {
        // We're adding two non-adjacent clusters so we just do not apply any changes to the cluster.
        return;
    }

    cluster_update_length_centre(cluster1);
}

void cluster_reset(ls_cluster *cluster){
    cluster->centre = 0.0f;
    cluster->length = 0;

    cluster_update_left_right(cluster);
}

////////// LIGHT SENSOR ARRAY //////////
gpio_num_t lsPins[LS_NUM] = {LS_0, LS_1, LS_2, LS_3, LS_4, LS_5, LS_6, LS_7, LS_8, LS_9, LS_10, LS_11, LS_12, 
                            LS_13, LS_14, LS_15, LS_16, LS_17, LS_18, LS_19, LS_20, LS_21, LS_22, LS_23};

void lsarray_init(void){
    for (int i = 0; i < LS_NUM; i++){
        light_sensor* sensor = (light_sensor*) malloc(sizeof(light_sensor));
        // TODO add the pins
        sensor->pin = lsPins[i];
        ls_init(sensors[i]);
    }

    cluster_reset(&cluster1);
    cluster_reset(&cluster2);
    cluster_reset(&cluster3);
}

void lsarray_read(void){
    for (int i = 0; i < LS_NUM; i++){
        data[i] = ls_on_white(sensors[i]);
    }
}

// in real code, doneFillInSensors is always false, so we just assume it will always be false
void lsarray_calc_clusters(){
    lsarray_reset_clusters();

    bool cluster1Done = false;
    bool cluster2Done = false;
    bool cluster3Done = false;

    ls_cluster cluster4;
}

void lsarray_reset_clusters(void){
    cluster_reset(&cluster1);
    cluster_reset(&cluster2);
    cluster_reset(&cluster3);
}