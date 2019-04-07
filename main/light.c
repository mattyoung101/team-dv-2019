#include "light.h"

static bool lsData[LS_NUM];
static const gpio_num_t lsPins[LS_NUM] = {LS_0, LS_1, LS_2, LS_3, LS_4, LS_5, LS_6, LS_7, LS_8, LS_9, LS_10, LS_11, LS_12, 
                            LS_13, LS_14, LS_15, LS_16, LS_17, LS_18, LS_19, LS_20, LS_21, LS_22, LS_23};
static ls_cluster cluster1, cluster2, cluster3;
static light_sensor *sensors[LS_NUM];
static bool filledInData[LS_NUM];

// This file contains the code that reads and processes the light sensors

////////// LIGHT SENSOR //////////
void ls_init_adc(void){
    // Teensy ADC is 10 bit
    adc1_config_width(ADC_WIDTH_BIT_10);
    // TODO do this for all channels on adc1
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
        // TODO ayyy fix this it's not channel 6 it'll be whatever the heck channel
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

void cluster_add_clockwise(ls_cluster *cluster){
    cluster->rightSensor = mod(cluster->rightSensor + 1, LS_NUM);
    cluster_update_length_centre(cluster);
}

float cluster_get_angle(ls_cluster *cluster){
    return cluster->centre / (float) LS_NUM * 360.0f;
}

float cluster_get_left_angle(ls_cluster *cluster){
    return cluster->leftSensor / (float) LS_NUM * 360.0f;

}

float cluster_get_right_angle(ls_cluster *cluster){
    return cluster->rightSensor / (float) LS_NUM * 360.0f;
}

// TODO update to latest LJStand code: http://bit.do/eKtCX

////////// LIGHT SENSOR ARRAY //////////
void lsarray_init(void){
    for (int i = 0; i < LS_NUM; i++){
        // cheeky malloc to stop it from being destroyed after going out of scope
        light_sensor *sensor = calloc(1, sizeof(light_sensor));
        sensor->pin = lsPins[i];
        ls_init(sensor);
        sensors[i] = sensor;
    }

    cluster_reset(&cluster1);
    cluster_reset(&cluster2);
    cluster_reset(&cluster3);
}

void lsarray_read(void){
    for (int i = 0; i < LS_NUM; i++){
        lsData[i] = ls_on_white(sensors[i]);
    }
}

void lsarray_fill_in_sensors(void){
    for (int i = 0; i < LS_NUM; i++) {
        filledInData[i] = lsData[i];

        if (!lsData[i] && lsData[mod(i - 1, LS_NUM)] && lsData[mod(i + 1, LS_NUM)]) {
            filledInData[i] = true;
        }
    }
}

void lsarray_reset_clusters(void){
    cluster_reset(&cluster1);
    cluster_reset(&cluster2);
    cluster_reset(&cluster3);
}

void lsarray_calc_clusters(void){
    lsarray_reset_clusters();

    bool cluster1Done = false;
    bool cluster2Done = false;
    bool cluster3Done = false;

    ls_cluster cluster4;

    // port note: in practice, doneFillInSensors is always false, so we just assume it will always be false
    // TODO also refactor this wtf
    for (int i = 0; i < LS_NUM; i++) {
        if (cluster1Done) {
            if (cluster2Done) {
                if (cluster3Done) {
                    if (lsData[i]) {
                        if (cluster4.length == 0) {
                            cluster4.centre = (float) i;
                            cluster4.length = 1;
                            cluster_update_left_right(&cluster4);
                        } else {
                            cluster_add_clockwise(&cluster4);
                        }

                        if (i == 23 && cluster1.leftSensor == 0) {
                            cluster_add_cluster(&cluster1, &cluster4);
                            cluster_reset(&cluster4);
                        }
                    } else {
                        if (cluster4.length != 0) {
                            lsarray_fill_in_sensors();
                            break;
                        }
                    }
                } else {
                    if (lsData[i]) {
                        if (cluster3.length == 0) {
                            cluster3.centre = (float) i;
                            cluster3.length = 1;
                            cluster_update_left_right(&cluster3);
                        } else {
                            cluster_add_clockwise(&cluster3);
                        }

                        if (i == 23 && cluster1.leftSensor == 0) {
                            //cluster1.addCluster(cluster3);
                            cluster_add_cluster(&cluster1, &cluster3);
                            //cluster3 = LightSensorCluster(0.0, 0);
                            cluster_reset(&cluster3);
                        }
                    } else {
                        if (cluster3.length != 0) {
                            cluster3Done = true;
                        }
                    }
                }
            } else {
                if (lsData[i]) {
                    if (cluster2.length == 0) {
                        //cluster2 = LightSensorCluster((double)i, 1);
                        cluster2.centre = (float) i;
                        cluster2.length = 1;
                        cluster_update_left_right(&cluster2);
                    } else {
                        //cluster2.addSensorClockwise();
                        cluster_add_clockwise(&cluster2);
                    }

                    if (i == 23 && cluster1.leftSensor == 0) {
                        //cluster1.addCluster(cluster2);
                        cluster_add_cluster(&cluster1, &cluster2);
                        // cluster2 = LightSensorCluster(0.0, 0);
                        cluster_reset(&cluster2);
                    }
                } else {
                    if (cluster2.length != 0) {
                        cluster2Done = true;
                    }
                }
            }
        } else {
            if (lsData[i]) {
                if (cluster1.length == 0) {
                    // cluster1 = LightSensorCluster((double)i, 1);
                    cluster2.centre = (float) i;
                    cluster2.length = 1;
                    cluster_update_left_right(&cluster2);
                } else {
                    cluster_add_clockwise(&cluster1);
                }
            } else {
                if (cluster1.length != 0) {
                    cluster1Done = true;
                }
            }
        }
    }

    numClusters = (uint8_t)(cluster1.length != 0) + (uint8_t)(cluster2.length != 0) + (uint8_t)(cluster3.length != 0);
}

void lsarray_calc_line(void){
    if (numClusters == 0){
        lineAngle = LS_NO_LINE_ANGLE;
        lineSize = LS_NO_LINE_SIZE;
    } else {
        float cluster1Angle = cluster_get_angle(&cluster1);
        float cluster2Angle = cluster_get_angle(&cluster2);
        float cluster3Angle = cluster_get_angle(&cluster3);

        // TODO refactor this, good lord
        if (numClusters == 1){
            lineAngle = cluster1Angle;
            lineSize = 1.0f - cosf(DEG_RAD * (angleBetween(cluster_get_left_angle(&cluster1), cluster_get_right_angle(&cluster1) / 2.0f)));
        } else if (numClusters == 2){
            lineAngle = angleBetween(cluster1Angle, cluster2Angle) <= 180 ? midAngleBetween(cluster1Angle, cluster2Angle) : midAngleBetween(cluster2Angle, cluster1Angle);
            lineSize = 1.0f - cosf(DEG_RAD * (angleBetween(cluster1Angle, cluster2Angle) <= 180 ? angleBetween(cluster1Angle, cluster2Angle) / 2.0 : angleBetween(cluster2Angle, cluster1Angle) / 2.0));        
        } else {
            float angleDiff12 = angleBetween(cluster1Angle, cluster2Angle);
            float angleDiff23 = angleBetween(cluster2Angle, cluster3Angle);
            float angleDiff31 = angleBetween(cluster3Angle, cluster1Angle);
            float biggestAngle = fmaxf(angleDiff12, fmaxf(angleDiff23, angleDiff31));

            if (angleDiff12 == biggestAngle) {
                lineAngle = midAngleBetween(cluster2Angle, cluster1Angle);
                lineSize = angleBetween(cluster2Angle, cluster1Angle) <= 180 ? 1 - cosf(DEG_RAD * (angleBetween(cluster2Angle, cluster1Angle) / 2.0)) : 1;
            } else if (angleDiff23 == biggestAngle) {
                lineAngle = midAngleBetween(cluster3Angle, cluster2Angle);
                lineSize = angleBetween(cluster3Angle, cluster2Angle) <= 180 ? 1 - cosf(DEG_RAD * (angleBetween(cluster3Angle, cluster2Angle) / 2.0)) : 1;
            } else {
                lineAngle = midAngleBetween(cluster1Angle, cluster3Angle);
                lineSize = angleBetween(cluster1Angle, cluster3Angle) <= 180 ? 1 - cosf(DEG_RAD * (angleBetween(cluster1Angle, cluster3Angle) / 2.0)) : 1;
            }
        }
    }
}