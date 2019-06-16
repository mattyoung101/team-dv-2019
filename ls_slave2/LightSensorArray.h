#ifndef LIGHT_SENSOR_ARRAY_H
#define LIGHT_SENSOR_ARRAY_H

#include <Arduino.h>
#include <Pinlist.h>
#include <LightSensor.h>
#include <LightSensorCluster.h>
#include <Utils.h>

class LightSensorArray {
public:

    void init();

    void read();

    void calculateClusters(bool doneFillInSensors = false);
    void fillInSensors();
    void calculateLine();

    double getLineAngle();
    double getLineSize();

    LightSensor sensors[LS_NUM];
    bool data[LS_NUM];
    bool filledInData[LS_NUM];

    LightSensorCluster cluster1 = LightSensorCluster(0.0, 0);
    LightSensorCluster cluster2 = LightSensorCluster(0.0, 0);
    LightSensorCluster cluster3 = LightSensorCluster(0.0, 0);

    int numClusters = 0;

private:
    void resetClusters();

    int lsPins[LS_NUM] = {LS_0, LS_1, LS_2, LS_3, LS_4, LS_5, LS_6, LS_7, LS_8, LS_9, LS_10, LS_11, LS_12, LS_13, LS_14, LS_15, LS_16, LS_17, LS_18, LS_19, LS_20, LS_21, LS_22, LS_23};

    double angle;
    double size;
};

#endif // LIGHT_SENSOR_ARRAY_H
