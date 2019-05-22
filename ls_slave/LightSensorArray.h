#ifndef LIGHT_SENSOR_ARRAY_H
#define LIGHT_SENSOR_ARRAY_H

#include <Arduino.h>
#include "digitalWriteFast.h"
#define LS_NUM 48
#define DEBUG_DATA true
#define DEBUG_RAW false

// Array of light sensors
class LightSensorArray {
public:
    LightSensorArray() {}

    void init();

    void read();
    int readSensor(int sensor);
    void changeMUXChannel(uint8_t channel);

    void calculateClusters(bool doneFillInSensors = false);
    void fillInSensors();
    void calculateLine();
    void resetStartEnds();

    void calibrate();

    double getLineAngle();
    double getLineSize();

    bool data[LS_NUM]; // Array of if sensors see white or not
    bool filledInData[LS_NUM]; // Data after sensors are filled in (if an off sensor has two adjacent on sensors, it will be turned on)

    uint16_t thresholds[LS_NUM]; // Thresholds for each sensor. A sensor is on if reading > threshold

    int starts[4]; // Array of cluster start indexes
    int ends[4]; // Array of cluster end indexes

    int numClusters = 0; // Number of clusters found

private:
    void resetClusters();

    uint8_t muxChannels[32] = {12, 11, 10, 9, 13, 14, 15, 16, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 1, 2, 3, 4, 5, 6, 7, 8}; // Multiplexer channels for each sensor (5-36)

    double angle; // Line angle
    double size; // Line size
};

#endif // LIGHT_SENSOR_ARRAY_H

