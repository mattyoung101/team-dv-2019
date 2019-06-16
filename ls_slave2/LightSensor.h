#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <Arduino.h>
#include "Config.h"

class LightSensor {
public:
    LightSensor() {}
    LightSensor(int in);
    void init();

    void read();
    bool isOnWhite();
    int getValue();

private:
    int value;
    int inPin;
    int thresholdValue;
};

#endif // LIGHT_SENSOR_H
