#include "LightSensor.h"

LightSensor::LightSensor(int in) {
    inPin = in;
}

void LightSensor::init() {
    pinMode(inPin, INPUT);

    // Auto calibrate
    read();
    int defaultValue = value;

    for (int i = 0; i < LS_CALIBRATION_COUNT; i++) {
        read();
        defaultValue += value;
    }

    thresholdValue = round((int)((double)defaultValue / LS_CALIBRATION_COUNT) + LS_CALIBRATION_BUFFER);
}

void LightSensor::read() {
    value = analogRead(inPin);
}

bool LightSensor::isOnWhite() {
    read();
    return (value > thresholdValue);
}

int LightSensor::getValue() {
    return value;
}
