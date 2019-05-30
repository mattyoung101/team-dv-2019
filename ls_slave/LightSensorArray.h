#ifndef LIGHT_SENSOR_ARRAY_H
#define LIGHT_SENSOR_ARRAY_H

#include <Arduino.h>
#define LS_NUM 48
#define DEBUG_DATA false
#define DEBUG_RAW false

#define LS_CALIBRATION_COUNT 10
#define LS_CALIBRATION_BUFFER 30
#define LS_ES_DEFAULT 69
#define NO_LINE_ANGLE 0xBAD
#define NO_LINE_SIZE 0xBAD
#define LS_NUM_MULTIPLIER 7.5 // 360 / LS_NUM

#define DEG_RAD 0.017453292519943295 // multiply to convert degrees to radians
#define RAD_DEG 57.29577951308232 // multiply to convert radians to degrees
    
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

    void updateLine(float angle, float size, float heading); // Function for updating variables
    void lineCalc(); // Function for line tracking

    bool isOnLine = false;
    bool lineOver = false;
    float lineAngle = NO_LINE_ANGLE;
    float lineSize = NO_LINE_SIZE;
    float firstAngle = NO_LINE_ANGLE;

    bool data[LS_NUM]; // Array of if sensors see white or not
    bool filledInData[LS_NUM]; // Data after sensors are filled in (if an off sensor has two adjacent on sensors, it will be turned on)

    uint16_t thresholds[LS_NUM]; // Thresholds for each sensor. A sensor is on if reading > threshold

    int starts[4]; // Array of cluster start indexes
    int ends[4]; // Array of cluster end indexes

    int numClusters = 0; // Number of clusters found

private:
    void resetClusters();

    // Index = LS num, value = mux binary
    // >= 24, next mux
    uint8_t muxLUT[LS_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, // mux 0
                              0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23}; // mux 1

    double angle; // Line angle
    double size; // Line size
};

#endif // LIGHT_SENSOR_ARRAY_H
