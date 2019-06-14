#ifndef LIGHT_SENSOR_CLUSTER_H
#define LIGHT_SENSOR_CLUSTER_H

#include <Arduino.h>
#include <Utils.h>
#include <Config.h>

class LightSensorCluster {
public:
    LightSensorCluster() {}

    // These constructors are very similar. Be careful when calling!
    LightSensorCluster(double clusterCentre, int clusterLength);
    LightSensorCluster(int leftSensor, int rightSensor);

    void addCluster(LightSensorCluster cluster);

    void addSensorClockwise();

    int getLeftSensor();
    int getRightSensor();

    double getCentre();
    int getLength();

    double getAngle();
    double getLeftAngle();
    double getRightAngle();

private:
    double centre;
    int length;
    int leftSensor;
    int rightSensor;

    void updateLeftRight();
    void updateLengthCentre();
};

#endif // LIGHT_SENSOR_CLUSTER_H
