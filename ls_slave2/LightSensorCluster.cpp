#include "LightSensorCluster.h"

LightSensorCluster::LightSensorCluster(double clusterCentre, int clusterLength) {
    centre = clusterCentre;
    length = clusterLength;

    updateLeftRight();
}

LightSensorCluster::LightSensorCluster(int left, int right) {
    leftSensor = left;
    rightSensor = right;

    updateLengthCentre();
}

void LightSensorCluster::addSensorClockwise() {
    rightSensor = mod(rightSensor + 1, LS_NUM);

    updateLengthCentre();
}

void LightSensorCluster::addCluster(LightSensorCluster cluster) {
    int leftOther = cluster.getLeftSensor();
    int rightOther = cluster.getRightSensor();

    if (mod(rightSensor + 1, LS_NUM) == leftOther) {
        rightSensor = rightOther;
    } else if (mod(rightOther + 1, LS_NUM) == leftSensor) {
        leftSensor = leftOther;
    } else {
        // We're adding two non-adjacent clusters so we just do not apply any changes to the cluster.
        return;
    }

    updateLengthCentre();
}

void LightSensorCluster::updateLeftRight() {
    leftSensor = mod(centre - ((length - 1) / 2.0), LS_NUM);
    rightSensor = mod(centre + ((length - 1) / 2.0), LS_NUM);
}

void LightSensorCluster::updateLengthCentre() {
    if (leftSensor > rightSensor) {
        centre = doubleMod((-(LS_NUM - leftSensor) + rightSensor) / 2.0, LS_NUM);
        length = (LS_NUM - (leftSensor - rightSensor)) + 1;
    } else {
        centre = (leftSensor + rightSensor) / 2.0;
        length = (rightSensor - leftSensor) + 1;
    }
}

int LightSensorCluster::getLeftSensor() {
    // Note: left is the counterclockwise boundary because its easier to write.
    return leftSensor;
}

int LightSensorCluster::getRightSensor() {
    // Note: right is the clockwise boundary because its easier to write.
    return rightSensor;
}

double LightSensorCluster::getCentre() {
    return centre;
}

int LightSensorCluster::getLength() {
    return length;
}

double LightSensorCluster::getAngle() {
    return centre / (double)LS_NUM * 360;
}

double LightSensorCluster::getLeftAngle() {
    return leftSensor / (double)LS_NUM * 360;
}

double LightSensorCluster::getRightAngle() {
    return rightSensor / (double)LS_NUM * 360;
}
