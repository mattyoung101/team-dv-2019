#include "LightSensorArray.h"

void LightSensorArray::init() {
    for (int i = 0; i < LS_NUM; i++) {
      if (i == 12) continue;
        sensors[i] = LightSensor(lsPins[i]);
        sensors[i].init();
    }
}

void LightSensorArray::read() {
    for (int i = 0; i < LS_NUM; i++) {
      if(i == 12) continue;
        data[i] = sensors[i].isOnWhite();
        sensors[i].read();
        // Serial.print(sensors[i].getValue());
        // Serial.print(" ");
        // Serial.printf("%d ", data[i]);
    }
    // Serial.println();
}

void LightSensorArray::calculateClusters(bool doneFillInSensors) {
    bool *lightData = !doneFillInSensors ? data : filledInData;

    resetClusters();

    bool cluster1Done = false;
    bool cluster2Done = false;
    bool cluster3Done = false;

    LightSensorCluster cluster4 = LightSensorCluster(0.0, 0);

    for (int i = 0; i < LS_NUM; i++) {
        if (cluster1Done) {
            if (cluster2Done) {
                if (cluster3Done) {
                    if (lightData[i]) {
                        if (cluster4.getLength() == 0) {
                            cluster4 = LightSensorCluster((double)i, 1);
                        } else {
                            cluster4.addSensorClockwise();
                        }

                        if (i == 23 && cluster1.getLeftSensor() == 0) {
                            cluster1.addCluster(cluster4);
                            cluster4 = LightSensorCluster(0.0, 0);
                        }
                    } else {
                        if (cluster4.getLength() != 0) {
                            if (!doneFillInSensors) {
                                fillInSensors();
                            } else {
                                resetClusters();
                            }

                            break;
                        }
                    }
                } else {
                    if (lightData[i]) {
                        if (cluster3.getLength() == 0) {
                            cluster3 = LightSensorCluster((double)i, 1);
                        } else {
                            cluster3.addSensorClockwise();
                        }

                        if (i == 23 && cluster1.getLeftSensor() == 0) {
                            cluster1.addCluster(cluster3);
                            cluster3 = LightSensorCluster(0.0, 0);
                        }
                    } else {
                        if (cluster3.getLength() != 0) {
                            cluster3Done = true;
                        }
                    }
                }
            } else {
                if (lightData[i]) {
                    if (cluster2.getLength() == 0) {
                        cluster2 = LightSensorCluster((double)i, 1);
                    } else {
                        cluster2.addSensorClockwise();
                    }

                    if (i == 23 && cluster1.getLeftSensor() == 0) {
                        cluster1.addCluster(cluster2);
                        cluster2 = LightSensorCluster(0.0, 0);
                    }
                } else {
                    if (cluster2.getLength() != 0) {
                        cluster2Done = true;
                    }
                }
            }
        } else {
            if (lightData[i]) {
                if (cluster1.getLength() == 0) {
                    cluster1 = LightSensorCluster((double)i, 1);
                } else {
                    cluster1.addSensorClockwise();
                }
            } else {
                if (cluster1.getLength() != 0) {
                    cluster1Done = true;
                }
            }
        }
    }

    numClusters = (int)(cluster1.getLength() != 0) + (int)(cluster2.getLength() != 0) + (int)(cluster3.getLength() != 0);
}

void LightSensorArray::fillInSensors() {
    for (int i = 0; i < LS_NUM; i++) {
        filledInData[i] = data[i];

        if (!data[i] && data[mod(i - 1, LS_NUM)] && data[mod(i + 1, LS_NUM)]) {
            filledInData[i] = true;
        }
    }

    calculateClusters(true);
}

void LightSensorArray::calculateLine() {
    if (numClusters == 0) {
        angle = NO_LINE_ANGLE;
        size = NO_LINE_SIZE;
    } else {
        double cluster1Angle = cluster1.getAngle();
        double cluster2Angle = cluster2.getAngle();
        double cluster3Angle = cluster3.getAngle();

        if (numClusters == 1) {
            angle = cluster1Angle;
            size = 1 - cos(degreesToRadians(angleBetween(cluster1.getLeftAngle(), cluster1.getRightAngle()) / 2.0));
        } else if (numClusters == 2) {
            angle = angleBetween(cluster1Angle, cluster2Angle) <= 180 ? midAngleBetween(cluster1Angle, cluster2Angle) : midAngleBetween(cluster2Angle, cluster1Angle);
            size = 1 - cos(degreesToRadians(angleBetween(cluster1Angle, cluster2Angle) <= 180 ? angleBetween(cluster1Angle, cluster2Angle) / 2.0 : angleBetween(cluster2Angle, cluster1Angle) / 2.0));
        } else {
            double angleDiff12 = angleBetween(cluster1Angle, cluster2Angle);
            double angleDiff23 = angleBetween(cluster2Angle, cluster3Angle);
            double angleDiff31 = angleBetween(cluster3Angle, cluster1Angle);

            double biggestAngle = max(angleDiff12, max(angleDiff23, angleDiff31));

            if (angleDiff12 == biggestAngle) {
                angle = midAngleBetween(cluster2Angle, cluster1Angle);
                size = angleBetween(cluster2Angle, cluster1Angle) <= 180 ? 1 - cos(degreesToRadians(angleBetween(cluster2Angle, cluster1Angle) / 2.0)) : 1;
            } else if (angleDiff23 == biggestAngle) {
                angle = midAngleBetween(cluster3Angle, cluster2Angle);
                size = angleBetween(cluster3Angle, cluster2Angle) <= 180 ? 1 - cos(degreesToRadians(angleBetween(cluster3Angle, cluster2Angle) / 2.0)) : 1;
            } else {
                angle = midAngleBetween(cluster1Angle, cluster3Angle);
                size = angleBetween(cluster1Angle, cluster3Angle) <= 180 ? 1 - cos(degreesToRadians(angleBetween(cluster1Angle, cluster3Angle) / 2.0)) : 1;
            }
        }
    }
}

void LightSensorArray::resetClusters() {
    cluster1 = LightSensorCluster(0.0, 0);
    cluster2 = LightSensorCluster(0.0, 0);
    cluster3 = LightSensorCluster(0.0, 0);
}

double LightSensorArray::getLineAngle() {
    return angle;
}

double LightSensorArray::getLineSize() {
    return size;
}
