#pragma once
#include "defines.h"
#include "esp_utils.h"

#define IMU_CALIBRATION_COUNT 15
#define IMU_CALIBRATION_TIME 200
#define IMU_THRESHOLD 1000
#define IMU_MULTIPLIER 100.0f

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

typedef struct {
    float x, y, z;
} vec3d_t;

extern float heading;

void simu_init();
void simu_read_accel();
vec3d_t simu_read_gyro();
void simu_read_mag();
void simu_calibrate();
void simu_calc();