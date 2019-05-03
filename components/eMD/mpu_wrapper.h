#pragma once
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include <math.h>
#include "defines.h"

typedef struct {
    float pitch, roll, yaw;
} euler_angles_t;

extern euler_angles_t eulerAngles;
extern float mpuYaw;
extern float mpuMagYaw;

/** Initialises the MPU and DMP **/
void mpuw_init();
/** Updates the DMP FIFO and calculates Euler angles **/
void mpuw_update();
/** Calibrates the magnetometer **/
void mpuw_mag_calibrate();