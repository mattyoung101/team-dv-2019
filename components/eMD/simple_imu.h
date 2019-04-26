#pragma once
#include "defines.h"
#include "esp_utils.h"

typedef struct {
    float x, y, z;
} vec3d_t;

extern float heading;

void simu_init(void);
void simu_read_accel(void);
vec3d_t simu_read_gyro(void);
void simu_read_mag(void);
void simu_calibrate(void);
void simu_calc(void);