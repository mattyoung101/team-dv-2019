#pragma once
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"

/** Initialises the MPU and DMP **/
void mpuw_init();
/** Updates the DMP FIFO and calculates Euler angles **/
void mpuw_update();