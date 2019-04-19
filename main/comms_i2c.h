#pragma once
#include "driver/i2c.h"
#include "defines.h"
#include "alloca.h"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "string.h"
#include "esp_task_wdt.h"

// Handles slave to master communication over I2C

typedef struct {
    uint16_t tsopAngle;
    uint16_t tsopStrength;
    uint16_t lineAngle;
    uint16_t lineSize;
    uint16_t heading;
} i2c_data_t;

/** data received over I2C **/
extern i2c_data_t receivedData;
/** received data semaphore for the variable receivedData **/
extern SemaphoreHandle_t rdSem;

/** Real life slave. Sends sensor data to the master. AUTOMODE_SLAVE. **/
void comms_i2c_init_master(i2c_port_t port);
/** Real life master. Receives sensor data from the master. AUTOMODE_MASTER. **/
void comms_i2c_init_slave();
/** Send a full data packet from the slave to the master **/
int comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize, uint16_t heading);