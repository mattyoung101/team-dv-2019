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
#include "i2c.pb.h"

// Handles slave to master communication over I2C

typedef struct {
    uint16_t tsopAngle;
    uint16_t tsopStrength;
    uint16_t lineAngle;
    uint16_t lineSize;
    uint16_t heading;
} i2c_data_t;

typedef struct {
    float lineAngle;
    float lineSize;
    bool isOnLine;
    bool isLineOver;
    float lastAngle;
    float batteryVoltage;
} nano_data_t;

/** last SensorUpdate protobuf message from slave **/
extern SensorUpdate lastSensorUpdate;
/** data received from Nano **/
extern nano_data_t nanoData;
/** Protobuf semaphore **/
extern SemaphoreHandle_t pbSem;
/** Nano data semaphore **/
extern SemaphoreHandle_t nanoDataSem;

/** Writes a Protobuf message to I2C, by first writing the header then the byte stream. **/
int comms_i2c_write_protobuf(uint8_t *buf, size_t msgSize, uint8_t msgId);
/** Init real life slave. Sends sensor data to the master. AUTOMODE_SLAVE. **/
void comms_i2c_init_master(i2c_port_t port);
/** Init real life master. Receives sensor data from the slave. AUTOMODE_MASTER. **/
void comms_i2c_init_slave(void);