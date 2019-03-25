#pragma once
#include "driver/i2c.h"
#include "defines.h"
#include "alloca.h"

typedef struct {
    uint16_t tsopAngle;
    uint16_t tsopStrength;
    uint16_t lineAngle;
    uint16_t lineSize;
} data_packet_t;

/** Real life slave. Sends sensor data to the master. AUTOMODE_MASTER. **/
void comms_i2c_init_master();
/** Real life master. Receives sensor data from the master. AUTOMODE_SLAVE. **/
void comms_i2c_init_slave();
/** Send a full data packet from the slave to the master **/
void comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize);