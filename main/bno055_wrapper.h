#pragma once
#include "bno055.h"
#include "defines.h"
#include "utils.h"
#include "driver/i2c.h"

// Contains code to interface the BNO055 driver from Bosch with the ESP32-IDF
// Source: https://github.com/Yuggo1/ESP32_BNO055/blob/master/main/i2c_example_main.c

#define BNO_GET_FUCKED_FUCK_SHIT_FUCK_FUCK_SHIT_FUCK 69

s8 bno_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void bno_delay(u32 ms);