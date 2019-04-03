#pragma once
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include "defines.h"
#include "rom/ets_sys.h"
#include "utils.h"

int esp_get_clock_ms(unsigned long *count);
int esp_delay_ms(unsigned long num_ms);
int esp_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * data);
int esp_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * data);