#pragma once
#include "driver/adc.h"
#include "defines.h"
#include <stdint.h>
#include <math.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

int32_t mod(int32_t x, int32_t m);
float floatMod(float x, float m);