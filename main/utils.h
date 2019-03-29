#pragma once
#include "driver/adc.h"
#include "defines.h"
#include <stdint.h>
#include <math.h>
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define ARRAYSHIFTDOWN(a, lower, upper){          \
    if (upper == (sizeof(a)/sizeof(a[0])) - 1){   \
        for (int q = upper - 1; q >= lower; q--){ \
            *(a + q + 1) = *(a + q); }            \
    } else {                                      \
        for (int q = upper; q >= lower; q--){     \
            *(a + q + 1) = *(a + q); }}}

/** first 8 bits of unsigned 16 bit int **/
#define HIGH_BYTE_16(num) ((uint8_t) ((num >> 8) & 0xF))
/** second 8 bits of unsigned 16 bit int **/
#define LOW_BYTE_16(num)  ((uint8_t) ((num & 0xFF)))
/** unpack two 8 bit integers into a 16 bit integer **/
#define UNPACK_16(a, b) ((uint16_t) ((a << 8) | b))

int32_t mod(int32_t x, int32_t m);
float floatMod(float x, float m);
int number_comparator_descending(const void *a, const void *b);
float angleBetween(float angleCounterClockwise, float angleClockwise);
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);