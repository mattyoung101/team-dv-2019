#pragma once
#include <stdint.h>
#include <stdlib.h>

// Moving average

typedef struct {
    size_t size;
    uint16_t counter;
    float *items;
} mov_avg_t;

mov_avg_t *mov_avg_create(size_t size);
void mov_avg_push(mov_avg_t *mov_avg, float value);
float mov_avg_calc(mov_avg_t *mov_avg);