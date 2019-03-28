#pragma once
#include "driver/gpio.h"
#include "defines.h"

typedef struct {
    gpio_num_t s0, s1, s2, s3, out;
} mplexer_4bit_t;

/** Initialises the pins on the multiplexer **/
void mplexer_4bit_init(mplexer_4bit_t *config);
/** Reads the Nth pin of a 4 bit multiplexer **/
uint8_t mplexer_4bit_read(mplexer_4bit_t *plexer, uint8_t pin);
/** Resolves IR number to multiplexer pin or straight to the ESP32 if required **/
gpio_num_t mplexer_ir_resolve(uint8_t num);