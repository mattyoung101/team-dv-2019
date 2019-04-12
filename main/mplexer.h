#pragma once
#include "driver/gpio.h"
#include "defines.h"

typedef struct {
    gpio_num_t s0, s1, s2, s3, out;
} mplexer_4bit_t;

/** Initialises the pins on the multiplexer **/
void mplexer_4bit_init(mplexer_4bit_t *config);
/** Reads the Nth pin of a 4 bit multiplexer **/
uint32_t mplexer_4bit_read(mplexer_4bit_t *plexer, uint8_t pin);

////////////////////////////////////////////////////////////////

typedef struct {
    gpio_num_t s0, s1, s2, s3, s4, out, en;
} mplexer_5bit_t;

/** Initialises the pins on the multiplexer **/
void mplexer_5bit_init(mplexer_5bit_t *config);
/** Dials in the pin to the multiplexer **/
void mplexer_5bit_select(mplexer_5bit_t *plexer, uint8_t pin);