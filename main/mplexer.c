#include "mplexer.h"

// Index = multiplexer pin, Value = TSOP number
// static const gpio_num_t irTable[] = {1, 2, 3, 10, 9, 8, 7, 6, 0, 17, 16, 15, 14, 13, 12, 11};

void mplexer_4bit_init(mplexer_4bit_t *config){
    gpio_set_direction(config->s0, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s1, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s2, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s3, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->out, GPIO_MODE_INPUT);
}

uint8_t mplexer_4bit_read(mplexer_4bit_t *plexer, uint8_t pin){
    // convert pin to binary number
    uint8_t binary[4];
    int index = 0;
    for (int i = 3; i >= 0; i--){
        uint8_t k = pin >> (uint8_t) i;
        if (k & 1){
            binary[index++] = 1;
        } else {
            binary[index++] = 0;
        }
    }

    ESP_LOGV("Multiplexer", "Pin %d, Binary: %d, %d, %d, %d", pin, binary[0], binary[1], binary[2], binary[3]);

    // tell the multiplexer we want to access the pin we selected by writing out the binary
    // en must be low in order for it do stuff
    gpio_set_level(plexer->s0, binary[0]);
    gpio_set_level(plexer->s1, binary[1]);
    gpio_set_level(plexer->s2, binary[2]);
    gpio_set_level(plexer->s3, binary[3]);
    
    return gpio_get_level(plexer->out);
}