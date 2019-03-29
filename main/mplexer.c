#include "mplexer.h"

void mplexer_4bit_init(mplexer_4bit_t *config){
    gpio_set_direction(config->s0, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s1, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s2, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s3, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->out, GPIO_MODE_INPUT);
}

uint32_t mplexer_4bit_read(mplexer_4bit_t *plexer, uint8_t pin){
    // convert pin to binary number
    uint32_t binary[4];
    int index = 0;
    for (int i = 3; i >= 0; i--){
        uint32_t k = pin >> (uint32_t) i;
        if (k & 1){
            binary[index++] = 1;
        } else {
            binary[index++] = 0;
        }
    }

    ESP_LOGD("Multiplexer", "Pin: %d, Binary: %d, %d, %d, %d", pin, binary[0], binary[1], binary[2], binary[3]);

    // tell the multiplexer we want to access the pin we selected by writing out the binary
    // en must be low in order for it do stuff
    gpio_set_level(plexer->s0, binary[0]);
    gpio_set_level(plexer->s1, binary[1]);
    gpio_set_level(plexer->s2, binary[2]);
    gpio_set_level(plexer->s3, binary[3]);
    
    return gpio_get_level(plexer->out);
}