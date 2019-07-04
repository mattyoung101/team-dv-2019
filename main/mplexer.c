#include "mplexer.h"
#include "utils.h"

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

    ESP_LOGV("Mux_4bit", "Pin: %d, Binary: %d%d%d%d", pin, binary[3], binary[2], binary[1], binary[0]);

    // tell the multiplexer we want to access the pin we selected by writing out the binary
    // reverse order is required as discovered by Ethan (thanks!)
    gpio_set_level(plexer->s0, binary[3]);
    gpio_set_level(plexer->s1, binary[2]);
    gpio_set_level(plexer->s2, binary[1]);
    gpio_set_level(plexer->s3, binary[0]);
    
    return gpio_get_level(plexer->out);
}

void mplexer_5bit_init(mplexer_5bit_t *config){
    gpio_set_direction(config->s0, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s1, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s2, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s3, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->s4, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->out, GPIO_MODE_INPUT);
    gpio_set_direction(config->en, GPIO_MODE_OUTPUT);
    gpio_set_direction(config->wr, GPIO_MODE_OUTPUT);

    gpio_set_level(config->en, 0);
    gpio_set_level(config->wr, 0);
}

// fast functions
static inline void set_pin_gmux0(gpio_num_t pin, bool value){
    if (value){
        GPIO.out_w1ts = (1 << pin);
    } else {
        GPIO.out_w1tc = (1 << pin);
    }
}

static inline void set_pin_gmux1(gpio_num_t pin, bool value){
    if (value){
        GPIO.out1_w1ts.data = (1 << (pin - 32));
    } else {
        GPIO.out1_w1tc.data = (1 << (pin - 32));
    }
}

inline void mplexer_5bit_select(mplexer_5bit_t *plexer, uint8_t pin){
    // convert pin to binary number
    uint32_t binary[5];
    int index = 0;
    for (int i = 4; i >= 0; i--){
        uint32_t k = pin >> (uint32_t) i;
        if (k & 1){
            binary[index++] = 1;
        } else {
            binary[index++] = 0;
        }
    }
    
    // gpio_set_level(plexer->s0, binary[4]);
    // gpio_set_level(plexer->s1, binary[3]);
    // gpio_set_level(plexer->s2, binary[2]);
    // gpio_set_level(plexer->s3, binary[1]);
    // gpio_set_level(plexer->s4, binary[0]);

    set_pin_gmux0(TSOP_MUX_S0, binary[4]);
    set_pin_gmux0(TSOP_MUX_S1, binary[3]);
    set_pin_gmux0(TSOP_MUX_S2, binary[2]);
    set_pin_gmux0(TSOP_MUX_S3, binary[1]);
    set_pin_gmux0(TSOP_MUX_S4, binary[0]);
}

inline uint32_t mplexer_5bit_read(mplexer_5bit_t *plexer, uint8_t pin){
    mplexer_5bit_select(plexer, pin);
    return gpio_get_level(plexer->out);
}