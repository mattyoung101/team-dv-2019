#include "tsop.h"

static float scaledSin[TSOP_NUM];
static float scaledCos[TSOP_NUM];
static uint16_t tsopCounter;
static uint16_t tempValues[TSOP_NUM];
static uint16_t tsopIndexes[TSOP_NUM];
static mplexer_4bit_t tsopMux = {
    TSOP_MUX_S0, TSOP_MUX_S1, TSOP_MUX_S2, TSOP_MUX_S3, TSOP_MUX_OUT
};
// Index = TSOP number, Value = multiplexer pin. If 255 it's unconnected.
static const gpio_num_t irTable[] = {8, 0, 1, 2, 255, 255, 7, 6, 5, 4, 3, 15, 14, 13, 12, 11, 10, 9};

void tsop_init(void){
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_PWR_1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_PWR_2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(TSOP_PWR_1, 1));
    ESP_ERROR_CHECK(gpio_set_level(TSOP_PWR_2, 1));

    ESP_ERROR_CHECK(gpio_set_direction(TSOP_4, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_5, GPIO_MODE_INPUT));

    mplexer_4bit_init(&tsopMux);

    for (int i = 0; i < TSOP_NUM; i++){
        float angle = (i * (360 / TSOP_NUM)) * DEG_RAD;

        scaledCos[i] = cosf(angle);
        scaledSin[i] = sinf(angle);
    }
}

void tsop_update(void *args){
    for (int i = 0; i < TSOP_NUM; i++){
        if (i == 4){
            tempValues[i] += gpio_get_level(TSOP_4) ^ 1;
        } else if (i == 5){
            tempValues[i] += gpio_get_level(TSOP_5) ^ 1;
        } else {
            tempValues[i] += mplexer_4bit_read(&tsopMux, irTable[i]) ^ 1;
        }
    }
    
    tsopCounter++;
}

// same as TSOPArray.finishRead()
void tsop_process(void){
    for (int i = 0; i < TSOP_NUM; i++){
        tsopValues[i] = 100.0f * (float) tempValues[i] / (float) tsopCounter;
        tempValues[i] = 0;
        tsopSortedValues[i] = 0;
        tsopIndexes[i] = 0;
    }
    tsopCounter = 0;

    // quicksort this?
    for (int i = 0; i < TSOP_NUM; i++) {
        for (int j = 0; j < TSOP_NUM; j++) {
            if (tsopValues[i] > tsopSortedValues[j]) {
                // We've found our place!
                // Shift elements from index j down
                if (j <= i) {
                    // Make sure we only shift what is needed
                    ARRAYSHIFTDOWN(tsopSortedValues, j, i);
                    ARRAYSHIFTDOWN(tsopIndexes, j, i);
                }

                tsopSortedValues[j] = tsopValues[i];
                tsopIndexes[j] = i;
                break;
            }
        }
    }
}

void tsop_calc(uint8_t n){
    int16_t x = 0;
    int16_t y = 0;

    for (int i = 0; i < n; i++){
        // convert vector to cartesian
        x += tsopSortedValues[i] * scaledCos[tsopIndexes[i]];
        y += tsopSortedValues[i] * scaledSin[tsopIndexes[i]];
    }

    if (x == 0 && y == 0) {
        // When vectors sum to (0, 0), we're in trouble. We've got some dodgy data
        ESP_LOGV("TSOP", "No ball found");
        tsopAngle = (float) TSOP_NO_BALL_ANGLE;
    } else {
        tsopAngle = mod(atan2f(y, x) * RAD_DEG, 360);
    }

    tsopStrength = sqrtf(x * x + y * y);
}

void tsop_dump(void){
    ESP_LOGI("TSOP", "Values: %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d",
        tempValues[0], tempValues[1], tempValues[2], tempValues[3], tempValues[4], tempValues[5], tempValues[6], 
        tempValues[7], tempValues[8], tempValues[9], tempValues[10], tempValues[11], tempValues[12], tempValues[13], 
        tempValues[14], tempValues[15], tempValues[16], tempValues[17]);
}