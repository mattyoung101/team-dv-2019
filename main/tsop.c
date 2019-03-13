#include "tsop.h"

static gpio_num_t TSOPPins[TSOP_NUM] = {TSOP_0, TSOP_1, TSOP_2, TSOP_3, TSOP_4, TSOP_5, TSOP_6, TSOP_7, TSOP_8, TSOP_9, 
                                TSOP_10, TSOP_11};
static float scaledSin[TSOP_NUM];
static float scaledCos[TSOP_NUM];
static uint16_t tsopCounter;
static uint16_t tempValues[TSOP_NUM];
static uint16_t tsopIndexes[TSOP_NUM];

// This file contains the code that reads and processes the TSOPs

void tsop_init(void){
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_PWR_1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_PWR_2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(TSOP_PWR_3, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(gpio_set_level(TSOP_PWR_1, 1));
    ESP_ERROR_CHECK(gpio_set_level(TSOP_PWR_2, 1));
    ESP_ERROR_CHECK(gpio_set_level(TSOP_PWR_3, 1));

    for (uint8_t i = 0; i < TSOP_NUM; i++) {
        ESP_ERROR_CHECK(gpio_set_direction(TSOPPins[i], GPIO_MODE_INPUT));
    }

    // TODO precalculate the scaled table
    for (int i = 0; i < TSOP_NUM; i++){
        float angle = (i * (360 / TSOP_NUM)) * DEG_RAD;

        scaledCos[i] = cosf(angle);
        scaledSin[i] = sinf(angle);
    }
}

void tsop_update(void){
    // Read each sensor once
    tempValues[0] += gpio_get_level(TSOP_0) ^ 1;
    tempValues[1] += gpio_get_level(TSOP_1) ^ 1;
    tempValues[2] += gpio_get_level(TSOP_2) ^ 1;
    tempValues[3] += gpio_get_level(TSOP_3) ^ 1;
    tempValues[4] += gpio_get_level(TSOP_4) ^ 1;
    tempValues[5] += gpio_get_level(TSOP_5) ^ 1;
    tempValues[6] += gpio_get_level(TSOP_6) ^ 1;
    tempValues[7] += gpio_get_level(TSOP_7) ^ 1;
    tempValues[8] += gpio_get_level(TSOP_8) ^ 1;
    tempValues[9] += gpio_get_level(TSOP_9) ^ 1;
    tempValues[10] += gpio_get_level(TSOP_10) ^ 1;
    tempValues[11] += gpio_get_level(TSOP_11) ^ 1;

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

    // TODO replace this with a call to qsort()
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
        tsopAngle = TSOP_NO_BALL_ANGLE;
    } else {
        tsopAngle = mod(atan2f(y, x) * RAD_DEG, 360);
    }

    tsopStrength = sqrtf(x * x + y * y);
}