#include "tsop.h"
// stupid hack to make it compile, we have to include these here
#define HANDMADE_MATH_IMPLEMENTATION
#define HANDMADE_MATH_NO_SSE
#include "HandmadeMath.h"

static uint16_t tsopCounter;
// in polar form, so x = mag, y = theta
static hmm_vec2 readings[TSOP_NUM] = {0};

static mplexer_4bit_t tsopMux = {
    TSOP_MUX_S0, TSOP_MUX_S1, TSOP_MUX_S2, TSOP_MUX_S3, TSOP_MUX_OUT
};

// Index = TSOP number, Value = multiplexer pin
// TSOP 4 & 5 are unconnected so they are represented by 255
static const gpio_num_t irTable[] = {8, 0, 1, 2, 255, 255, 7, 6, 5, 4, 3, 15, 14, 13, 12, 11, 10, 9};
static const char *TAG = "TSOP";

static void tsop_reset(){
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X = 0.0f;
        readings[i].Y = i * (360.0f / (float) TSOP_NUM) * DEG_RAD;
    }
}

void tsop_init(void){
    gpio_set_direction(TSOP_PWR_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(TSOP_PWR_2, GPIO_MODE_OUTPUT);
    gpio_set_level(TSOP_PWR_1, 1);
    gpio_set_level(TSOP_PWR_2, 1);

    gpio_set_direction(TSOP_4, GPIO_MODE_INPUT);
    gpio_set_direction(TSOP_5, GPIO_MODE_INPUT);

    mplexer_4bit_init(&tsopMux);

    tsop_reset();
}

void tsop_update(void *args){
    for (int i = 0; i < TSOP_NUM; i++){
        if (i == 4){
            ESP_LOGD(TAG, "Reading TSOP 4 from GPIO");
            readings[i].X += gpio_get_level(TSOP_4) ^ 1;
        } else if (i == 5){
            ESP_LOGD(TAG, "Reading TSOP 5 from GPIO");
            readings[i].X += gpio_get_level(TSOP_5) ^ 1;
        } else {
            ESP_LOGD(TAG, "Reading TSOP %d from mux", i);
            readings[i].X += mplexer_4bit_read(&tsopMux, irTable[i]) ^ 1;
        }
    }
    printf("===== NEW READ =====\n");
    tsopCounter++;
}

static int cmp_vec_mag(const void *p, const void *q){
    hmm_vec2 a = *(const hmm_vec2 *) p;
    hmm_vec2 b = *(const hmm_vec2 *) q;

    if (a.X < b.X){
        return 1;
    } else if (a.X > b.X){
        return -1;
    } else {
        return 1;
    }
}

void tsop_calc(){
    // scale down the magnitudes
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X = ((float) readings[i].X / (float) tsopCounter) * 100.0f;
    }

    // sort values to obtain the best n vectors with the highest magnitudes
    qsort(readings, TSOP_NUM, sizeof(hmm_vec2), cmp_vec_mag);

    // convert them to cartesian (r cos theta, r sin theta)
    for (int i = 0; i < TSOP_BEST; i++){
        float tmpX = readings[i].X;
        float tmpY = readings[i].Y;
        readings[i].X = tmpX * cosf(tmpY * DEG_RAD);
        readings[i].Y = tmpX * sinf(tmpY * DEG_RAD);
    }

    // sum them
    hmm_vec2 sum;
    for (int i = 0; i < TSOP_BEST; i++){
        sum = HMM_AddVec2(sum, readings[i]);
    }

    // convert back to polar
    float sumX = sum.X;
    float sumY = sum.Y;
    sum.X = sqrtf(sq(sumX) + sq(sumY));
    sum.Y = atan2f(sumY, sumX);

    tsopStrength = sum.X;
    tsopAngle = sum.Y;

    tsop_reset();
}

void tsop_dump(void){
    // FIXME needs to be changed for new library, print magnitude?
    // ESP_LOGD(TAG, "Values: %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d | Updated %d times "
    //     "(target %d)",
    //     readings[0], readings[1], readings[2], readings[3], readings[4], readings[5], readings[6], 
    //     readings[7], readings[8], readings[9], readings[10], readings[11], readings[12], readings[13], 
    //     readings[14], readings[15], readings[16], readings[17], tsopCounter, TSOP_TARGET_READS);
}