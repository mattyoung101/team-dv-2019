#include "tsop.h"
// stupid hack to make it compile, we have to include these here
#define HANDMADE_MATH_IMPLEMENTATION
#define HANDMADE_MATH_NO_SSE
#include "HandmadeMath.h"

static uint16_t tsopCounter = 0;
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
        readings[i].Y = i * (360.0f / (float) TSOP_NUM);
    }
    tsopCounter = 0;
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
            readings[i].X += gpio_get_level(TSOP_4) ^ 1;
        } else if (i == 5){
            readings[i].X += gpio_get_level(TSOP_5) ^ 1;
        } else {
            readings[i].X += mplexer_4bit_read(&tsopMux, irTable[i]) ^ 1;
        }
    }
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
    // int64_t begin = esp_timer_get_time();

    // scale down the magnitudes
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X = ((float) readings[i].X / (float) tsopCounter);
    }

    // sort values to obtain the best n vectors with the highest magnitudes
    qsort(readings, TSOP_NUM, sizeof(hmm_vec2), cmp_vec_mag);

    // convert them to cartesian
    for (int i = 0; i < TSOP_BEST; i++){
        float r = readings[i].X;
        float theta = readings[i].Y;
        readings[i].X = r * cosfd(theta);
        readings[i].Y = r * sinfd(theta);
    }

    // sum them
    hmm_vec2 sum;
    for (int i = 0; i < TSOP_BEST; i++){
        sum = HMM_AddVec2(sum, readings[i]);
    }

    // convert back to polar
    float sumX = sum.X;
    float sumY = sum.Y;
    tsopStrength = sqrtf(sq(sumX) + sq(sumY));
    tsopAngle = fmodf((atan2f(sumY, sumX) * RAD_DEG) + 360.0f, 360.0f);

    tsop_reset();

    // int64_t diff = esp_timer_get_time() - begin;
    // ESP_LOGV(TAG, "Time: %" PRIu64 " us", diff);
}

void tsop_dump(void){
    // this is generated using tsop_format_gen.py in the scripts folder
    // even still, holy shit
    ESP_LOGV(TAG, "Values: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)," 
    "(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), "
    "(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)", readings[0].X, readings[0].Y, readings[1].X, readings[1].Y, readings[2].X, 
    readings[2].Y, readings[3].X, readings[3].Y, readings[4].X, readings[4].Y, readings[5].X, readings[5].Y, readings[6].X, 
    readings[6].Y, readings[7].X, readings[7].Y, readings[8].X, readings[8].Y, readings[9].X, readings[9].Y, readings[10].X, 
    readings[10].Y, readings[11].X, readings[11].Y, readings[12].X, readings[12].Y, readings[13].X, readings[13].Y, 
    readings[14].X, readings[14].Y, readings[15].X, readings[15].Y, readings[16].X, readings[16].Y, readings[17].X, 
    readings[17].Y);
}