#include "tsop.h"
#include "HandmadeMath.h"

static uint16_t tsopCounter = 0;
// in polar form, so x = mag, y = theta
static hmm_vec2 readings[TSOP_NUM] = {0};
static mplexer_5bit_t tsopMux = {
    TSOP_MUX_S0, TSOP_MUX_S1, TSOP_MUX_S2, TSOP_MUX_S3, TSOP_MUX_S4, TSOP_MUX_OUT, TSOP_MUX_EN, TSOP_MUX_WR
};
// Index = TSOP number, Value = multiplexer pin
static const gpio_num_t irTable[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 23, 22, 21, 20, 19, 18, 17, 16};
#ifdef TSOP_DEBUG
    static const char *TAG = "TSOP";
#endif
static movavg_t *angleAvg = NULL;
static movavg_t *strengthAvg = NULL;
float tsopAngle = 0.0f;
float tsopStrength = 0.0f;
float tsopAvgAngle = 0.0f;
float tsopAvgStrength = 0.0f;

static void tsop_reset(){
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X = 0.0f;
        readings[i].Y = i * (360.0f / (float) TSOP_NUM);
    }
    tsopCounter = 0;
}

void tsop_init(void){
    mplexer_5bit_init(&tsopMux);
    tsop_reset();

    angleAvg = movavg_create(TSOP_MOVAVG_SIZE);
    strengthAvg = movavg_create(TSOP_MOVAVG_SIZE);
}

inline void tsop_update(void *args){
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X += mplexer_5bit_read(&tsopMux, irTable[i]) ^ 1;
    }
    tsopCounter++;
}

/** compares polar vectors based on their magnitudes in descending order **/
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

void tsop_calc(void){
    #ifdef TSOP_DEBUG
        ESP_LOGI(TAG, "TSOP 0: %f, TSOP 12: %f", readings[0].X, readings[12].X);
        ESP_LOGI(TAG, "Read %d times", tsopCounter);
    #endif

    // scale down the magnitudes
    for (int i = 0; i < TSOP_NUM; i++){
        readings[i].X = ((float) readings[i].X / (float) tsopCounter);
        #if TSOP_SCALING
            readings[i].X = constrain(readings[i].X - TSOP_TUNING[i], 0, 1);
        #endif
    }
    #ifdef TSOP_DEBUG
        ESP_LOGI(TAG, "Scaled down:");
        tsop_dump();
    #endif

    // sort values to obtain the best n vectors with the highest magnitudes
    qsort(readings, TSOP_NUM, sizeof(hmm_vec2), cmp_vec_mag);
    #ifdef TSOP_DEBUG
        ESP_LOGI(TAG, "Sorted:");
        tsop_dump();
    #endif

    // convert them to cartesian
    for (int i = 0; i < TSOP_NUM; i++){
        float r = readings[i].X;
        float theta = -1.0 * (readings[i].Y - 90.0f);
        readings[i].X = r * cosfd(theta);
        readings[i].Y = r * sinfd(theta);
    }
    #ifdef TSOP_DEBUG
        ESP_LOGI(TAG, "Converted to cartesian:");
        tsop_dump();
    #endif

    // sum them
    hmm_vec2 sum = {0};
    for (int i = 0; i < TSOP_BEST; i++){
        #ifdef TSOP_DEBUG
            ESP_LOGD(TAG, "Current sum: (%f, %f) adding: (%f, %f)", sum.X, sum.Y, readings[i].X, readings[i].Y);
        #endif
        sum = HMM_AddVec2(sum, readings[i]);
    }

    // convert back to polar
    float sumX = sum.X;
    float sumY = sum.Y;
    tsopStrength = sqrtf(sq(sumX) + sq(sumY));
    tsopAngle = fmodf(450 - (atan2f(sumY, sumX) * RAD_DEG), 360.0f);
    
    movavg_push(angleAvg, tsopAngle);
    movavg_push(strengthAvg, tsopStrength);
    tsopAvgAngle = movavg_calc(angleAvg);
    tsopAvgStrength = movavg_calc(strengthAvg);

    #ifdef TSOP_DEBUG
        ESP_LOGI(TAG, "Average angle: %f, Average strength: %f", tsopAvgAngle, tsopAvgStrength);
        ESP_LOGI(TAG, "Angle %f, Strength: %f", tsopAngle, tsopStrength);
    #endif

    tsop_reset();
}

void tsop_dump(void){
    // this is generated using tsop_format_gen.py in the scripts folder, yes I know it sucks
    // printf("BEGIN_TSOP_DEBUG %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f "
    // "%.4f %.4f %.4f %.4f %.4f %.4f\n", readings[0].X, readings[1].X, readings[2].X, readings[3].X, readings[4].X, 
    // readings[5].X, readings[6].X, readings[7].X, readings[8].X, readings[9].X, readings[10].X, readings[11].X, 
    // readings[12].X, readings[13].X, readings[14].X, readings[15].X, readings[16].X, readings[17].X, readings[18].X, 
    // readings[19].X, readings[20].X, readings[21].X, readings[22].X, readings[23].X);
    // printf("BEGIN_TSOP_DEBUG %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f "
    // "%.4f %.4f %.4f %.4f %.4f %.4f\n", readings[0].Y, readings[1].Y, readings[2].Y, readings[3].Y, readings[4].Y, 
    // readings[5].Y, readings[6].Y, readings[7].Y, readings[8].Y, readings[9].Y, readings[10].Y, readings[11].Y, 
    // readings[12].Y, readings[13].Y, readings[14].Y, readings[15].Y, readings[16].Y, readings[17].Y, readings[18].Y, 
    // readings[19].Y, readings[20].Y, readings[21].Y, readings[22].Y, readings[23].Y);
    printf("Values: (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), "
    "(%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), "
    "(%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), (%f, %f), "
    "(%f, %f), (%f, %f)\n", readings[0].X, readings[0].Y, readings[1].X, readings[1].Y, readings[2].X, readings[2].Y, 
    readings[3].X, readings[3].Y, readings[4].X, readings[4].Y, readings[5].X, readings[5].Y, readings[6].X, readings[6].Y, 
    readings[7].X, readings[7].Y, readings[8].X, readings[8].Y, readings[9].X, readings[9].Y, readings[10].X, readings[10].Y, 
    readings[11].X, readings[11].Y, readings[12].X, readings[12].Y, readings[13].X, readings[13].Y, readings[14].X, 
    readings[14].Y, readings[15].X, readings[15].Y, readings[16].X, readings[16].Y, readings[17].X, readings[17].Y, 
    readings[18].X, readings[18].Y, readings[19].X, readings[19].Y, readings[20].X, readings[20].Y, readings[21].X, 
    readings[21].Y, readings[22].X, readings[22].Y, readings[23].X, readings[23].Y);
}