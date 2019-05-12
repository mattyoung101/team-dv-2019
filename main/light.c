#include "light.h"
#include "HandmadeMath.h"
#include "simple_imu.h"
#include "ads1015.h"

static hmm_vec2 readings[LS_NUM] = {0};
static uint32_t rawValues[LS_NUM] = {0};
static light_sensor *sensors[LS_NUM] = {0};;
esp_adc_cal_characteristics_t *adc1_chars;

static mplexer_5bit_t lsMux0 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX0_OUT, LS_MUX_EN, LS_MUX_WR
};

static mplexer_5bit_t lsMux1 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX1_OUT, LS_MUX_EN, LS_MUX_WR
};

static ls_cluster cluster1, cluster2, cluster3;

float lineAngle = LS_NO_LINE_ANGLE;
float lineSize = 0;
float lastAngle = LS_NO_LINE_ANGLE;
bool isOnLine;
bool lineOver;

static const char *TAG = "LightSensor";

////////////////////////////// LIGHT SENSOR //////////////////////////////
uint16_t ls_read(uint8_t mux){
    // uint32_t reading = 0;
    // for (int i = 0; i < ADC_SAMPLES; i++){
    //     if (mux == 0){
    //         reading += adc1_get_raw(LS_MUX0_OUT);
    //     } else {
    //         reading += adc1_get_raw(LS_MUX1_OUT);
    //     }
    // }
    // return reading / ADC_SAMPLES;

    return ads1015_read(mux);
}

static void ls_func_calibrate(light_sensor* ls, uint8_t mux){
    float defaultValue = (float) ls_read(mux);
    uint16_t buffer = mux == 0 ? LS_MUX0_BUFFER : LS_MUX1_BUFFER; // detect which mux we're reading from

    for (int i = 0; i < LS_CALIBRATION_COUNT; i++){
        defaultValue += ls_read(mux);
    }
    // ESP_LOGD(TAG, "Default value: %f, Mux: %d, Sensor: %d", defaultValue, mux, ls->pin);
    ls->thresholdValue = roundf((defaultValue / LS_CALIBRATION_COUNT) + buffer);
}

// read and put in array
static void ls_func_read(light_sensor *ls, uint8_t mux){
    // uint64_t begin = esp_timer_get_time();
    uint16_t reading = ls_read(mux);
    readings[ls->pin].X = reading > ls->thresholdValue;
    rawValues[ls->pin] = reading;
    // printf("Read time: %lld\n", esp_timer_get_time() - begin);
}

void ls_iterate(ls_func_t func){
    for (int i = 0; i < LS_NUM_PER_MUX; i++){
        // dial in the pin on both multiplexers
        mplexer_5bit_select(&lsMux0, i);
        mplexer_5bit_select(&lsMux1, i);

        // call the function pointer for each mux
        (*func)(sensors[i], 0);
        (*func)(sensors[i + 24], 1);
    }   
}

///////// CLUSTER ////////
void cluster_update_left_right(ls_cluster *cluster){
    cluster->leftSensor = mod(cluster->centre - ((cluster->length - 1) / 2.0f), LS_NUM);
    cluster->rightSensor = mod(cluster->centre + ((cluster->length - 1) / 2.0f), LS_NUM);
}

void cluster_update_length_centre(ls_cluster *cluster){
    if (cluster->leftSensor > cluster->rightSensor) {
        cluster->centre = floatMod((-(LS_NUM - cluster->leftSensor) + cluster->rightSensor) / 2.0, LS_NUM);
        cluster->length = (LS_NUM - (cluster->leftSensor - cluster->rightSensor)) + 1;
    } else {
        cluster->centre = (cluster->leftSensor + cluster->rightSensor) / 2.0;
        cluster->length = (cluster->rightSensor - cluster->leftSensor) + 1;
    }
}

void cluster_add_cluster(ls_cluster *cluster1, ls_cluster *cluster2){
    int16_t leftOther = cluster2->leftSensor;
    int16_t rightOther = cluster2->rightSensor;

    if (mod(cluster1->rightSensor + 1, LS_NUM) == leftOther) {
        cluster1->rightSensor = rightOther;
    } else if (mod(rightOther + 1, LS_NUM) == cluster1->leftSensor) {
        cluster1->leftSensor = leftOther;
    } else {
        // We're adding two non-adjacent clusters so we just do not apply any changes to the cluster.
        return;
    }

    cluster_update_length_centre(cluster1);
}

void cluster_reset(ls_cluster *cluster){
    cluster->centre = 0.0f;
    cluster->length = 0;

    cluster_update_left_right(cluster);
}

void cluster_add_clockwise(ls_cluster *cluster){
    cluster->rightSensor = mod(cluster->rightSensor + 1, LS_NUM);
    cluster_update_length_centre(cluster);
}

float cluster_get_angle(ls_cluster *cluster){
    return cluster->centre / (float) LS_NUM * 360.0f;
}

float cluster_get_left_angle(ls_cluster *cluster){
    return cluster->leftSensor / (float) LS_NUM * 360.0f;

}

float cluster_get_right_angle(ls_cluster *cluster){
    return cluster->rightSensor / (float) LS_NUM * 360.0f;
}

////////// LIGHT SENSOR ARRAY //////////

void ls_init(void){
    mplexer_5bit_init(&lsMux0);
    mplexer_5bit_init(&lsMux1);

    // instantiate all light sensor structs
    for (int i = 0; i < LS_NUM; i++){
        light_sensor *sensor = calloc(1, sizeof(light_sensor));
        sensor->pin = i;
        sensors[i] = sensor;
        readings[i].Y = i * (360.0f / (float) LS_NUM); // add angle to each sensor vector
    }

    // calibrate ALL light sensors at once with ls_iterate
    ls_iterate(&ls_func_calibrate);

    cluster_reset(&cluster1);
    cluster_reset(&cluster2);
    cluster_reset(&cluster3);

    ESP_LOGI(TAG, "Light sensor init OK");
}

////////////////////////////// LIGHT SENSOR ARRAY //////////////////////////////

void lsarray_read(void){
    ls_iterate(&ls_func_read);
}

void lsarray_debug(void){
    // ls_iterate(&ls_func_read);

    // // Print raw values
    // printf("BEGIN ");
    // for (int i = 0; i < LS_NUM; i++){
    //     printf("%d ", rawValues[i]);
    // }
    // printf("END\n");

    // Print on line values
    // printf("BEGIN ");
    // for (int i = 0; i < LS_NUM; i++){
    //     printf("%d ", (uint8_t) readings[i].X);
    // }
    // printf("END\n");

    mplexer_5bit_select(&lsMux0, 0);
    printf("Value: %d\n", ads1015_read(0));
}

void lsarray_calc_vec(void){
    hmm_vec2 sum = {0};

    for (int i = 0; i < LS_NUM; i++){
        // convert vectors to cartesian
        float r = readings[i].X;
        float theta = readings[i].Y;
        readings[i].X = r * cosfd(theta);
        readings[i].Y = r * sinfd(theta);

        // vector add all vectors
        sum = HMM_AddVec2(sum, readings[i]);
    }

    // convert back to polar and scale between 1 and 0
    float sumX = ((float) sum.X / (float) LS_NUM);
    float sumY = sum.Y;

    lineSize = sqrtf(sq(sumX) + sq(sumY));
    lineAngle = fmodf((atan2f(sumY, sumX) * RAD_DEG) + 360.0f, 360.0f);
}