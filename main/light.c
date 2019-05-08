#include "light.h"
#include "HandmadeMath.h"
#include "simple_imu.h"

static hmm_vec2 readings[LS_NUM] = {0};
static uint32_t rawValues[LS_NUM] = {0};
static light_sensor *sensors[LS_NUM] = {0};
esp_adc_cal_characteristics_t *adc1_chars;

static mplexer_5bit_t lsMux0 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX0_OUT, LS_MUX_EN, LS_MUX_WR
};

static mplexer_5bit_t lsMux1 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX1_OUT, LS_MUX_EN, LS_MUX_WR
};


float lineAngle = LS_NO_LINE_ANGLE;
float lineSize = 0;
float lastAngle = LS_NO_LINE_ANGLE;
bool isOnLine;
bool lineOver;

static const char *TAG = "LightSensor";

////////////////////////////// LIGHT SENSOR //////////////////////////////
uint16_t ls_read(uint8_t mux){
    uint32_t reading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++){
        if (mux == 0){
            reading += adc1_get_raw(LS_MUX0_OUT);
        } else {
            reading += adc1_get_raw(LS_MUX1_OUT);
        }
    }
    return reading / ADC_SAMPLES;
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

static void print_char_val_type(esp_adc_cal_value_t val_type){
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref");
    }
}

void ls_init(void){
    // TODO make this 12 bit for higher accuracy - does it make it slower?
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LS_MUX0_OUT, ADC_ATTEN_0db);
    adc1_config_channel_atten(LS_MUX1_OUT, ADC_ATTEN_0db);
    adc1_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    print_char_val_type(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_10, 10, adc1_chars));
    
    // esp_adc_cal_characteristics_t *adc1_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // print_char_val_type(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_10, 1100, adc1_chars));

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
    ESP_LOGI(TAG, "Light sensor init OK");
}

////////////////////////////// LIGHT SENSOR ARRAY //////////////////////////////
void lsarray_read(void){
    ls_iterate(&ls_func_read);
}

void lsarray_debug(void){
    ls_iterate(&ls_func_read);

    // Print raw values
    printf("BEGIN ");
    for (int i = 0; i < LS_NUM; i++){
        // printf("%d ", esp_adc_cal_raw_to_voltage(rawValues[i], adc1_chars));
        printf("%d ", rawValues[i]);
    }
    printf("END\n");

    // Print on line values
    // printf("BEGIN ");
    // for (int i = 0; i < LS_NUM; i++){
    //     printf("%d ", (uint8_t) readings[i].X);
    // }
    // printf("END\n");
}

void lsarray_calc(void){
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

    // Processing of values
    isOnLine = lineSize == 0 ? false : true;
    lineAngle = fmod(lineAngle + heading, 360);
    if (lineSize != 0) lineSize = lineOver ? 1 - lineSize : lineSize;


}