#include "light.h"

static bool readings[LS_NUM] = {0};
static uint32_t rawValues[LS_NUM] = {0};
static light_sensor *sensors[LS_NUM] = {0};
static mplexer_5bit_t lsMux0 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX0_OUT, LS_EN
};
static mplexer_5bit_t lsMux1 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX1_OUT, LS_EN
};
static const char *TAG = "LightSensor";

////////////////////////////// LIGHT SENSOR //////////////////////////////
uint16_t ls_read(uint8_t mux){
    uint32_t reading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++){
        if (mux == 0){
            reading += adc1_get_raw(LS_MUX0_OUT);
        } else {
            int heck = 0;
            ESP_ERROR_CHECK(adc2_get_raw(LS_MUX1_OUT, ADC_WIDTH_10Bit, &heck));
            reading += heck;
        }
    }
    // ESP_LOGD(TAG, "The fucking reading is fucking %d on mux %d", reading, mux);
    return reading / ADC_SAMPLES;
}

static void ls_func_calibrate(light_sensor* ls, uint8_t mux){
    float defaultValue = (float) ls_read(mux);
    for (int i = 0; i < LS_CALIBRATION_COUNT; i++){
        defaultValue += ls_read(mux);
    }
    // ESP_LOGD(TAG, "Default value: %f, Mux: %d, Sensor: %d", defaultValue, mux, ls->pin);
    ls->thresholdValue = roundf((defaultValue / LS_CALIBRATION_COUNT) + LS_CALIBRATION_BUFFER);
}

// read and put in array
static void ls_func_read(light_sensor *ls, uint8_t mux){
    uint16_t reading = ls_read(mux);
    readings[ls->pin] = reading > ls->thresholdValue;
    rawValues[ls->pin] = reading;
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
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(LS_MUX0_OUT, ADC_ATTEN_0db);
    esp_adc_cal_characteristics_t *adc1_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    print_char_val_type(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_10, 1100, adc1_chars));
    
    adc2_config_channel_atten(LS_MUX1_OUT, ADC_ATTEN_0db);
    esp_adc_cal_characteristics_t *adc2_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    print_char_val_type(esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_BIT_10, 1100, adc2_chars));

    mplexer_5bit_init(&lsMux0);
    mplexer_5bit_init(&lsMux1);

    // instantiate all light sensor structs
    for (int i = 0; i < LS_NUM; i++){
        light_sensor *sensor = calloc(1, sizeof(light_sensor));
        sensor->pin = i;
        sensors[i] = sensor;
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
    
    printf("BEGIN ");
    for (int i = 0; i < LS_NUM; i++){
        printf("%d ", rawValues[i]);
    }
    printf("END\n");
}

void lsarray_calc(void){
    
}