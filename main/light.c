#include "light.h"
#include "HandmadeMath.h"
#include "simple_imu.h"
#include "ads1015.h"

static bool readings[LS_NUM] = {0};
static uint32_t rawValues[LS_NUM] = {0};
static light_sensor *sensors[LS_NUM] = {0};
static mplexer_5bit_t lsMux0 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX0_OUT, LS_MUX_EN, LS_MUX_WR
};
static mplexer_5bit_t lsMux1 = {
    LS_MUX_S0, LS_MUX_S1, LS_MUX_S2, LS_MUX_S3, LS_MUX_S4, LS_MUX1_OUT, LS_MUX_EN, LS_MUX_WR
};
static bool data[LS_NUM] = {0};
static bool filledInData[LS_NUM] = {0};
static uint8_t starts[4] = {69};
static uint8_t ends[4] = {69};
static esp_timer_handle_t *lsTimer = NULL;
static const char *TAG = "LightSensor";

float lineAngle = LS_NO_LINE_ANGLE;
float lineSize = 0;
float lastAngle = LS_NO_LINE_ANGLE;
bool isOnLine;
bool lineOver;

////////////////////////////// LIGHT SENSOR //////////////////////////////
uint16_t ls_read(uint8_t mux){
    return ads1015_read(mux);
}

// calibrate each sensor
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

////////////////////////////// LIGHT SENSOR ARRAY //////////////////////////////
static void reset_start_ends(void){
    for (int i = 0; i < 4; i++) {
        // just indicate it's invalid with a random number
        starts[i] = 69;
        ends[i] = 69;
    }
}

static void fill_in_sensors(){
    // "Filling in sensors" if an off sensor has two adjacent on sensors, it will be turned on
    for (int i = 0; i < LS_NUM; i++) {
        filledInData[i] = data[i];

        if (!data[i] && data[mod(i - 1, LS_NUM)] && data[mod(i + 1, LS_NUM)]) {
            filledInData[i] = true;
        }
    }

    lsarray_calc_clusters(true);
}

static void ls_timer_callback(void *arg){

}

void ls_init(void){
    mplexer_5bit_init(&lsMux0);
    mplexer_5bit_init(&lsMux1);

    // instantiate all light sensor structs
    for (int i = 0; i < LS_NUM; i++){
        light_sensor *sensor = calloc(1, sizeof(light_sensor));
        sensor->pin = i;
        sensors[i] = sensor;
        readings[i] = false;
    }

    // calibrate ALL light sensors at once with ls_iterate
    // TODO save these to NVS
    ls_iterate(&ls_func_calibrate);

    // create LS timer
    esp_timer_create_args_t args = {
        .callback = ls_timer_callback,
        .arg = NULL,
        .name = "LSTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, lsTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(*lsTimer, LS_TIMER_PERIOD));

    ESP_LOGI(TAG, "Light sensor init OK");
}

void lsarray_read(void){
    ls_iterate(&ls_func_read);
}

void lsarray_calc_clusters(bool doneFillInSensors){
    reset_start_ends();

    // Sensor index
    uint8_t index = 0;
    
    // Previous sensor on/off value
    bool previousValue = false;

    // Loop through the sensors to find clusters
    for (int i = 0; i < LS_NUM; i++) {
        // Cluster start if sensors go off->on
        if (readings[i] && !previousValue) {
            starts[index] = i;
        }

        // Cluster end if sensors go on->off
        if (!readings[i] && previousValue) {
            ends[index] = i - 1;
            index++;

            if (index > 3) {
                // Too many clusters
                if (!doneFillInSensors) {
                    // "Fill in" sensors
                    fill_in_sensors();
                } else {
                    // Unrecognisable line
                    reset_start_ends();
                    numClusters = 0;
                }

                return;
            }
        }

        previousValue = readings[i];
    }

    // Number of completed clusters
    int tempNumClusters = (int)(starts[0] != LS_ES_DEFAULT) + (int)(starts[1] != LS_ES_DEFAULT) + (int)(starts[2] != LS_ES_DEFAULT) + (int)(starts[3] != LS_ES_DEFAULT);

    if (tempNumClusters != index) {
        // If the final cluster didn't end, index will be one less than tempNumClusters

        if (starts[0] == 0) {
            // If the first cluster starts at 0, then merge the first and last
            starts[0] = starts[index];
        } else {
            // Otherwise, end the last cluster
            ends[index] = LS_NUM - 1;
            index++;

            if (index > 3) {
                // Too many clusters
                if (!doneFillInSensors) {
                    // "Fill in" sensors
                    fill_in_sensors();
                } else {
                    // Unrecognisable line
                    reset_start_ends();
                    numClusters = 0;
                }

                return;
            }
        }
    }

    numClusters = index;
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

    // Select MUX 1 LS 0
    mplexer_5bit_select(&lsMux0, 0);
    // Read ADC channel 1
    printf("Value: %d\n", ads1015_read(0));
}