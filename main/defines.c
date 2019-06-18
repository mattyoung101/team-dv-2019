#include "defines.h"
#include "string.h"

static const char *TAG = "Defines";
float TSOP_TUNING[TSOP_NUM] = {0};

void defines_init(uint8_t robotId){
    ESP_LOGI(TAG, "Initialising values as robot ID %d", robotId);

    if (robotId == 0){
        float values[] = {1.0f, 0.8f, 0.6f, 0.6f, 0.6f, 0.5f, 1.0f, 1.0f, 0.2f, 0.2f, 1.2f, 1.2f, 1.0f, 1.0f, 1.0f, 1.0f, 
                        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        // fuckin C bullshit workaround to set values
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    } else {
        float values[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    }

    ESP_LOGD(TAG, "Just making sure it works: %f,%f,%f,%f", TSOP_TUNING[0], TSOP_TUNING[1], TSOP_TUNING[2], TSOP_TUNING[3]);
}