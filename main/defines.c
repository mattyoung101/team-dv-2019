#include "defines.h"
#include "string.h"

static const char *TAG = "Defines";
float TSOP_TUNING[TSOP_NUM] = {0};
bool MOTOR_FL_REVERSED = false;
bool MOTOR_FR_REVERSED = false;
bool MOTOR_BL_REVERSED = false;
bool MOTOR_BR_REVERSED = false;
uint8_t ROBOT_MODE = 254;
int16_t CAM_OFFSET_X;
int16_t CAM_OFFSET_Y;
int16_t TSOP_CORRECTION;

// Code which sets per-robot values, i.e. values that cannot be set at compile time using #defines

void defines_init(uint8_t robotId){
    ESP_LOGI(TAG, "Initialising values as robot ID #%d", robotId);

    if (robotId == 0){
        float values[] = {0.8f, 0.8f, 0.6f, 0.5f, 0.5f, 0.4f, 0.3f, 0.3f, 0.2f, 0.2f, 1.2f, 1.2f, 1.0f, 1.0f, 1.0f, 1.0f, 
                        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.8f, 0.8f};
        MOTOR_FL_REVERSED = true;
        MOTOR_FR_REVERSED = true;
        MOTOR_BL_REVERSED = true;
        MOTOR_BR_REVERSED = true;
        ROBOT_MODE = MODE_ATTACK;
        CAM_OFFSET_X = 92;
        CAM_OFFSET_Y = 68;
        
        // fuckin C bullshit workaround to set values
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    } else {
        float values[] = {1.0f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 1.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.0f, 0.8f, 0.8f, 1.0f,
                        0.6f, 0.6f, 0.6f, 1.0f, 1.0f, 1.0f, 1.1f, 1.0f};
        MOTOR_FL_REVERSED = true;
        MOTOR_FR_REVERSED = true;
        MOTOR_BL_REVERSED = false;
        MOTOR_BR_REVERSED = true;
        ROBOT_MODE = MODE_ATTACK;
        CAM_OFFSET_X = 94;
        CAM_OFFSET_Y = 47;
        TSOP_CORRECTION = -30;
        
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    }
}