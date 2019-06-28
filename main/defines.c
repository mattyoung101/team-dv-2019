#include "defines.h"
#include "string.h"

static const char *TAG = "Defines";
float TSOP_TUNING[TSOP_NUM] = {0};
bool MOTOR_FL_REVERSED = false;
bool MOTOR_FR_REVERSED = false;
bool MOTOR_BL_REVERSED = false;
bool MOTOR_BR_REVERSED = false;
uint8_t ROBOT_MODE = 254;

// --- Camera --- //
int16_t CAM_OFFSET_X;
int16_t CAM_OFFSET_Y;

// --- TSOPS --- //
int16_t TSOP_CORRECTION;

// --- Goalie --- //
uint8_t DEFEND_DISTANCE;
uint8_t SURGE_DISTANCE;
uint8_t SURGE_STRENGTH;

// --- Orbit --- //
uint8_t BALL_FAR_STRENGTH;
uint8_t BALL_CLOSE_STRENGTH;
uint8_t ORBIT_SPEED_SLOW;
uint8_t ORBIT_SPEED_FAST;

uint8_t ORBIT_CONST;

// Code which sets per-robot values, i.e. values that cannot be set at compile time using #defines

void defines_init(uint8_t robotId){
    ESP_LOGI(TAG, "Initialising values as robot ID #%d", robotId);

    if (robotId == 0){
        float values[] = {
            1.1f, 1.1f, 1.0f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 1.0f, 1.0f, 
            1.0f, 1.0f, 1.0f, 0.9f, 0.9f, 0.9f, 0.9f, 0.9f, 1.0f, 1.0, 1.3f, 1.3f
        };
        MOTOR_FL_REVERSED = true;
        MOTOR_FR_REVERSED = true;
        MOTOR_BL_REVERSED = false;
        MOTOR_BR_REVERSED = true;
        ROBOT_MODE = MODE_ATTACK;
        CAM_OFFSET_X = 56;
        CAM_OFFSET_Y = 56;
        TSOP_CORRECTION = -10;
        DEFEND_DISTANCE = 30;
        SURGE_DISTANCE = 35;
        SURGE_STRENGTH = 150;
        BALL_FAR_STRENGTH = 100;
        BALL_CLOSE_STRENGTH = 130;
        ORBIT_SPEED_SLOW = 30;
        ORBIT_SPEED_FAST = 40;
        ORBIT_CONST = 0.2;
        
        // fuckin C bullshit workaround to set values
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    } else {
        float values[] = {
            1.1f, 1.1f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 
            1.0f, 1.0f, 1.0f, 1.0f, 0.8f, 0.8f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f
        };
        MOTOR_FL_REVERSED = false;
        MOTOR_FR_REVERSED = true;
        MOTOR_BL_REVERSED = true;
        MOTOR_BR_REVERSED = false;
        ROBOT_MODE = MODE_DEFEND;
        CAM_OFFSET_X = 56;
        CAM_OFFSET_Y = 56;
        TSOP_CORRECTION = -25;
        DEFEND_DISTANCE = 40;
        SURGE_DISTANCE = 40;
        SURGE_STRENGTH = 165;
        BALL_FAR_STRENGTH = 100;
        BALL_CLOSE_STRENGTH = 140;
        ORBIT_SPEED_SLOW = 30;
        ORBIT_SPEED_FAST = 40;
        ORBIT_CONST = 0.2;
        
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    }
}