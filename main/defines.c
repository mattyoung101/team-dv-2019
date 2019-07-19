#include "defines.h"
#include "string.h"

static const char *TAG = "Defines";
float TSOP_TUNING[TSOP_NUM] = {0};
bool MOTOR_FL_REVERSED = false;
bool MOTOR_FR_REVERSED = false;
bool MOTOR_BL_REVERSED = false;
bool MOTOR_BR_REVERSED = false;
uint8_t ROBOT_MODE = 254;
uint16_t DRIBBLE_BALL_TOO_FAR;
uint16_t ORBIT_DIST;
uint16_t IN_FRONT_MIN_ANGLE;
uint16_t IN_FRONT_MAX_ANGLE;

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

float ORBIT_CONST;

// Code which sets per-robot values, i.e. values that cannot be set at compile time using #defines

void defines_init(uint8_t robotId){
    ESP_LOGI(TAG, "Initialising values as robot ID #%d", robotId);

    if (robotId == 0){
        float values[] = {
            0.15, 0.20, 0.25, 0.25, 0.26, 0.26, 0.25, 0.25, 0.30, 0.30, 0.30, 0.30,
            0.27, 0.30, 0.35, 0.35, 0.35, 0.23, 0.22, 0.20, 0.15, 0.20, 0.00, 0.10
        };
        // float values[] = {
        //     0.0, 0.15, 0.15, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        // }; // Spare PCB values
        MOTOR_FL_REVERSED = false;
        MOTOR_FR_REVERSED = true;
        MOTOR_BL_REVERSED = false;
        MOTOR_BR_REVERSED = false;
        ROBOT_MODE = MODE_ATTACK;
        CAM_OFFSET_X = 110;
        CAM_OFFSET_Y = 110;
        TSOP_CORRECTION = 0; // -15 on spare PCB
        DEFEND_DISTANCE = 30;
        SURGE_DISTANCE = 35;
        SURGE_STRENGTH = 100;
        BALL_FAR_STRENGTH = 80;
        BALL_CLOSE_STRENGTH = 40;
        ORBIT_SPEED_SLOW = 30;
        ORBIT_SPEED_FAST = 30;
        ORBIT_CONST = 0.6;
        DRIBBLE_BALL_TOO_FAR = 50;
        ORBIT_DIST = 0;
        IN_FRONT_MIN_ANGLE = 10;
        IN_FRONT_MAX_ANGLE = 350;
        
        // fuckin C bullshit workaround to set values
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    } else {
        // float values[] = {
        //     0.1f, 0.05f, 0.0f, -0.00f, 0.0f, 0.01f, 0.00f, -0.0f, -0.05f, -0.05f, -0.05f, -0.05f,
        //     -0.02f, -0.05f, -0.1f, -0.1f, -0.1f, 0.02f, 0.03f, 0.05f, 0.1f, 0.05f, 0.05f, 0.15f
        // };
        float values[] = {
            0.05, 0.10, 0.15, 0.15, 0.15, 0.14, 0.15, 0.15, 0.20, 0.20, 0.20, 0.20,
            0.17, 0.20, 0.25, 0.25, 0.25, 0.13, 0.12, 0.10, 0.05, 0.10, 0.10, 0.00
        };
        MOTOR_FL_REVERSED = false;
        MOTOR_FR_REVERSED = false;
        MOTOR_BL_REVERSED = false;
        MOTOR_BR_REVERSED = true;
        ROBOT_MODE = MODE_DEFEND;
        CAM_OFFSET_X = 110;
        CAM_OFFSET_Y = 110;
        TSOP_CORRECTION = -10;
        DEFEND_DISTANCE = 70; // 24
        SURGE_DISTANCE = 80; // 35
        SURGE_STRENGTH = 60; 
        BALL_FAR_STRENGTH = 80;
        BALL_CLOSE_STRENGTH = 50;
        ORBIT_SPEED_SLOW = 20;
        ORBIT_SPEED_FAST = 30;
        ORBIT_CONST = 0.2;
        DRIBBLE_BALL_TOO_FAR = 0; // TODO FIX THESE VALUES FOR PASSIVE BALL STUFF
        ORBIT_DIST = 0;
        IN_FRONT_MIN_ANGLE = 10;
        IN_FRONT_MAX_ANGLE = 350;
        
        memcpy(TSOP_TUNING, values, TSOP_NUM * sizeof(float));
    }
}