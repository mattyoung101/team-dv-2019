#pragma once
#include "esp_log.h"

// Uncomment to write either Master or Slave to NVS flash so that the device can be identified as either
// #define NVS_WRITE_MASTER
// #define NVS_WRITE_SLAVE

// Websocket
// #define WEBSOCKET_ENABLED

// I2C
#define I2C_ESP_SLAVE_ADDR 23
#define I2C_TIMEOUT 250 // ms
#define I2C_ACK_MODE 0x1 // 0x0 to disable ack
#define I2C_BEGIN_BYTE 0xB

// Music
#define MUSIC_BPM 100

// PIDs
#define FORWARD_KP 10
#define FORWARD_KI 0
#define FORWARD_KD 0.1
#define FORWARD_MAX 255

#define SIDE_KP 5
#define SIDE_KI 0
#define SIDE_KD 0.2
#define SIDE_MAX 255

// Defence
#define DEFEND_DISTANCE 30

// Timers
#define TIMER_TSOP 1

// Maths
#define PI 3.14159265358979323846
#define DEG_RAD 0.017453292519943295
#define RAD_DEG 57.29577951308232

// Logging
#define CONF_LOG_LEVEL ESP_LOG_DEBUG

// AutoMode (code that automatically starts attack/defence tasks based on NVS)
#define AUTOMODE_ILLEGAL 254
#define AUTOMODE_SLAVE 0
#define AUTOMODE_MASTER 1

// ADC
#define ADC_SAMPLES 32

// WiFi
#define WIFI_SSID "Deus Vult Robot"
#define WIFI_PASS "ClapInts123"
#define WIFI_MAXCON 2

// Camera
#define CAM_DATA_LEN 8
#define SERIAL_BUF_LEN 64
#define CAM_BEGIN_BYTE 0xB
#define CAM_END_BYTE 0xE
#define CAM_FRAME_WIDTH 100
#define CAM_FRAME_HEIGHT 100
#define CAM_OFFSET_X 0
#define CAM_OFFSET_Y 0
#define CAM_NO_VALUE 0xBAD

// Goals
#define ENEMY_GOAL 0 //0: Yellow, 1: Blue, 2: OFF
#define HALFWAY_DISTANCE 90

// Pins
// Motors and Encoders
#define MOTOR_FL_PWM 12
#define MOTOR_FL_IN1 4
#define MOTOR_FL_IN2 5
#define MOTOR_FL_ANGLE 315
#define MOTOR_FL_REVERSED false
#define ENC_FL_IN1 1
#define ENC_FL_IN2 0

#define MOTOR_FR_PWM 13
#define MOTOR_FR_IN1 18
#define MOTOR_FR_IN2 19
#define MOTOR_FR_ANGLE 45
#define MOTOR_FR_REVERSED false
#define ENC_FR_IN1 3
#define ENC_FR_IN2 2

#define MOTOR_BL_PWM 14
#define MOTOR_BL_IN1 25
#define MOTOR_BL_IN2 26
#define MOTOR_BL_ANGLE 225
#define MOTOR_BL_REVERSED true
#define ENC_BL_IN1 5
#define ENC_BL_IN2 4

#define MOTOR_BR_PWM 15
#define MOTOR_BR_IN1 27
#define MOTOR_BR_IN2 32
#define MOTOR_BR_ANGLE 130
#define MOTOR_BR_REVERSED true
#define ENC_BR_IN1 7
#define ENC_BR_IN2 6

// Light sensor
#define LS_CALIBRATION_COUNT 10
#define LS_CALIBRATION_BUFFER 300
#define LS_NUM 24
#define LS_NO_LINE_ANGLE 0xBAD
#define LS_NO_LINE_SIZE 0xBAD

#define LS_0 0
#define LS_1 0
#define LS_2 0
#define LS_3 0
#define LS_4 0
#define LS_5 0
#define LS_6 0
#define LS_7 0
#define LS_8 0
#define LS_9 0
#define LS_10 0
#define LS_11 0
#define LS_12 0
#define LS_13 0
#define LS_14 0
#define LS_15 0
#define LS_16 0
#define LS_17 0
#define LS_18 0
#define LS_19 0
#define LS_20 0
#define LS_21 0
#define LS_22 0
#define LS_23 0

// TSOPs
// TODO update these pins
#define TSOP_NUM 24
#define TSOP_PWR_1 30
#define TSOP_PWR_2 29
#define TSOP_PWR_3 28

#define TSOP_0 1
#define TSOP_1 5
#define TSOP_2 7
#define TSOP_3 9
#define TSOP_4 25
#define TSOP_5 27
#define TSOP_6 26
#define TSOP_7 24
#define TSOP_8 8
#define TSOP_9 6
#define TSOP_10 2
#define TSOP_11 0

#define TSOP_TIMER_PERIOD 4
#define TSOP_NO_BALL_ANGLE 0xBAD