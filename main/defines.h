#pragma once
#include "esp_log.h"

// Timers
#define TIMER_TSOP 1

// Maths
#define PI 3.14159265358979323846
#define DEG_RAD 0.017453292519943295
#define RAD_DEG 57.29577951308232

// Logging
#define CONF_LOG_LEVEL ESP_LOG_INFO
#define BAUD 9600

// ADC
#define ADC_SAMPLES 32

// WiFi
#define WIFI_SSID "Deus Vult Robot"
#define WIFI_PASS "ClapInts123"
#define WIFI_MAXCON 2

// Pins
// Motors and Encoders
#define MOTOR_FL_PWM 23
#define MOTOR_FL_IN1 19
#define MOTOR_FL_IN2 18
#define MOTOR_FL_ANGLE 315
#define MOTOR_FL_REVERSED false
#define ENC_FL_IN1 1
#define ENC_FL_IN2 0

#define MOTOR_FR_PWM 22
#define MOTOR_FR_IN1 17
#define MOTOR_FR_IN2 16
#define MOTOR_FR_ANGLE 45
#define MOTOR_FR_REVERSED false
#define ENC_FR_IN1 3
#define ENC_FR_IN2 2

#define MOTOR_BL_PWM 21
#define MOTOR_BL_IN1 57
#define MOTOR_BL_IN2 56
#define MOTOR_BL_ANGLE 225
#define MOTOR_BL_REVERSED true
#define ENC_BL_IN1 5
#define ENC_BL_IN2 4

#define MOTOR_BR_PWM 20
#define MOTOR_BR_IN1 55
#define MOTOR_BR_IN2 54
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