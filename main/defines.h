#pragma once
#include "esp_log.h"

// Maths
#define PI 3.14159265358979323846
#define DEG_RAD 0.017453292519943295
#define RAD_DEG 57.29577951308232

// Logging
#define CONF_LOG_LEVEL ESP_LOG_INFO
#define CONF_BAUD 9600

// WiFi
#define CONF_WIFI_SSID "Deus Vult Robot"
#define CONF_WIFI_PASS "ClapInts123"
#define CONF_WIFI_MAXCON 2

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