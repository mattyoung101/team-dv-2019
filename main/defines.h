#pragma once
#include "esp_log.h"

// Uncomment ONE of these to identify the device as either a master or a slave.
// #define NVS_WRITE_MASTER
// #define NVS_WRITE_SLAVE

// If this is defined, the value of the robot number will be written to NVS
// #define NVS_WRITE_ROBOTNUM 0 // 0 or 1, 0 = wifi host, 1 = wifi client

// Debug mode - if uncommented, enables the code that communicates to the Aquila monitoring webapp
// Will probably slow down the robot, don't enable in competition!
#define WEB_DEBUG_ENABLED

// FreeRTOS
#define SEMAPHORE_UNLOCK_TIMEOUT 25 // ms
#define CONF_LOG_LEVEL ESP_LOG_DEBUG

// I2C
#define I2C_ESP_SLAVE_ADDR 23
#define I2C_TIMEOUT 250 // ms
#define I2C_ACK_MODE 0x1 // 0x0 to disable ack
#define I2C_BEGIN_DEFAULT 0xB // default packet, has sensor data
#define I2C_BEGIN_DEBUG 0xC // debug packet, has raw data for sending to webserver

// Music
#define MUSIC_BPM 100

// PIDs

// --- IMU Correction --- //
// Note: this needs to be reversed (-pid_update)
#define HEADING_KP 0.6
#define HEADING_KI 0
#define HEADING_KD 0.06
#define HEADING_MAX_CORRECTION 100

#define FORWARD_KP 10
#define FORWARD_KI 0
#define FORWARD_KD 0.1
#define FORWARD_MAX 100

#define SIDE_KP 5
#define SIDE_KI 0
#define SIDE_KD 0.2
#define SIDE_MAX 100

#define COORD_KP 5
#define COORD_KI 0
#define COORD_KD 0.2
#define COORD_MAX 100

#define LRF_KP 1
#define LRF_KI 0
#define LRF_KD 0.1
#define LRF_MAX 100

// --- Goal Correction --- //
#define GOAL_KP 0.6
#define GOAL_KI 0
#define GOAL_KD 0.06
#define GOAL_MAX_CORRECTION 100

// --- Idle Correction --- //
#define IDLE_KP 0.1
#define IDLE_KI 0
#define IDLE_KD 0.01
#define IDLE_MAX_CORRECTION 100

// Maths
#define PI 3.14159265358979323846
#define E 2.71828182845904523536
#define DEG_RAD 0.017453292519943295 // multiply to convert degrees to radians
#define RAD_DEG 57.29577951308232 // multiply to convert radians to degrees

// AutoMode (code that automatically starts attack/defence tasks based on NVS)
#define AUTOMODE_ILLEGAL 254
#define AUTOMODE_SLAVE 0
#define AUTOMODE_MASTER 1

// WiFi
#define WIFI_SSID "DVRobotLink"
#define WIFI_PASS "ClapInts123"
#define WIFI_MAXCON 3 // 1 for other robot, 2 for websockets 
#define SOCK_ADDR "192.168.0.165"
#define SOCK_PORT 12323
#define WS_PORT 14323

// Camera
#define CAM_DATA_LEN 8
#define CAM_BEGIN_BYTE 0xB
#define CAM_END_BYTE 0xE
#define CAM_FRAME_WIDTH 0
#define CAM_FRAME_HEIGHT 0
#define CAM_OFFSET_X 75
#define CAM_OFFSET_Y 55
#define CAM_ANGLE_OFFSET 0
#define CAM_NO_VALUE 0xBAD
#define CAM_UART_TX 17
#define CAM_UART_RX 16

// Goals
#define GOAL_YELLOW 0
#define GOAL_BLUE 1
#define GOAL_OFF 2
#define ENEMY_GOAL GOAL_YELLOW
#define HALFWAY_DISTANCE 90
#define IDLE_DISTANCE 90
#define IDLE_OFFSET 30
#define COORD_THRESHOLD 0
#define GOAL_TRACK_DIST 10000

// Motors and Encoders
#define MOTOR_FL_PWM 15
#define MOTOR_FL_IN1 27
#define MOTOR_FL_IN2 32
#define MOTOR_FL_ANGLE 300
#define MOTOR_FL_REVERSED false
#define ENC_FL_IN1 1
#define ENC_FL_IN2 0

#define MOTOR_FR_PWM 23
#define MOTOR_FR_IN1 4
#define MOTOR_FR_IN2 5
#define MOTOR_FR_ANGLE 60
#define MOTOR_FR_REVERSED false
#define ENC_FR_IN1 3
#define ENC_FR_IN2 2

#define MOTOR_BL_PWM 14
#define MOTOR_BL_IN1 25
#define MOTOR_BL_IN2 26
#define MOTOR_BL_ANGLE 225
#define MOTOR_BL_REVERSED false
#define ENC_BL_IN1 5
#define ENC_BL_IN2 4

#define MOTOR_BR_PWM 13
#define MOTOR_BR_IN1 18
#define MOTOR_BR_IN2 19
#define MOTOR_BR_ANGLE 135
#define MOTOR_BR_REVERSED false
#define ENC_BR_IN1 7
#define ENC_BR_IN2 6

// Light sensor
#define LS_CALIBRATION_COUNT 10
#define LS_CALIBRATION_BUFFER 300
#define LS_NUM 48
#define LS_NUM_PER_MUX 24 // number of light sensors on each multiplexer
#define LS_NO_LINE_ANGLE 0xBAD
#define LS_NO_LINE_SIZE 0xBAD
#define ADC_SAMPLES 4

#define LS_MUX0_OUT ADC1_CHANNEL_5
#define LS_MUX1_OUT ADC1_CHANNEL_6
#define LS_MUX_S0 2
#define LS_MUX_S1 4
#define LS_MUX_S2 5
#define LS_MUX_S3 23
#define LS_MUX_S4 13
#define LS_MUX_EN 15
#define LS_MUX_WR 14

// TSOPs
#define TSOP_NUM 24
#define TSOP_BEST 5
#define TSOP_TARGET_READS 255
#define TSOP_READ_PERIOD_US 75
// #define TSOP_TIMER_PERIOD 4
#define TSOP_NO_BALL_ANGLE 0xBAD
#define TSOP_MOVAVG_SIZE 4
// #define TSOP_DEBUG // if enabled, prints verbose logging info for the TSOP
#define TSOP_CORRECTION 25 // at 0 degrees TSOPs actually print a different value, so use this to correct it

#define TSOP_MUX_S0 19
#define TSOP_MUX_S1 18
#define TSOP_MUX_S2 17
#define TSOP_MUX_S3 16
#define TSOP_MUX_S4 32
#define TSOP_MUX_OUT 35
#define TSOP_MUX_EN 27
#define TSOP_MUX_WR 26

// IMU
#define IMU_CALIBRATION_COUNT 15
#define IMU_CALIBRATION_TIME 200
#define IMU_THRESHOLD 1000
#define IMU_MULTIPLIER 100.0f

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

// Orbit
#define BALL_FAR_STRENGTH 120
#define BALL_CLOSE_STRENGTH 150
#define ORBIT_SPEED_SLOW 50
#define ORBIT_SPEED_FAST 60

// Attacker FSM defines
#define DRIBBLE_BALL_TOO_FAR 170 // if less than this, switch out of dribble
#define ORBIT_DIST 75 // switch from orbit to pursue if value is less than this
#define IN_FRONT_MIN_ANGLE 25
#define IN_FRONT_MAX_ANGLE 335
#define IDLE_TIME 5000 // ms
#define DRIBBLE_SPEED 100 // speed at which robot dribbles the ball
#define ACCEL_PROG 0.05

// Defence FSM defines
#define DEFEND_DISTANCE 30