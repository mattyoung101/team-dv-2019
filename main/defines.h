#pragma once
#include "esp_log.h"

// Uncomment ONE of these to identify the device as either a master or a slave.
// #define NVS_WRITE_MASTER
// #define NVS_WRITE_SLAVE

// If this is defined, the value of the robot number will be written to NVS
// #define NVS_WRITE_ROBOTNUM 0 // 0 or 1, 0 = bluetooth acceptor (master), 1 = bluetooth initiator (slave)

// FreeRTOS
#define SEMAPHORE_UNLOCK_TIMEOUT 25 // ms
#define CONF_LOG_LEVEL ESP_LOG_DEBUG

// Bluetooth
#define ROBOT0_NAME "DeusVult_Robot0"
#define ROBOT1_NAME "DeusVult_Robot1"
#define SPP_NAME "DeusVult_SPP"
#define PACKET_QUEUE_LENGTH 1
#define BT_CONF_RES_STATIC 0 // uses pre-defined roles to resolve conflicts
#define BT_CONF_RES_DYNAMIC 1 // uses ball data to resolve conflicts
#define BT_PACKET_TIMEOUT 1500 // ms, if we haven't received a packet in this long, other robot is off for damage
#define BT_SWITCH_COOLDOWN 2500 // ms, wait this many ms after a switch before anotehr switch is allowed
// #define BLUETOOTH_ENABLED
// #define BT_SWITCHING_ENABLED // if Bluetooth role switching is enabled or not (defender damage switch always runs)
#define BT_CONF_RES_MODE BT_CONF_RES_STATIC
#define DEFENCE false

// I2C
#define I2C_ESP_SLAVE_ADDR 0x23
#define I2C_NANO_SLAVE_ADDR 0x12
#define I2C_TIMEOUT 250 // ms
#define I2C_ACK_MODE 0x1 // 0x0 to disable ack
#define I2C_BEGIN_DEFAULT 0xB // default packet, has sensor data
#define I2C_BEGIN_DEBUG 0xC // debug packet, has raw data for sending to webserver
#define NANO_PACKET_SIZE 11 // size of packet coming from Nano LS slave, including start byte

// Goals
#define GOAL_YELLOW 0
#define GOAL_BLUE 1
#define GOAL_OFF 2
#define HALFWAY_DISTANCE 45
#define COORD_THRESHOLD 0
#define GOAL_TRACK_DIST 10000 // If the goal distance is less than this, track the goal
#define IDLE_MIN_SPEED 0 // The lowest speed for which the robot will move while positioning
#define GOAL_TOO_CLOSE 30
#define GOAL_WIDTH 40
#define ENEMY_GOAL GOAL_YELLOW

// Protobuf
#define PROTOBUF_SIZE 64 // size of protobuf input/output buffer, make it a safe size to avoid buffer overflows
#define I2C_BUF_SIZE 128 // size of I2C buffer
#define MSG_SENSORUPDATE_ID 0 // should probably make these an enum
#define MSG_BTPROVIDE_ID 1

// Music
#define MUSIC_BPM 100

// PIDs

// --- IMU Correction --- //
// Note: this needs to be reversed (-pid_update)
#define HEADING_KP 0.5
#define HEADING_KI 0
#define HEADING_KD 0.05
#define HEADING_MAX_CORRECTION 100

#define LINEAVOID_KP 100
#define LINEAVOID_KI 0
#define LINEAVOID_KD 0
#define LINEAVOID_MAX 60

// --- Idle Correction --- //
#define IDLE_KP 0.3
#define IDLE_KI 0
#define IDLE_KD 0.042
#define IDLE_MAX_CORRECTION 100

// --- Goalie PIDs --- //
#define FORWARD_KP 3.5
#define FORWARD_KI 0
#define FORWARD_KD 0
#define FORWARD_MAX 100

#define SIDE_KP 1
#define SIDE_KI 0
#define SIDE_KD 0
#define SIDE_MAX 100

#define INTERCEPT_KP 2
#define INTERCEPT_KI 0
#define INTERCEPT_KD 0
#define INTERCEPT_MAX 60
#define INTERCEPT_MIN 0

#define GOALIE_KP 1.5
#define GOALIE_KI 0
#define GOALIE_KD 0.1
#define GOALIE_MAX 100

// --- Coordinate PID --- //
// Note: doesn't fucking work
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
#define GOAL_KD 0.03
#define GOAL_MAX_CORRECTION 100

// Maths
#define PI 3.14159265358979323846
#define E 2.71828182845904523536
#define DEG_RAD 0.017453292519943295 // multiply to convert degrees to radians
#define RAD_DEG 57.29577951308232 // multiply to convert radians to degrees

// AutoMode (code that automatically starts attack/defence tasks based on NVS)
#define AUTOMODE_SLAVE 0
#define AUTOMODE_MASTER 1

// Camera
#define CAM_DATA_LEN 8
#define CAM_BEGIN_BYTE 0xB
#define CAM_END_BYTE 0xE
#define CAM_FRAME_WIDTH 0
#define CAM_FRAME_HEIGHT 0
extern int16_t CAM_OFFSET_X;
extern int16_t CAM_OFFSET_Y;
#define CAM_ANGLE_OFFSET 0
#define CAM_NO_VALUE 0xBAD
#define CAM_UART_TX 17
#define CAM_UART_RX 16

// Motors and Encoders
#define MOTOR_FL_PWM 15
#define MOTOR_FL_IN1 27
#define MOTOR_FL_IN2 32
#define MOTOR_FL_ANGLE 300
extern bool MOTOR_FL_REVERSED;

#define MOTOR_FR_PWM 23
#define MOTOR_FR_IN1 4
#define MOTOR_FR_IN2 5
#define MOTOR_FR_ANGLE 60
extern bool MOTOR_FR_REVERSED;

#define MOTOR_BL_PWM 14
#define MOTOR_BL_IN1 25
#define MOTOR_BL_IN2 26
#define MOTOR_BL_ANGLE 225
extern bool MOTOR_BL_REVERSED;

#define MOTOR_BR_PWM 13
#define MOTOR_BR_IN1 18
#define MOTOR_BR_IN2 19
#define MOTOR_BR_ANGLE 135
extern bool MOTOR_BR_REVERSED;

#define TORQUE_SCALAR 1
#define FRONT_MOTOR_ANGLE 60
#define BACK_MOTOR_ANGLE 45
#define MOTOR_SPEED_CAP 100.0f // max speed motors are allowed to go to in %

// Light sensor
#define LS_CALIBRATION_COUNT 10
#define LS_MUX0_BUFFER 100
#define LS_MUX1_BUFFER 5
#define LS_NUM 48
#define LS_NUM_PER_MUX 24 // number of light sensors on each multiplexer
#define LS_NO_LINE_ANGLE 400
#define LS_NO_LINE_SIZE 400
#define ADC_SAMPLES 1
#define LS_ES_DEFAULT 69
#define LS_TIMER_PERIOD 1000 // microseconds

// Line avoid settings
#define LS_LINE_OVER_BUFFER 80
#define LINE_BIG_SIZE 0
#define LINE_SMALL_SIZE 0
#define LINE_AVOID_SPEED 100
#define LINE_TRACK_SPEED 30
#define LINE_SPEED_MULTIPLIER 0.5
#define LINE_AVOID_TIME 1500 // ms

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
#define TSOP_NUM 24 // total number of TSOPs
#define TSOP_BEST 5 // pick the TSOP_BEST number of TSOPs to calculate with
#define TSOP_TARGET_READS 100 // number of reads to do per slave task loop
#define TSOP_NO_BALL_ANGLE 0xBAD
#define TSOP_MOVAVG_SIZE 4
// #define TSOP_DEBUG // if enabled, prints verbose logging info for the TSOP
extern int16_t TSOP_CORRECTION; // at 0 degrees TSOPs actually print a different value, so use this to correct it
#define TSOP_SCALING true

#define TSOP_MUX_S0 19
#define TSOP_MUX_S1 18
#define TSOP_MUX_S2 17
#define TSOP_MUX_S3 16
#define TSOP_MUX_S4 32
#define TSOP_MUX_OUT 35
#define TSOP_MUX_EN 27
#define TSOP_MUX_WR 26

// individual TSOP calibration for each sensor
extern float TSOP_TUNING[TSOP_NUM];

// IMU
#define IMU_CALIBRATION_COUNT 80
#define IMU_CALIBRATION_TIME 10
#define IMU_THRESHOLD 1000
#define I2C_MULTIPLIER 100.0f

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

#define DMP_RATE 200 // DMP update rate in Hz, max is 200

// Orbit
extern uint8_t BALL_FAR_STRENGTH;
extern uint8_t BALL_CLOSE_STRENGTH;
extern uint8_t ORBIT_SPEED_SLOW;
extern uint8_t ORBIT_SPEED_FAST;
#define ORBIT_SlOW_ANGLE_MIN 45
#define ORBIT_SLOW_ANGLE_MAX 360 - ORBIT_SlOW_ANGLE_MIN
#define ORBIT_SLOW_STRENGTH 150
#define ORBIT_SLOW_SPEED_THING 20

extern float ORBIT_CONST;

// Attacker FSM defines
extern uint16_t DRIBBLE_BALL_TOO_FAR; // if less than this, switch out of dribble
extern uint16_t ORBIT_DIST;  // switch from orbit to pursue if value is more than this
extern uint16_t IN_FRONT_MIN_ANGLE; // angle range in which the ball is considered to be in front of the robot
extern uint16_t  IN_FRONT_MAX_ANGLE;
#define IN_FRONT_ANGLE_BUFFER 40
#define IN_FRONT_STRENGTH_BUFFER 30
#define IDLE_TIMEOUT 3000 // if ball is not visible for this length of time in ms or more, switch to idle state
#define IDLE_DISTANCE 40
#define IDLE_OFFSET 0
#define DRIBBLE_TIMEOUT 100 // ms, if robot sees ball in this position for this time it will switch to dribble state
#define DRIBBLE_SPEED 80 // speed at which robot dribbles the ball, out of 100
#define ACCEL_PROG 0.01 // update the acceleration interpolation by this amount per tick, 1 tick is about 10ms, so 0.01 will accelerate completely in 1 second
#define GOAL_MIN_ANGLE 30
#define GOAL_MAX_ANGLE 330
#define GOAL_SHOOT_DIST 40 // if we are within this distance, shoot

// Defence FSM defines
extern uint8_t DEFEND_DISTANCE;
extern uint8_t SURGE_DISTANCE;
extern uint8_t SURGE_STRENGTH;
#define SURGE_SPEED 100
#define REVERSE_SPEED 60
#define DEFEND_MIN_STRENGTH 70
#define DEFEND_MAX_ANGLE 120
#define DEFEND_MIN_ANGLE 250
#define KICKER_STRENGTH 100 // if ball strength greater than this, kick
#define SURGEON_ANGLE_MIN 6 // angles to surge between
#define SURGEON_ANGLE_MAX 360 - SURGEON_ANGLE_MIN
#define SURGE_CAN_KICK_TIMEOUT 500 // ms to be in surge for before we can kick

// General FSM defines
#define MODE_ATTACK 0
#define MODE_DEFEND 1
extern uint8_t ROBOT_MODE;

// Kicker
#define KICKER_PIN 33
#define KICKER_DELAY 10 // ms to wait between solenoid activation and deactivation
#define SHOOT_TIMEOUT 1000 // ms until we are allowed to kick again

// RGB LEDs
#define LED_PIN 13
#define LED_NUM 12
#define RAINBOW_TRANSITION_TIME 0.1f // seconds
#define DEBUG_LED_1 23 // supposed to be orange
#define DEBUG_LED_2 25 // supposed to be white

/** initialises per robot values */
void defines_init(uint8_t robotId);