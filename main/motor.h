#pragma once
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "defines.h"
#include "utils.h"
#include "esp_err.h"

// Ported from Move.cpp

float pwmValues[4];
float flmotor_pwm;
float frmotor_pwm;
float blmotor_pwm;
float brmotor_pwm;

/** Initialies motor pin direction **/
void motor_init(void);
/** Calculates PWM values for each motor **/
void motor_calc(int16_t angle, int16_t direction, int8_t speed);
/** Handles writing to the motor controller **/
void motor_write_controller(int8_t speed, gpio_num_t inOnePin, gpio_num_t inTwoPin, gpio_num_t pwmPin, bool reversed, bool brake);
/** Writes to each controller to move all motors **/
void motor_move(bool brake);