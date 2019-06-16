#pragma once
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "defines.h"
#include "utils.h"
#include "esp_err.h"
#include "driver/mcpwm.h"

// Ported from Move.cpp

/** Initialies motor pin direction **/
void motor_init(void);
/** Calculates PWM values for each motor **/
void motor_calc(int16_t direction, int16_t orientation, float speed);
/** Handles writing to the motor controller **/
void motor_write_controller(float speed, gpio_num_t inOnePin, gpio_num_t inTwoPin, gpio_num_t pwmPin, bool reversed, bool brake);
/** Writes to each controller to move all motors **/
void motor_move(bool brake);
/** Writes the given PWM to all controllers **/
void motor_run_pwm(float pwm);