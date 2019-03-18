#include "motor.h"

static float pwmValues[4];
static float flmotor_pwm;
static float frmotor_pwm;
static float blmotor_pwm;
static float brmotor_pwm;

// TODO change this to use MCPWM
void motor_init(void){
    /**
     * Ok, so here's the deal: we have three ways to do PWM: LEDC, MCPWM and Sigma Delta and the best is one MCPWM
     * since it's specifically designed for this task.
     * HOWEVER: it's designed for H-bridges like the L298 which just have 2 pins, but our motor controllers
     * have 3 pins.
     * So what we do instead is a hybrid approach: use MCPWM on all the PWM pins and just regular old
     * GPIO on the IN1 and IN2 pins
     */

    // Setup pins
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_FL_PWM));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FL_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FL_IN2, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_FR_PWM));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FR_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FR_IN2, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_BL_PWM));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BL_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BL_IN2, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_BR_PWM));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BR_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BR_IN2, GPIO_MODE_OUTPUT));

    // Configure MCPWM timer
    mcpwm_config_t config = {
        .frequency = 488.28, // Teensy 3.5 runs on 488.28 Hz
        .cmpr_a = 0.0f,
        .cmpr_b = 0.0f,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &config));
}

/*
NOTES for thursday:
    - we need the four channels of PWM I think in order to get the 4 motor controller pins to do PWM
*/

void motor_calc(int16_t angle, int16_t direction, int8_t speed){
    float radAngle = DEG_RAD * (float) angle;

    pwmValues[0] = cosf(((MOTOR_FL_ANGLE + 90.0f) * DEG_RAD) - radAngle);
    pwmValues[1] = cosf(((MOTOR_FR_ANGLE + 90.0f) * DEG_RAD) - radAngle);
    pwmValues[2] = cosf(((MOTOR_BL_ANGLE + 90.0f) * DEG_RAD) - radAngle);
    pwmValues[3] = cosf(((MOTOR_BR_ANGLE + 90.0f) * DEG_RAD) - radAngle);

    flmotor_pwm = speed * pwmValues[0] + direction;
    frmotor_pwm = speed * pwmValues[1] + direction;
    blmotor_pwm = speed * pwmValues[2] + direction;
    brmotor_pwm = speed * pwmValues[3] + direction;

    float maxSpeed = fmaxf(
        fmaxf(fabsf(flmotor_pwm), fabsf(frmotor_pwm)), 
        fmaxf(fabsf(blmotor_pwm), fabsf(brmotor_pwm)));

    flmotor_pwm = speed == 0 ? flmotor_pwm : (flmotor_pwm / maxSpeed) * speed;
    frmotor_pwm = speed == 0 ? frmotor_pwm : (frmotor_pwm / maxSpeed) * speed;
    blmotor_pwm = speed == 0 ? blmotor_pwm : (blmotor_pwm / maxSpeed) * speed;
    brmotor_pwm = speed == 0 ? brmotor_pwm : (brmotor_pwm / maxSpeed) * speed;
}

void motor_write_controller(int8_t speed, gpio_num_t inOnePin, gpio_num_t inTwoPin, gpio_num_t pwmPin, 
                            bool reversed, bool brake){
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, constrain(speed, (int8_t) 0, (int8_t) 255));
        // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, constrain(speed, (int8_t) 0, (int8_t) 255));
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        
        // TODO figure out the above do we need a set duty high thing
        
        // TODO refactor this section
        if (speed > 0){
            if (reversed){
                gpio_set_level(inOnePin, 1);
                gpio_set_level(inTwoPin, 0);
            } else {
                gpio_set_level(inOnePin, 0);
                gpio_set_level(inTwoPin, 1);
            }
        } else if (speed < 0){
            if (reversed){
                gpio_set_level(inOnePin, 0);
                gpio_set_level(inTwoPin, 1);
            } else {
                gpio_set_level(inOnePin, 1);
                gpio_set_level(inTwoPin, 0);
            }
        } else {
            if (brake){
                gpio_set_level(inOnePin, 0);
                gpio_set_level(inTwoPin, 0);
                gpio_set_level(pwmPin, 1);
            } else {
                gpio_set_level(inOnePin, 1);
                gpio_set_level(inTwoPin, 1);
                gpio_set_level(pwmPin, 1);
            }
        }
}

void motor_move(bool brake){
    motor_write_controller(flmotor_pwm, MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FL_REVERSED, brake);
    motor_write_controller(frmotor_pwm, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM, MOTOR_FR_REVERSED, brake);
    motor_write_controller(blmotor_pwm, MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM, MOTOR_BL_REVERSED, brake);
    motor_write_controller(brmotor_pwm, MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM, MOTOR_BR_REVERSED, brake);
}

void motor_run_pwm(uint8_t pwm){
    motor_write_controller(pwm, MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FL_REVERSED, false);
    motor_write_controller(pwm, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM, MOTOR_FR_REVERSED, false);
    motor_write_controller(pwm, MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM, MOTOR_BL_REVERSED, false);
    motor_write_controller(pwm, MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM, MOTOR_BR_REVERSED, false);
}