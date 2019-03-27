#include "motor.h"

// static float pwmValues[4];
static float flmotor_speed;
static float frmotor_speed;
static float blmotor_speed;
static float brmotor_speed;

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
        .frequency = 488.28, // Teensy 3.5 runs on 488.28 Hz, change this to play bangers on ya motor controller
        .cmpr_a = 0.0f,
        .cmpr_b = 0.0f,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    // we only use 0A, 0B, 1A, 1B = 2 timers
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &config));

    // just in case?
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0));

    ESP_LOGI("Motor", "Motor init OK");
}

void motor_calc(int16_t angle, int16_t direction, float speed){
    angle = floatMod(450 - angle, 360.0f);

    float sinAngle = sinf(DEG_RAD * angle);
    float cosAngle = cosf(DEG_RAD * angle);

    float alpha = DEG_RAD * MOTOR_FR_ANGLE;
    float beta = DEG_RAD * (180 - MOTOR_BR_ANGLE);

    frmotor_speed = (-cosAngle * (sinf(alpha) + sinf(beta)) + cosf(alpha) * sinAngle + cosf(beta) * 
                    (sinAngle + 2 * direction * sinf(alpha)) + direction * powf(sinf(beta), 2.0) / 
                    (2.0 * (sinf(alpha) + sinf(beta)) * (cosf(alpha) + cosf(beta))));
    brmotor_speed = ((direction - frmotor_speed) * (sinf(alpha) + sinf(beta)) + sinAngle) / (sinf(alpha) + sinf(beta));
    blmotor_speed = direction - frmotor_speed;
    flmotor_speed = direction - brmotor_speed;

    float ratio = speed / fmaxf(fabsf(frmotor_speed), fmaxf(fabsf(brmotor_speed), fmaxf(fabsf(blmotor_speed), 
                fabsf(flmotor_speed))));

    frmotor_speed *= ratio;
    brmotor_speed *= ratio;
    blmotor_speed *= ratio;
    flmotor_speed *= ratio;
}

void motor_write_controller(float speed, gpio_num_t inOnePin, gpio_num_t inTwoPin, gpio_num_t pwmPin, 
                            bool reversed, bool brake){
        // constrained speed shorthand
        float s = constrain(abs(speed), 0.0f, 100.0f);

        // write to the right MCPWM pin
        switch (pwmPin){
            case MOTOR_FL_PWM:
                // front left, 0A
                ESP_LOGD("Motor", "Writing to UNIT_0, TIMER_0, OPR_A (front left), speed: %f", s);
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, s);
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
                break;
            case MOTOR_FR_PWM:
                // front right, 0B
                ESP_LOGD("Motor", "Writing to UNIT_0, TIMER_0, OPR_B (front right)");
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, s);
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
                break;
            case MOTOR_BL_PWM:
                // back left, 1A
                ESP_LOGD("Motor", "Writing to UNIT_0, TIMER_1, OPR_A (back left)");
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, s);
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
                break;
            case MOTOR_BR_PWM:
                // back right, 1B
                ESP_LOGD("Motor", "Writing to UNIT_0, TIMER_1, OPR_B (back right)");
                mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, s);
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
                break;
            default:
                // should only happen if something's really gone wrong
                ESP_LOGE("Motor", "Illegal PWM pin: %d", pwmPin);
                return;
        }

        gpio_set_level(inOnePin, 0);
        gpio_set_level(inTwoPin, 0);
        // gpio_set_level(pwmPin, 1);
        
        // TODO refactor this section
        if (speed > 0){
            ESP_LOGD("Motor", "Speed is > 0");
            if (reversed){
                ESP_LOGD("Motor", "Reversed: 1, 0");
                gpio_set_level(inOnePin, 1);
                gpio_set_level(inTwoPin, 0);
            } else {
                ESP_LOGD("Motor", "Not revered, 0, 1");
                gpio_set_level(inOnePin, 0);
                gpio_set_level(inTwoPin, 1);
            }
        } else if (speed < 0){
            ESP_LOGD("Motor", "Speed is < 0");
            if (reversed){
                ESP_LOGD("Motor", "Reversed, 0, 1");
                gpio_set_level(inOnePin, 0);
                gpio_set_level(inTwoPin, 1);
            } else {
                ESP_LOGD("Motor", "Not revered, 1, 0");
                gpio_set_level(inOnePin, 1);
                gpio_set_level(inTwoPin, 0);
            }
        } else {
            ESP_LOGD("Motor", "Speed IS zero");
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
    motor_write_controller(flmotor_speed, MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FL_REVERSED, brake);
    motor_write_controller(frmotor_speed, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM, MOTOR_FR_REVERSED, brake);
    motor_write_controller(blmotor_speed, MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM, MOTOR_BL_REVERSED, brake);
    motor_write_controller(brmotor_speed, MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM, MOTOR_BR_REVERSED, brake);
}

void motor_run_pwm(float speed){
    motor_write_controller(speed, MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FL_REVERSED, false);
    motor_write_controller(speed, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM, MOTOR_FR_REVERSED, false);
    motor_write_controller(speed, MOTOR_BL_IN1, MOTOR_BL_IN2, MOTOR_BL_PWM, MOTOR_BL_REVERSED, false);
    motor_write_controller(speed, MOTOR_BR_IN1, MOTOR_BR_IN2, MOTOR_BR_PWM, MOTOR_BR_REVERSED, false);
}