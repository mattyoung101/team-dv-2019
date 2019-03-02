#include "motor.h"

void motor_init_pins(){
    // TODO do we actually need all these error check calls
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FL_PWM, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FL_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FL_IN2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FR_PWM, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FR_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_FR_IN2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BL_PWM, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BL_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BL_IN2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BR_PWM, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BR_IN1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_BR_IN2, GPIO_MODE_OUTPUT));

    // Teensy 3.5 frequency: 488.28 Hz, see https://www.pjrc.com/teensy/td_pulse.html#frequency
    // Setup PWM (LEDC) timer using 8 bit resolution (we only need 0-255 speed) and same frequency as Teensy 3.5
    // for backwards compatibility and using high speed mode for fast hardware switching of duty values
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, 
        .freq_hz = 488,                      
        .speed_mode = LEDC_HIGH_SPEED_MODE,  
        .timer_num = LEDC_TIMER_0            
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void motor_calc(int16_t angle, int16_t direction, int16_t speed){
    // TODO cast required?
    float radAngle = DEG_RAD * (float) angle;

    pwmValues[0] = cosf(((MOTOR_FL_ANGLE + 90) * DEG_RAD) - radAngle);
    pwmValues[1] = cosf(((MOTOR_FR_ANGLE + 90) * DEG_RAD) - radAngle);
    pwmValues[2] = cosf(((MOTOR_BL_ANGLE + 90) * DEG_RAD) - radAngle);
    pwmValues[3] = cosf(((MOTOR_BR_ANGLE + 90) * DEG_RAD) - radAngle);

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
        // set PWM value using LEDC
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, constrain(speed, 0, 255)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
        
        // TODO refactor this method
        if (speed > 0){
            if (reversed){
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 1));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 0));
            } else {
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 0));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 1));
            }
        } else if (speed < 0){
            if (reversed){
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 0));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 1));
            } else {
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 1));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 0));
            }
        } else {
            if (brake){
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 0));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 0));
                ESP_ERROR_CHECK(gpio_set_level(pwmPin, 1));
            } else {
                ESP_ERROR_CHECK(gpio_set_level(inOnePin, 1));
                ESP_ERROR_CHECK(gpio_set_level(inTwoPin, 1));
                ESP_ERROR_CHECK(gpio_set_level(pwmPin, 1));
            }
        }
}