#include "pid.h"

static int64_t lastTime;
static float integral;
static float lastInput;

float pid_update(pid_config_t *conf, float input, float setpoint, float modulus){
    float derivative;
    float error = setpoint - input;

    int64_t currentTime = esp_timer_get_time();
    float elapsedTime = (float) (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    integral += lastTime * error;

    if (modulus != 0.0){
        float difference = (input - lastInput);
        
        if (difference < -modulus) {
            difference += modulus;
        } else if (difference > modulus) {
            difference -= modulus;
        }

        derivative = difference / elapsedTime;
    } else {
        derivative = (input - lastInput) / elapsedTime;
    }

    lastInput = input;

    double correction = conf->kp * error + conf->ki * integral - conf->kd * derivative;
    return conf->absMax == 0 ? correction : constrain(correction, -conf->absMax, conf->absMax);
}