#include "pid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <inttypes.h>

static int64_t lastTime = 0;
static float integral = 0;
static float lastInput = 0;

float pid_update(pid_config_t *conf, float input, float setpoint, float modulus){
    // printf("PID inputs: confP %f, confI %f, confD %f, confAbsMax %f, input: %f, setpoint: %f, modulus: %f\n",
    // conf.kp, conf.ki, conf.kd, conf.absMax, input, setpoint, modulus);

    float derivative = 0.0f;
    float error = setpoint - input;

    int64_t currentTime = esp_timer_get_time();
    float elapsedTime = (float) (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    integral += elapsedTime * error;

    if (modulus != 0.0f){
        float difference = (input - conf->lastInput);
        if (difference < -modulus) {
            difference += modulus;
        } else if (difference > modulus) {
            difference -= modulus;
        }
        // printf("Difference 1: %f\n", difference);

        derivative = difference / elapsedTime;
    } else {
        derivative = (input - conf->lastInput) / elapsedTime;
        // printf("(%f - %f) / %f\n", input, conf->lastInput, elapsedTime);
    }

    conf->lastInput = input;

    float correction = (conf->kp * error) + (conf->ki * integral) - (conf->kd * derivative);
    // printf("Derivative: %f, Error: %f, elapsed time: %f, integral: %f, correction: %f, lastInput: %f, lastTime: %f\n", 
    // derivative, error, elapsedTime, integral, correction, (float) lastInput, (float) lastTime);
    return conf->absMax == 0 ? correction : constrain(correction, -conf->absMax, conf->absMax);
}