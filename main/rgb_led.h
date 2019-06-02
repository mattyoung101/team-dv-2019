#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <stdio.h>
#include "defines.h"
#include "led_strip.h"
#include "esp_task_wdt.h"
#include "utils.h"

typedef enum {
    /** rainbow that plays during the boot sequence and calibrations (also great for celebrating pride month with) **/
    RGB_LED_MODE_RAINBOW = 0,
    /** lights point towards where the robot thinks the ball is according to TSOP data **/
    RGB_LED_MODE_TSOP,
    /** similar to RGB_LED_MODE_TSOP, but is used to visualise light sensor clusters **/
    RGB_LED_MODE_LS,
    /** yellow lights point towards yellow goal, blue lights point towards blue goal **/
    RGB_LED_MODE_GOAL,
    /** lights point towards where the IMU's relative forward is **/
    RGB_LED_MODE_IMU,
    /** similar to the Xbox 360 red ring of death, a ring that trails around the robot in case of error **/
    RGB_LED_MODE_ERROR

} rgb_led_mode_t;

/** Initialises the RGB LED controller task and driver **/
void rgb_led_init();
/** Changes RGB LED mode **/
void rgb_led_change_mode(rgb_led_mode_t mode_);