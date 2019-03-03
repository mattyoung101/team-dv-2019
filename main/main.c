#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "defines.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include <math.h>
#include "pid.h"
#include "motor.h"

void main_task(void *pvParameter){
    // should be the same as xTaskCreate pcName
    static const char *TAG = "MainTask";

    while (true){
        // esp logging uses printf internally, which prints to UART0, which should already be initialised
        // at this time
        ESP_LOGI(TAG, "Hello info!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "Hello warning!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensor_task(void *pvParameter){
    static const char *TAG = "SensorTask";
    
    while (true){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(){
    nvs_flash_init();
    // set log level for all loggers to the one we specified, TODO only for our loggers?
    esp_log_level_set("*", CONF_LOG_LEVEL);
    uart_set_baudrate(UART_NUM_0, CONF_BAUD);

    // run the tasks
    xTaskCreate(&main_task, "MainTask", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
    //xTaskCreate(&sensor_task, "SensorTask", 512, NULL, configMAX_PRIORITIES, NULL);

    pid_config conf = {
        .kp = 5,
        .kd = 0,
        .ki = 69,
        .absMax = 420
    };
    pid_update(&conf, 42.0, 32.0, 33.0);

    /*
    max - 1 sensor read - small stack size (or big due to buffering?), biggest priority, 
    max - 2 main task (maths, etc) - big stack size, large priority
    max - 4 wifi thread
    max - 6 OTA thread? (make sure to queue update if in game mode, log Rejecting update, game in progress.)
    */
}
