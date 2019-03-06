#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "defines.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include <math.h>
#include "pid.h"
#include "motor.h"
#include "light.h"
#include "tsop.h"
#include <str.h>

// note: portTICK_PERIOD_MS is equivalent to 1000 / 100 = 10

void master_task(void *pvParameter){
    // should be the same as xTaskCreate pcName
    static const char *TAG = "MasterTask";

    // Initialise hardware
    motor_init();

    while (true){
        ESP_LOGI(TAG, "Hello info!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "Hello warning!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*
        loop will be like:
        1. check mailbox (xTaskNotify notification bit) and process it
        */
    }
}

void slave_task(void *pvParameter){
    //static const char *TAG = "SlaveTask";

    ls_init_adc();
    lsarray_init();
    tsop_init();
    
    while (true){
        tsop_update_once();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// can be used by multiple timer instances
void timer_callback(TimerHandle_t timer){
    uint32_t timerId = (uint32_t) pvTimerGetTimerID(timer);

    if (timerId == TIMER_TSOP){
        tsop_process();
        lsarray_read();
        lsarray_calc_clusters();
        // TODO lsarray_calc_line();
        // time is automatically reset here
    } else {
        ESP_LOGW("TimerCallback", "Unknown timer ID.");
    }
}

void app_main(){
    esp_log_level_set("*", CONF_LOG_LEVEL);

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI("AppMain", "Reflashing NVS");
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // run the tasks, either master task or slave task should be running - but not both!
    // we use xTaskCreatePinnedToCore() because its necessary to get hardware accelerated floating point maths
    // see: https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/freertos-smp.html#floating-point-aritmetic
    xTaskCreatePinnedToCore(&master_task, "MasterTask", 1024, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
    
    // tsop tick period is dank, it's in Us so we convert it to ms and convert that into ticks
    int32_t periodUs = 833 * TSOP_TIMER_PERIOD;
    TimerHandle_t tsopTimer = xTimerCreate("TSOPTimer", (periodUs / 1000) / portTICK_PERIOD_MS, pdTRUE, (void*) TIMER_TSOP, 
                                timer_callback);
    xTimerStart(tsopTimer, 0);
    
    // FreeRTOS will now have full control over the device and which task runs
    vTaskStartScheduler();

    // FreeRTOS docs say if we reach this point something must've gone wrong (insufficient RAM)
    ESP_LOGE("AppMain", "Illegal state: vTaskStartScheduler returned");
    abort();
}