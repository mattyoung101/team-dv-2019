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
#include "fsm.h"
#include "cam.h"
#include "states.h"

// note: portTICK_PERIOD_MS is equivalent to 1000 / 100 = 10
// This file is the main entry point of the robot that creates FreeRTOS tasks which update everything else

static uint8_t mode = AUTOMODE_ILLEGAL;

void master_task(void *pvParameter){
    static const char *TAG = "MasterTask";

    // Initialise hardware
    motor_init();
    cam_init();

    // Initialise software controllers
    state_machine machine;

    while (true){
        ESP_LOGI(TAG, "Hello info!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "Hello warning!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // communicate with slave here, might be done in separate task

        // read sensors
        cam_update();

        // update the FSM and other controllers
        fsm_update(&machine);
    }
}

void slave_task(void *pvParameter){
    //static const char *TAG = "SlaveTask";

    // Initialise hardware
    ls_init_adc();
    lsarray_init();
    tsop_init();
    
    while (true){
        tsop_update();

        // send over i2c back to master (separate thread????)
    }
}

// can be used by multiple timer instances
void timer_callback(TimerHandle_t timer){
    uint32_t timerId = (uint32_t) pvTimerGetTimerID(timer);

    if (timerId == TIMER_TSOP && mode == AUTOMODE_SLAVE){
        // read sensors
        tsop_process();
        tsop_calc(5); 

        lsarray_read();
        lsarray_calc_clusters();
        lsarray_calc_line();

        // transmit back to master
    } else {
        ESP_LOGW("TimerCallback", "Unknown timer ID given mode (id: %d, mode: %d)", timerId, mode);
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

    // AutoMode: automatically assign to slave/master depending on a value set in NVS
    nvs_handle storageHandle;
    ESP_ERROR_CHECK(nvs_open("RobotSettings", NVS_READWRITE, &storageHandle));

    // we may wish to write out Master or Slave to NVS on first boot
    #ifdef NVS_WRITE_MASTER
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_MASTER));
        ESP_ERROR_CHECK(nvs_commit(storageHandle));
    #elif NVS_WRITE_SLAVE
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_SLAVE));
        ESP_ERROR_CHECK(nvs_commit(storageHandle));
    #endif

    err = nvs_get_u8(storageHandle, "Mode", &mode);

    if (err == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE("AutoMode", "Key not found! Assuming master.");
        mode = AUTOMODE_MASTER;
    } else if (err != ESP_OK) {
        ESP_LOGE("AutoMode", "Unexpected error reading key: %s. Assuming master.", esp_err_to_name(err));
        mode = AUTOMODE_MASTER;
    }
    nvs_close(storageHandle);

    // we use xTaskCreatePinnedToCore() because its necessary to get hardware accelerated floating point maths
    // see: https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/freertos-smp.html#floating-point-aritmetic
    // also according to a forum post, going against the FreeRTOS norm, stack size is in fact in bytes, NOT words!
    // source: https://esp32.com/viewtopic.php?t=900#p3879
    // therefore we use a 4K stack (remember we have like 512K RAM)
    if (mode == AUTOMODE_MASTER){
        xTaskCreatePinnedToCore(master_task, "MasterTask", 4096, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
    } else {
        xTaskCreatePinnedToCore(slave_task, "SlaveTask", 4096, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);  
        
        // TSOP timer, goes off on the slave when its time to read
        int32_t periodUs = 833 * TSOP_TIMER_PERIOD;
        TimerHandle_t tsopTimer = xTimerCreate("TSOPTimer", (periodUs / 1000) / portTICK_PERIOD_MS, pdTRUE, 
                                    (void*) TIMER_TSOP, timer_callback);
        xTimerStart(tsopTimer, 0);
    }
    
    // FreeRTOS will now have full control over the device and which task runs
    vTaskStartScheduler();

    // FreeRTOS docs say if we reach this point something must've gone wrong (insufficient RAM)
    ESP_LOGE("AppMain", "Illegal state: vTaskStartScheduler returned");
    abort();
}