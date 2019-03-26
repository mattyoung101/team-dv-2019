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
#include "soc/efuse_reg.h"
#include "comms_i2c.h"

// This file is the main entry point of the robot that creates FreeRTOS tasks which update everything else

static uint8_t mode = AUTOMODE_ILLEGAL;

// Task which runs on the master. Receives sensor data from slave and handles complex routines
// like moving, finite state machines, Bluetooth, etc
void master_task(void *pvParameter){
    static const char *TAG = "MasterTask";

    // Initialise hardware
    motor_init();
    comms_i2c_init_slave();
    ESP_LOGI(TAG, "Master hardware init OK");

    while (true){
        // printf("BACKWARDS\n");
        // motor_run_pwm(-20.0);
        // vTaskDelay(pdMS_TO_TICKS(2500));

        // printf("FORWARDS\n");
        // motor_run_pwm(20.0);
        // vTaskDelay(pdMS_TO_TICKS(2500));
        vTaskDelay(portMAX_DELAY);
    }
}

// Task which runs on the slave. Reads and calculates sensor data, then sends to master.
void slave_task(void *pvParameter){
    static const char *TAG = "SlaveTask";

    // Initialise hardware
    comms_i2c_init_master();
    ESP_LOGI(TAG, "Slave hardware init OK");
    
    while (true){
        comms_i2c_send(1234, 4321, 1111, 2222);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Callback for all timers. Should only run on the slave.
void timer_callback(TimerHandle_t timer){
    uint32_t timerId = (uint32_t) pvTimerGetTimerID(timer);

    if (timerId == TIMER_TSOP && mode == AUTOMODE_SLAVE){
        // read sensors
        // tsop_process();
        // tsop_calc(5); 

        // lsarray_read();
        // lsarray_calc_clusters();
        // lsarray_calc_line();

        // transmit back to master?
    } else {
        ESP_LOGW("TimerCallback", "Unknown timer ID given mode (id: %d, mode: %d)", timerId, mode);
    }
}

// Handles both HTTP, WebSocket and OTA updates. Bluetooth still runs on the master task.
void network_task(void *pvParameter){
    static const char *TAG = "NetworkTask";
    
    // init websockets and wifi here

    ESP_LOGI(TAG, "Networking init OK");
    while (true){
        // serve websockets, in this case just block forever
        vTaskDelay(portMAX_DELAY);
    }
}

void app_main(){
    esp_log_level_set("*", ESP_LOG_INFO);

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI("AppMain", "Reflashing NVS");
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // AutoMode: automatically assign to slave/master depending on a value set in NVS
    nvs_handle storageHandle;
    ESP_ERROR_CHECK(nvs_open("RobotSettings", NVS_READWRITE, &storageHandle));

    // if set, write out Master or Slave to NVS on first boot
    #ifdef NVS_WRITE_MASTER
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_MASTER));
        ESP_ERROR_CHECK(nvs_commit(storageHandle));
        ESP_LOGE("AutoMode", "Successfully wrote Master to NVS.\n");
    #elif defined NVS_WRITE_SLAVE
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_SLAVE));
        ESP_ERROR_CHECK(nvs_commit(storageHandle));
        ESP_LOGE("AutoMode", "Successfully wrote Slave to NVS.\n");
    #endif

    err = nvs_get_u8(storageHandle, "Mode", &mode);
    nvs_close(storageHandle);

    if (err == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE("AutoMode", "Key not found! Please identify this device, see main.c for help. Cannot continue.");
        abort();
    } else if (err != ESP_OK) {
        ESP_LOGE("AutoMode", "Unexpected error reading key: %s. Cannot continue.", esp_err_to_name(err));
        abort();
    }
    fflush(stdout);

    // we use xTaskCreatePinnedToCore() because its necessary to get hardware accelerated floating point maths
    // see: https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/freertos-smp.html#floating-point-aritmetic
    // also according to a forum post, going against the FreeRTOS norm, stack size is in fact in bytes, NOT words!
    // source: https://esp32.com/viewtopic.php?t=900#p3879
    if (mode == AUTOMODE_MASTER){
        ESP_LOGI("AppMain", "Running as master");
        xTaskCreatePinnedToCore(master_task, "MasterTask", 4096, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
        #ifdef WEBSOCKET_ENABLED
            ESP_LOGI("AppMain", "Running networking");
            // controlling the robot is much more important than sending/receiving miscellaneous data
            // but the webserver requires a bigger stack size
            xTaskCreatePinnedToCore(network_task, "NetworkTask", 8192, NULL, configMAX_PRIORITIES - 4, NULL, PRO_CPU_NUM);
        #endif
    } else {
        ESP_LOGI("AppMain", "Running as slave");
        xTaskCreatePinnedToCore(slave_task, "SlaveTask", 4096, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);  
        
        // TSOP timer, goes off on the slave when its time to read
        // TODO this aborts the device
        // int32_t periodUs = 833 * TSOP_TIMER_PERIOD;
        // TimerHandle_t tsopTimer = xTimerCreate("TSOPTimer", pdMS_TO_TICKS(periodUs / 1000), pdTRUE, 
        //                             (void*) TIMER_TSOP, timer_callback);
        // xTimerStart(tsopTimer, 0);
    }
}