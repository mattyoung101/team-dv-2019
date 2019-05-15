#define HANDMADE_MATH_IMPLEMENTATION
#define HANDMADE_MATH_NO_SSE
#include "HandmadeMath.h"
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
#include "motor.h"
#include "light.h"
#include "tsop.h"
#include <str.h>
#include "fsm.h"
#include "cam.h"
#include "states.h"
#include "soc/efuse_reg.h"
#include "comms_i2c.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "simple_imu.h"
#include "pid.h"
#include "vl53l0x_api.h"
#include "ads1015.h"

#if ENEMY_GOAL == GOAL_YELLOW
    #define AWAY_GOAL goalYellow
    #define HOME_GOAL goalBlue
#elif ENEMY_GOAL == GOAL_BLUE
    #define AWAY_GOAL goalBlue
    #define HOME_GOAL goalYellow
#endif

static uint8_t mode = AUTOMODE_ILLEGAL;

// Task which runs on the master. Receives sensor data from slave and handles complex routines
// like moving, finite state machines, Bluetooth, etc
void master_task(void *pvParameter){
    static const char *TAG = "MasterTask";

    // Initialise hardware
    motor_init();
    comms_i2c_init_slave();
    // comms_wifi_init_host();
    cam_init();
    ESP_LOGI(TAG, "Master hardware init OK");

    // Initialise software controllers
    state_machine_t stateMachine = {0};
    stateMachine.currentState = &stateGeneralNothing;
    robotStateSem = xSemaphoreCreateMutex();
    xSemaphoreGive(robotStateSem);

    // we do it like this to make sure that pursue_enter is called
    fsm_change_state(&stateMachine, &stateAttackIdle);

    esp_task_wdt_add(NULL);

    while (true){
        // update cam
        cam_calc();

        // update values for FSM
        if (xSemaphoreTake(robotStateSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT)) && 
            xSemaphoreTake(rdSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT)) && 
            xSemaphoreTake(goalDataSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                // reset out values
                robotState.outShouldBrake = false;
                robotState.outOrientation = 0;
                robotState.outDirection = 0;

                // update
                robotState.inBallAngle = (receivedData.tsopAngle - TSOP_CORRECTION) % 360;
                robotState.inBallStrength = receivedData.tsopStrength;

                robotState.inGoalVisible = robotState.outIsAttack ? AWAY_GOAL.exists : HOME_GOAL.exists;
                robotState.inGoalAngle = robotState.outIsAttack ? AWAY_GOAL.angle + CAM_ANGLE_OFFSET : HOME_GOAL.angle 
                                        + CAM_ANGLE_OFFSET;
                robotState.inGoalLength = robotState.outIsAttack ? (int16_t) AWAY_GOAL.length : HOME_GOAL.length;
                robotState.inGoalDistance = robotState.outIsAttack ? AWAY_GOAL.distance : HOME_GOAL.distance;
                // hack to convert to IMU data to float by multiplying it by 100 before sending then diving it
                robotState.inHeading = receivedData.heading / IMU_MULTIPLIER;
                robotState.inX = robotX;
                robotState.inY = robotY;

                // unlock semaphores
                xSemaphoreGive(robotStateSem);
                xSemaphoreGive(rdSem);
                xSemaphoreGive(goalDataSem);
        } else {
            ESP_LOGW(TAG, "Failed to acquire semaphores, cannot update FSM data.");
        }

        // update the actual FSM
        fsm_update(&stateMachine);

        // run motors
        motor_calc(robotState.outDirection, robotState.outOrientation, robotState.outSpeed);
        motor_move(robotState.outShouldBrake);

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task which runs on the slave. Reads and calculates sensor data, then sends to master.
void slave_task(void *pvParameter){
    static const char *TAG = "SlaveTask";

    // Initialise hardware
    comms_i2c_init_master(I2C_NUM_0);
    i2c_scanner();

    puts("Initialising TSOP");
    tsop_init();
    puts("Initialising LS");
    ls_init();
    puts("Initialising IMU");
    simu_init();
    simu_calibrate();

    ESP_LOGI(TAG, "Slave hardware init OK");
    esp_task_wdt_add(NULL);
    
    while (true) {
        for (int i = 0; i < 255; i++){
            tsop_update(NULL);
        }
        tsop_calc();

        // lsarray_read();
        // lsarray_debug();

        simu_calc();

        comms_i2c_send((uint16_t) tsopAngle, (uint16_t) tsopStrength, (uint16_t) LS_NO_LINE_ANGLE, (uint16_t) LS_NO_LINE_SIZE, (uint16_t) (heading * IMU_MULTIPLIER));

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(0));
    }
}

void app_main(){
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
    #else
        ESP_LOGI("AutoMode", "No read/write performed");
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
        xTaskCreatePinnedToCore(master_task, "MasterTask", 8192, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
    } else {
        ESP_LOGI("AppMain", "Running as slave");
        xTaskCreatePinnedToCore(slave_task, "SlaveTask", 8192, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);  
    }
}