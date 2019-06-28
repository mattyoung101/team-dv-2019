#define HANDMADE_MATH_IMPLEMENTATION
#define HANDMADE_MATH_NO_SSE
#define _GNU_SOURCE
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
#include "comms_bluetooth.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "i2c.pb.h"
#include "lrf.h"

#if ENEMY_GOAL == GOAL_YELLOW
    #define AWAY_GOAL goalYellow
    #define HOME_GOAL goalBlue
#elif ENEMY_GOAL == GOAL_BLUE
    #define AWAY_GOAL goalBlue
    #define HOME_GOAL goalYellow
#endif

static uint8_t mode = 69; // start out with invalid mode
state_machine_t *stateMachine = NULL;

// Task which runs on the master. Receives sensor data from slave and handles complex routines
// like moving, finite state machines, Bluetooth, etc
static void master_task(void *pvParameter){
    static const char *TAG = "MasterTask";
    uint8_t robotId = 69;

    // Initialise comms and hardware
    motor_init();
    comms_i2c_init_slave();
    cam_init();
    gpio_set_direction(KICKER_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "=============== Master hardware init OK ===============");

    // read robot ID from NVS and init Bluetooth
    nvs_get_u8_graceful("RobotSettings", "RobotID", &robotId);
    defines_init(robotId);
    ESP_LOGI(TAG, "Running as robot #%d", robotId);
    robotState.inRobotId = robotId;

    #ifdef BLUETOOTH_ENABLED
    if (robotId == 0){
        comms_bt_init_master();
    } else {
        comms_bt_init_slave();
    }
    #endif

    // Initialise FSM, start out in defence until we get a BT connection
    #ifdef BLUETOOTH_ENABLED
    stateMachine = fsm_new(&stateDefenceDefend);
    #else
    stateMachine = fsm_new(&stateAttackPursue);
    #endif

    // Wait for the slave to calibrate IMU and send over the first packets
    ESP_LOGI(TAG, "Waiting for slave IMU calibration to complete...");
    vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_COUNT * IMU_CALIBRATION_TIME + 1000));
    ESP_LOGI(TAG, "Running!");

    esp_task_wdt_add(NULL);

    while (true){
        // update cam
        cam_calc();

        // update values for FSM, mutexes are used to prevent race conditions
        if (xSemaphoreTake(robotStateSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT)) && 
            xSemaphoreTake(pbSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT)) && 
            xSemaphoreTake(goalDataSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                // reset out values
                robotState.outShouldBrake = false;
                robotState.outOrientation = 0;
                robotState.outDirection = 0;
                robotState.outSwitchOk = false;

                // update FSM values
                robotState.inBallAngle = floatMod(lastSensorUpdate.tsopAngle + TSOP_CORRECTION, 360.0f);
                robotState.inBallStrength = lastSensorUpdate.tsopStrength;
                // TODO make goal stuff floats as well
                if (robotState.outIsAttack){
                    robotState.inGoalVisible = AWAY_GOAL.exists;
                    robotState.inGoalAngle = AWAY_GOAL.angle + CAM_ANGLE_OFFSET;
                    robotState.inGoalLength = (int16_t) AWAY_GOAL.length;
                    robotState.inGoalDistance = AWAY_GOAL.distance;

                    robotState.inOtherGoalVisible = HOME_GOAL.exists;
                    robotState.inOtherGoalAngle = HOME_GOAL.angle + CAM_ANGLE_OFFSET;
                    robotState.inOtherGoalLength = (int16_t) HOME_GOAL.length;
                    robotState.inOtherGoalDistance = HOME_GOAL.distance;
                } else {
                    robotState.inOtherGoalVisible = AWAY_GOAL.exists;
                    robotState.inOtherGoalAngle = AWAY_GOAL.angle + CAM_ANGLE_OFFSET;
                    robotState.inOtherGoalLength = (int16_t) AWAY_GOAL.length;
                    robotState.inOtherGoalDistance = AWAY_GOAL.distance;

                    robotState.inGoalVisible = HOME_GOAL.exists;
                    robotState.inGoalAngle = HOME_GOAL.angle + CAM_ANGLE_OFFSET;
                    robotState.inGoalLength = (int16_t) HOME_GOAL.length;
                    robotState.inGoalDistance = HOME_GOAL.distance;
                }
                robotState.inHeading = lastSensorUpdate.heading;
                robotState.inX = robotX;
                robotState.inY = robotY;
                robotState.inBatteryVoltage = lastSensorUpdate.voltage;
                robotState.inLineAngle = lastSensorUpdate.lineAngle;
                robotState.inLineSize = lastSensorUpdate.lineSize;
                robotState.inLastAngle = lastSensorUpdate.lastAngle;
                robotState.inOnLine = lastSensorUpdate.onLine;
                robotState.inLineOver = lastSensorUpdate.lineOver;

                // unlock semaphores
                xSemaphoreGive(robotStateSem);
                xSemaphoreGive(pbSem);
                xSemaphoreGive(goalDataSem);
        } else {
            ESP_LOGW(TAG, "Failed to acquire semaphores, cannot update FSM data.");
        }

        // update the actual FSM
        fsm_update(stateMachine);

        // line over runs after the FSM to override it
        update_line(&robotState);

        // print_ball_data(&robotState);
        // print_goal_data(&robotState);
        // vTaskDelay(pdMS_TO_TICKS(250));
        // print_motion_data(&robotState);
        
        // goal_correction(&robotState);

        // run motors
        motor_calc(robotState.outDirection, robotState.outOrientation, robotState.outSpeed);
        motor_move(robotState.outShouldBrake);

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10)); // Random delay at of loop to allow motors to spin
    }
}

static bool ledOn = false;
static void blinky_callback(TimerHandle_t callback){
    gpio_set_level(DEBUG_LED_1, ledOn);
    ledOn = !ledOn;
}

// Task which runs on the slave. Reads and calculates sensor data, then sends to master.
static void slave_task(void *pvParameter){
    static const char *TAG = "SlaveTask";
    static uint8_t pbBuf[PROTOBUF_SIZE] = {0};
    uint8_t robotId = 69;

    // Initialise software
    nvs_get_u8_graceful("RobotSettings", "RobotID", &robotId);
    defines_init(robotId);
    TimerHandle_t blinky = xTimerCreate("Blinky", pdMS_TO_TICKS(500), true, NULL, blinky_callback);
    xTimerStart(blinky, portMAX_DELAY);

    // Initialise comms
    comms_i2c_init_master(I2C_NUM_0);
    i2c_scanner();

    // Initialise hardware
    tsop_init();
    simu_init();
    simu_calibrate();
    gpio_set_direction(DEBUG_LED_1, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "=============== Slave hardware init OK ===============");
    esp_task_wdt_add(NULL);
    
    while (true) {
        // update TSOPs
        for (int i = 0; i < TSOP_TARGET_READS; i++){
            tsop_update(NULL);
        }
        tsop_calc();

        // update IMU
        simu_calc();

        // setup protobuf byte stream, variables will be disposed of after loop ends
        memset(pbBuf, 0, PROTOBUF_SIZE);
        SensorUpdate msg = SensorUpdate_init_zero;
        pb_ostream_t stream = pb_ostream_from_buffer(pbBuf, PROTOBUF_SIZE);

        // set the message's values
        if (xSemaphoreTake(nanoDataSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
            msg.lastAngle = nanoData.lastAngle;
            msg.lineAngle = nanoData.lineAngle;
            msg.lineOver = nanoData.isLineOver;
            msg.lineSize = nanoData.lineSize;
            msg.onLine = nanoData.isOnLine;
            msg.voltage = nanoData.batteryVoltage;
            xSemaphoreGive(nanoDataSem);
        } else {
            ESP_LOGW(TAG, "Failed to unlock nano data semaphore!");
        }
        msg.heading = heading;
        msg.tsopAngle = tsopAngle;
        msg.tsopStrength = tsopAvgStrength;

        // encode and send it
        if (pb_encode(&stream, SensorUpdate_fields, &msg)){
            comms_i2c_write_protobuf(pbBuf, stream.bytes_written, MSG_SENSORUPDATE_ID);
            vTaskDelay(pdMS_TO_TICKS(4)); // wait so that the slave realises we're not sending any more data
        } else {
            ESP_LOGE(TAG, "Failed to encode SensorUpdate message: %s", PB_GET_ERROR(&stream));
        }

        // activate/deactivate debug LED if we're on the line
        // gpio_set_level(DEBUG_LED_1, msg.onLine || msg.lineOver);
        esp_task_wdt_reset();

        // printf("angle: %f, strength: %f\n", tsopAngle, tsopStrength);
        // ESP_LOGD(TAG, "%f", heading);
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_test_task(void *pvParameter){
    static const char *TAG = "MotorTestTask";

    motor_init();
    ESP_ERROR_CHECK(gpio_set_direction(KICKER_PIN, GPIO_MODE_OUTPUT));
    ESP_LOGI(TAG, "Motor test init OK");

    while (true){
        ESP_LOGI(TAG, "Going forward");
        motor_calc(0, 0, 75.0f);
        motor_move(false);
        vTaskDelay(pdMS_TO_TICKS(2500));
        
        ESP_LOGI(TAG, "Going backwards");
        motor_calc(180, 0, 75.0f);
        motor_move(false);
        vTaskDelay(pdMS_TO_TICKS(2500));

        // ESP_LOGI(TAG, "Kicking");
        // ESP_ERROR_CHECK(gpio_set_level(KICKER_PIN, 1));
        // vTaskDelay(pdMS_TO_TICKS(KICKER_TIMEOUT));
        // ESP_ERROR_CHECK(gpio_set_level(KICKER_PIN, 0));
        // vTaskDelay(pdMS_TO_TICKS(SHOOT_TIMEOUT));
    }
}

void app_main(){
    puts("====================================================================================");
    puts(" * This ESP32 belongs to a robot from Team Deus Vult at Brisbane Boys' College.");
    puts(" * Software copyright (c) 2019 Team Deus Vult. All rights reserved.");
    puts("====================================================================================");

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

    // write master/slave/robot ID to NVS if configured
    #if defined NVS_WRITE_MASTER
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_MASTER));
        ESP_LOGE("AutoMode", "Successfully wrote Master to NVS.");
    #elif defined NVS_WRITE_SLAVE
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "Mode", AUTOMODE_SLAVE));
        ESP_LOGE("AutoMode", "Successfully wrote Slave to NVS.");
    #elif defined NVS_WRITE_ROBOTNUM
        ESP_ERROR_CHECK(nvs_set_u8(storageHandle, "RobotID", NVS_WRITE_ROBOTNUM));
        ESP_LOGE("RobotID", "Successfully wrote robot number to NVS.");
    #endif

    #if defined NVS_WRITE_MASTER || defined NVS_WRITE_SLAVE || defined NVS_WRITE_ROBOTNUM
        ESP_ERROR_CHECK(nvs_commit(storageHandle));
    #endif

    // we don't use nvs_get_u8_graceful, because its different here as we have the storage handle already open
    err = nvs_get_u8(storageHandle, "Mode", &mode);
    nvs_close(storageHandle);

    if (err == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE("AutoMode", "Mode key not found! Please identify this device, see main.c for help. Cannot continue.");
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
        xTaskCreatePinnedToCore(master_task, "MasterTask", 12048, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
        // xTaskCreate(motor_test_task, "MotorTestTask", 8192, NULL, configMAX_PRIORITIES, NULL);
    } else {
        ESP_LOGI("AppMain", "Running as slave");
        xTaskCreatePinnedToCore(slave_task, "SlaveTask", 12048, NULL, configMAX_PRIORITIES, NULL, APP_CPU_NUM);  
    }
}