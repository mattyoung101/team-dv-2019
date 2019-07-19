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
#include "driver/spi_master.h"

#if ENEMY_GOAL == GOAL_YELLOW
    #define AWAY_GOAL goalYellow
    #define HOME_GOAL goalBlue
#elif ENEMY_GOAL == GOAL_BLUE
    #define AWAY_GOAL goalBlue
    #define HOME_GOAL goalYellow
#endif

static uint8_t mode = 69; // start out with invalid mode
state_machine_t *stateMachine = NULL;
static const char *RST_TAG = "ResetReason";

// supposed to break the robot on error, doesn't work tho
static void shutdown_handler(){
    ets_printf("Shutdown handler executing\n");
    motor_calc(0, 0, 0.0f);
    motor_move(true);
}

static void print_reset_reason(){
    esp_reset_reason_t resetReason = esp_reset_reason();
    if (resetReason == ESP_RST_PANIC){
        ESP_LOGW(RST_TAG, "Reset due to panic!");
    } else if (resetReason == ESP_RST_INT_WDT || resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT){
        ESP_LOGW(RST_TAG, "Reset due to watchdog timer!");
    } else {
        ESP_LOGD(RST_TAG, "Other reset reason: %d", resetReason);
    }
}

// Task which runs on the master. Receives sensor data from slave and handles complex routines
// like moving, finite state machines, Bluetooth, etc
static void master_task(void *pvParameter){
    static const char *TAG = "MasterTask";
    uint8_t robotId = 69;

    print_reset_reason();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(shutdown_handler));

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
    #if DEFENCE
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
                robotState.inBallAngle = orangeBall.angle;
                robotState.inBallStrength = orangeBall.length;
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
                // robotState.inX = robotX;
                // robotState.inY = robotY;
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

        // robotState.outSpeed = 0;
        // imu_correction(&robotState);

        // line over runs after the FSM to override it
        update_line(&robotState);

        // print_ball_data(&robotState);
        // print_goal_data(&robotState);
        // vTaskDelay(pdMS_TO_TICKS(250));
        // print_motion_data(&robotState);
        // print_position_data(&robotState);

        // printf("%f\n", robotState.inLineAngle);

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

static void imu_task(void *pvParameter){
    int16_t i = 0;
    
    float begin = (float) esp_timer_get_time();
    while (true){
        simu_calc();

        if (i++ > 1024){
            printfln("Time: %f us", ((float) esp_timer_get_time() - begin) / 1024.0f);
            begin = esp_timer_get_time();
            i = 0;
        }
    }
}

// Task which runs on the slave. Reads and calculates sensor data, then sends to master.
static void slave_task(void *pvParameter){
    static const char *TAG = "SlaveTask";
    static uint8_t pbBuf[PROTOBUF_SIZE] = {0};
    uint8_t robotId = 69;
    uint16_t currentTimerPeriod = 500;

    print_reset_reason();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(shutdown_handler));

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
    xTaskCreatePinnedToCore(imu_task, "IMUTask", 4096, NULL, configMAX_PRIORITIES, NULL, PRO_CPU_NUM);

    ESP_LOGI(TAG, "=============== Slave hardware init OK ===============");
    // puts("Time,Heading");
    esp_task_wdt_add(NULL);
    
    while (true) {
        // // update TSOPs
        // for (int i = 0; i < TSOP_TARGET_READS; i++){
        //     tsop_update(NULL);
        // }
        // tsop_calc();

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
            if (comms_i2c_write_protobuf(pbBuf, stream.bytes_written, MSG_SENSORUPDATE_ID) != ESP_OK){
                // try to detect that weird crash bug
                ESP_LOGE(TAG, "I2C error detected!");
                if (currentTimerPeriod != 100){
                    xTimerChangePeriod(blinky, pdMS_TO_TICKS(100), portMAX_DELAY);
                    currentTimerPeriod = 100;
                }
            } else {
                if (currentTimerPeriod != 500){
                    xTimerChangePeriod(blinky, pdMS_TO_TICKS(500), portMAX_DELAY);
                    currentTimerPeriod = 500;
                }
            }
            
            ets_delay_us(4000); // wait so that the slave realises we're not sending any more data
        } else {
            ESP_LOGE(TAG, "Failed to encode SensorUpdate message: %s", PB_GET_ERROR(&stream));
        }

        // activate/deactivate debug LED if we're on the line
        gpio_set_level(DEBUG_LED_1, msg.onLine || msg.lineOver);
        esp_task_wdt_reset();

        // printf("angle: %f, strength: %f\n", tsopAngle, tsopStrength);
        // ESP_LOGD(TAG, "%f", heading);
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_test_task(void *pvParameter){
    static const char *TAG = "TestTask";

    spi_bus_config_t conf = {
        .mosi_io_num = -1,
        .miso_io_num = -1,
        .sclk_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &conf, 1));

    spi_device_handle_t *teensyHandle = NULL;
    spi_device_interface_config_t teensyConf = {0};
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &teensyConf, teensyHandle));

    while (true){
        uint8_t data[] = {42, 33, 100, 22, 100};
        spi_transaction_t trans = {0};
        trans.length = 5 * 8;
        trans.tx_buffer = data;
        ESP_ERROR_CHECK(spi_device_transmit(teensyHandle, &trans));
        ESP_LOGI(TAG, "Transmission completed");

        vTaskDelay(pdMS_TO_TICKS(100));
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