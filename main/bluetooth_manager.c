#include "comms_bluetooth.h"

// contains the logic which handles Bluetooth

void comms_bt_slave_controller_task(void *pvParameter){
    static const char *TAG = "BTController_S";
    uint32_t handle = (uint32_t) pvParameter;

    ESP_LOGI(TAG, "BT slave controller init OK, handle: %d", handle);

    while (true){
        TASK_HALT;
    }
}

void comms_bt_master_controller_task(void *pvParameter){
    static const char *TAG = "BTController_M";
    uint32_t handle = (uint32_t) pvParameter;

    ESP_LOGI(TAG, "BT master controller init OK, handle: %d", handle);

    while (true){
        TASK_HALT;
    }
}