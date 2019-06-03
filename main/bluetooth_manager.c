#include "comms_bluetooth.h"

// contains the logic which handles Bluetooth

void comms_bt_slave_controller_task(void *pvParameter){
    static const char *TAG = "BTCtrl_S";

    while (true){
        TASK_HALT;
    }
}

void comms_bt_master_controller_task(void *pvParameter){
    static const char *TAG = "BTCtrl_M";

    while (true){
        TASK_HALT;
    }
}