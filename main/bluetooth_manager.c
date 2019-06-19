#include "comms_bluetooth.h"

// task which runs when a Bluetooth connection is established.
// manages sending and receiving data as well as logic

void comms_bt_receive_task(void *pvParameter){
    static const char *TAG = "BTReceive";
    uint32_t handle = (uint32_t) pvParameter;
    bool wasSwitchOk = false;
    uint8_t switchBuffer[] = {'S', 'W', 'I', 'T', 'C', 'H'};

    ESP_LOGI(TAG, "BT manager init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);

    while (true){
        BTProvide recvMsg = BTProvide_init_zero;

        if (xQueueReceive(packetQueue, &recvMsg, pdMS_TO_TICKS(250))){
            ESP_LOGD(TAG, "Received BT packet: state: %s, robotX: %f, robotY: %f, switch ok: %s", recvMsg.fsmState, recvMsg.robotX, 
            recvMsg.robotY, recvMsg.switchOk ? "yes" : "no");
        }

        // decide if we should switch or not
        if (recvMsg.switchOk){
            ESP_LOGI(TAG, "Other robot is willing to switch");
            wasSwitchOk = true;

            if (robotState.outSwitchOk){
                ESP_LOGI(TAG, "I'm also willing to switch: switching NOW!");
                esp_spp_write(handle, 6, switchBuffer);
                fsm_change_state(stateMachine, &stateDefenceIdle);
                esp_task_wdt_reset();
                continue;
            }
        } else if (wasSwitchOk){
            // if the other robot is not willing to switch, but was previously willing to switch
            ESP_LOGI(TAG, "Other robot is NO LONGER willing to switch.");
            wasSwitchOk = false;
        }

        esp_task_wdt_reset();
    }
}

void comms_bt_send_task(void *pvParameter){
    static const char *TAG = "BTSend";
    uint32_t handle = (uint32_t) pvParameter;
    uint8_t buf[PROTOBUF_SIZE] = {0};

    ESP_LOGI(TAG, "BT manager init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);
    
    while (true){
        memset(buf, 0, PROTOBUF_SIZE);

        BTProvide sendMsg = BTProvide_init_zero;
        sendMsg.onLine = robotState.inOnLine;
        strcpy(sendMsg.fsmState, stateMachine->currentState->name);
        sendMsg.robotX = robotState.inX;
        sendMsg.robotY = robotState.inY;
        sendMsg.switchOk = robotState.outSwitchOk;

        pb_ostream_t stream = pb_ostream_from_buffer(buf, PROTOBUF_SIZE);
        if (pb_encode(&stream, BTProvide_fields, &sendMsg)){
            ESP_LOGD(TAG, "Sending %d bytes", stream.bytes_written);
            // should probably check congestion status before write, but we don't have access to that here
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100)); // TODO decrease this but test first
    }
}