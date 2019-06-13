#include "comms_bluetooth.h"

// task which runs when a Bluetooth connection is established.
// manages sending and receiving data as well as logic

void comms_bt_logic_task(void *pvParameter){
    static const char *TAG = "BTManager";
    uint32_t handle = (uint32_t) pvParameter;
    uint8_t buf[PROTOBUF_SIZE] = {0};
    bool wasSwitchOk = false;
    uint8_t switch_buffer[] = {'S', 'W', 'I', 'T', 'C', 'H'};

    ESP_LOGI(TAG, "BT manager init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);

    while (true){
        BTProvide recvMsg = BTProvide_init_zero;

        //////////////////////// RECEIVE PACKET //////////////////////
        if (xQueueReceive(packetQueue, &recvMsg, pdMS_TO_TICKS(15))){
            ESP_LOGD(TAG, "Received packet");
            ESP_LOGD(TAG, "Stuff in packet, state: %s, robotX: %f, robotY: %f", recvMsg.fsmState, recvMsg.robotX, 
            recvMsg.robotY);
        }

        // decide if we should switch or not
        if (recvMsg.switchOk){
            ESP_LOGI(TAG, "Other robot is willing to switch.");
            wasSwitchOk = true;

            if (robotState.outSwitchOk){
                ESP_LOGI(TAG, "Switching NOW!");
                esp_spp_write(handle, 6, switch_buffer);
                // TODO do actual switch FSM bullshit whatever the fuck here
                
                esp_task_wdt_reset();
                continue;
            }
        } else if (wasSwitchOk){
            ESP_LOGI(TAG, "Other robot is NO LONGER willing to switch.");
            wasSwitchOk = false;
        }

        //////////////////////// SEND PACKET //////////////////////
        memset(buf, 0, PROTOBUF_SIZE);

        BTProvide msg = BTProvide_init_zero;
        msg.onLine = robotState.inOnLine;
        strcpy(msg.fsmState, stateMachine->currentState->name);
        msg.robotX = robotState.inX;
        msg.robotY = robotState.inY;
        msg.switchOk = robotState.outSwitchOk;

        pb_ostream_t stream = pb_ostream_from_buffer(buf, PROTOBUF_SIZE);
        if (pb_encode(&stream, BTProvide_fields, &msg)){
            ESP_LOGD(TAG, "Sending %d bytes", stream.bytes_written);
            // should probably check congestion status before write, but we don't have access to that here
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        esp_task_wdt_reset();
    }
}