#include "comms_bluetooth.h"

// task which runs when a Bluetooth connection is established.
// manages sending and receiving data as well as logic

void comms_bt_logic_task(void *pvParameter){
    static const char *TAG = "BTManager";
    uint32_t handle = (uint32_t) pvParameter;
    uint8_t buf[PROTOBUF_SIZE] = {0};

    ESP_LOGI(TAG, "BT manager init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);

    while (true){
        //////////////////////// SEND PACKET //////////////////////
        memset(buf, 0, PROTOBUF_SIZE);

        BTProvide msg = BTProvide_init_zero;
        msg.onLine = false;
        strncpy(msg.fsmState, stateMachine->currentState->name, strlen(stateMachine->currentState->name));
        msg.robotX = 69.420f;
        msg.robotY = 1337.421212f;
        msg.switchOk = true;

        pb_ostream_t stream = pb_ostream_from_buffer(buf, PROTOBUF_SIZE);
        if (pb_encode(&stream, BTProvide_fields, &msg)){
            ESP_LOGD(TAG, "Sending %d bytes", stream.bytes_written);
            // should probably check congestion status before write, but we don't have access to that here
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        //////////////////////// RECEIVE PACKET //////////////////////
        if (xQueueReceive(packetQueue, &msg, pdMS_TO_TICKS(15))){
            ESP_LOGD(TAG, "Received packet");
            ESP_LOGD(TAG, "Stuff in packet, state: %s, robotX: %f, robotY: %f", msg.fsmState, msg.robotX, msg.robotY);
        }

        esp_task_wdt_reset();
    }
}