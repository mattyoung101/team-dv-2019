#include "comms_bluetooth.h"

// contains the logic which handles Bluetooth
// TODO can these actually both be the same thing? we don't really have the concept of the master or slave

void comms_bt_slave_controller_task(void *pvParameter){
    static const char *TAG = "BTManager_S";
    uint32_t handle = (uint32_t) pvParameter;
    uint8_t buf[PROTOBUF_SIZE] = {0};

    ESP_LOGI(TAG, "BT slave manager init OK, handle: %d", handle);

    while (true){
        memset(buf, 0, PROTOBUF_SIZE);

        BTProvide msg = BTProvide_init_zero;
        msg.onLine = false;
        strncpy(msg.fsmState, "StateYeet", strlen("StateYeet"));
        msg.robotX = 69.420f;
        msg.robotY = 1337.421212f;

        pb_ostream_t stream = pb_ostream_from_buffer(buf, PROTOBUF_SIZE);
        if (pb_encode(&stream, BTProvide_fields, &msg)){
            ESP_LOGD(TAG, "Sending %d bytes", stream.bytes_written);

            // should probably check congestion status before write, but we don't have access to that here
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void comms_bt_master_controller_task(void *pvParameter){
    static const char *TAG = "BTManager_M";
    uint32_t handle = (uint32_t) pvParameter;
    BTProvide msg = BTProvide_init_zero;

    ESP_LOGI(TAG, "BT master manager init OK, handle: %d", handle);

    while (true){
        if (xQueueReceive(packetQueue, &msg, portMAX_DELAY)){
            ESP_LOGD(TAG, "Received packet");
            ESP_LOGD(TAG, "Stuff in packet, state: %s, robotX: %f, robotY: %f", msg.fsmState, msg.robotX, msg.robotY);
        }
    }
}