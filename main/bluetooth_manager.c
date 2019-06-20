#include "comms_bluetooth.h"

// tasks which runs when a Bluetooth connection is established. manages sending and receiving data as well as logic.

static dv_timer_t packetTimer = {NULL, false};
static dv_timer_t cooldownTimer = {NULL, false};
static SemaphoreHandle_t cooldownSem = NULL;

static void packet_timer_callback(TimerHandle_t timer){
    static const char *TAG = "BTTimeout";

    ESP_LOGW(TAG, "Packet timeout has gone off, other robot is off for damage");
    uint32_t handle = (uint32_t) pvTimerGetTimerID(timer);

    if (robotState.outIsAttack) fsm_change_state(stateMachine, &stateDefenceDefend);
    vTaskSuspend(receiveTaskHandle);
    vTaskSuspend(sendTaskHandle);
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_spp_disconnect(handle));  
    dv_timer_stop(&packetTimer);
}

static void cooldown_timer_callback(TimerHandle_t timer){
    static const char *TAG = "CooldownTimer";
    ESP_LOGI(TAG, "Cooldown timer gone off, re-enabling switch");
    xSemaphoreGive(cooldownSem);
    dv_timer_stop(&cooldownTimer);
}

void comms_bt_receive_task(void *pvParameter){
    static const char *TAG = "BTReceive";
    uint32_t handle = (uint32_t) pvParameter;
    bool wasSwitchOk = false;
    uint8_t switchBuffer[] = {'S', 'W', 'I', 'T', 'C', 'H'};

    // create packet timeout timer if it's not been created in a previous run of this task
    if (packetTimer.timer == NULL){
        ESP_LOGI(TAG, "Creating packet timeout timer");
        packetTimer.timer = xTimerCreate("BTTimeout", pdMS_TO_TICKS(BT_PACKET_TIMEOUT), false, pvParameter, 
                        packet_timer_callback);
    }

    if (cooldownTimer.timer == NULL){
        ESP_LOGI(TAG, "Creating cooldown timer");
        cooldownTimer.timer = xTimerCreate("CooldownTimer", pdMS_TO_TICKS(BT_SWITCH_COOLDOWN), false, pvParameter,
                                cooldown_timer_callback);
    }
    
    if (cooldownSem == NULL){
        ESP_LOGI(TAG, "Creating switch semaphore");
        cooldownSem = xSemaphoreCreateBinary();
    }

    ESP_LOGI(TAG, "BT receive task init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);

    while (true){
        BTProvide recvMsg = BTProvide_init_zero;

        if (xQueueReceive(packetQueue, &recvMsg, portMAX_DELAY)){
            ESP_LOGD(TAG, "Received BT packet: state: %s, robotX: %f, robotY: %f, switch ok: %s", recvMsg.fsmState, 
            recvMsg.robotX, recvMsg.robotY, recvMsg.switchOk ? "yes" : "no");
            
            // reset the timeout timer if we got a packet
            xTimerStart(packetTimer.timer, portMAX_DELAY);
            xTimerReset(packetTimer.timer, portMAX_DELAY);
        }

        // decide if we should switch or not
        if (recvMsg.switchOk){
            ESP_LOGI(TAG, "Other robot is willing to switch");
            wasSwitchOk = true;

            if (robotState.outSwitchOk && xSemaphoreTake(cooldownSem, 0)){
                ESP_LOGI(TAG, "I'm also willing to switch: switching NOW!");
                // write switch, change states and start cooldown timer to re-anble switching
                esp_spp_write(handle, 6, switchBuffer);
                fsm_change_state(stateMachine, &stateDefenceDefend); 
                dv_timer_start(&cooldownTimer);
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

    ESP_LOGI(TAG, "BT send task init OK, handle: %d", handle);
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
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}