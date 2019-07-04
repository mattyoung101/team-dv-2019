#include "comms_bluetooth.h"

// Based on the ESP32 BT SPP initiator example (https://git.io/fj465) and the BT SPP acceptor example (https://git.io/fj46d)

static const char *TAGM = "CommsBT_M";
static const char *TAGS = "CommsBT_S";
static const char *TAG = "CommsBT";

TaskHandle_t receiveTaskHandle = NULL;
TaskHandle_t sendTaskHandle = NULL;
QueueHandle_t packetQueue = NULL;
static uint8_t switch_buffer[] = {'S', 'W', 'I', 'T', 'C', 'H'};
static bool isMaster = false;
static uint8_t totalErrors = 0;
static bool firstConnection = true;
static uint8_t btErrors = 0;

/** Initialises Bluetooth stack **/
static void bt_init(void){
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_LOGI(TAG, "Bluetooth stack init OK");
}

/** restart GAP discovery **/
static void bt_gap_restart_disc(void){
    if (btErrors > 4){
        // unclear how well this will work considering esp_restart() doesn't actually reset Bluetooth
        ESP_LOGE(TAG, "Too many Bluetooth errors, resetting to fix!");
        abort();
    }

    esp_bt_gap_cancel_discovery();
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
    ESP_LOGI(TAG, "Restarting GAP discovery...");
}

/** decode data and push to packet queue */
static void bt_pb_decode_and_push(uint16_t size, uint8_t *data, uint32_t handle){
    // check if the buffer is exactly equivalent to the string "SWITCH" in which case switch
    if (memcmp(data, switch_buffer, size) == 0){
        ESP_LOGI(TAG, "========== Switch request received: switching NOW! ==========");
        // invert state
        if (robotState.outIsAttack){
            fsm_change_state(stateMachine, &stateDefenceDefend);
        } else {
            fsm_change_state(stateMachine, &stateAttackPursue);
        }
        return;
    }

    // otherwise, decode the message as a normal protobuf buffer
    BTProvide msg = BTProvide_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(data, size);

    if (pb_decode(&stream, BTProvide_fields, &msg)){
        xQueueOverwrite(packetQueue, &msg);
        totalErrors = 0;
    } else {
        ESP_LOGE(TAG, "Protobuf decode error: %s", PB_GET_ERROR(&stream));
        if (totalErrors++ > 4){
            ESP_LOGE(TAG, "Too many errors in a row, dropping connection");
            esp_spp_disconnect(handle);
        }
    }
}

/** starts the logic task */
static void bt_start_tasks(esp_spp_cb_param_t *param){
    // change into our default mode if it's the first connection
    if (firstConnection){
        ESP_LOGI(TAG, "First connection, changing into default mode");
        fsm_change_state(stateMachine, ROBOT_MODE == MODE_ATTACK ? &stateAttackPursue : &stateDefenceDefend);
        firstConnection = false;
    }
    RS_SEM_LOCK
    robotState.inBTConnection = true;
    RS_SEM_UNLOCK

    // delay so that Bluetooth, which runs on core 0, will pick up on the fact that we changed states
    vTaskDelay(pdMS_TO_TICKS(500));

    xTaskCreate(comms_bt_receive_task, "BTReceiveTask", 4096, (void*) param->open.handle, 
            configMAX_PRIORITIES - 4, &receiveTaskHandle);
    
    xTaskCreate(comms_bt_send_task, "BTSendTask", 4096, (void*) param->open.handle, 
            configMAX_PRIORITIES - 5, &sendTaskHandle);

    // if we're here we can assume that the connection was successful
    btErrors = 0;
}

void comms_bt_stop_tasks(void){
    if (receiveTaskHandle != NULL){
        esp_task_wdt_delete(receiveTaskHandle);
        vTaskDelete(receiveTaskHandle);
        receiveTaskHandle = NULL;
    }

    if (sendTaskHandle != NULL){
        esp_task_wdt_delete(sendTaskHandle);
        vTaskDelete(sendTaskHandle);
        sendTaskHandle = NULL;
    }

    RS_SEM_LOCK
    robotState.inBTConnection = false;
    RS_SEM_UNLOCK
}

//////////////////////////// MASTER (ACCEPTOR) //////////////////////////// 

static void esp_bt_gap_cb_master(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            ESP_LOGD(TAGM, "ESP_BT_GAP_DISC_RES_EVT, remote addr:");
            ESP_LOG_BUFFER_HEX_LEVEL(TAGM, param->disc_res.bda, ESP_BD_ADDR_LEN, ESP_LOG_DEBUG);
            break;
        }
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAGM, "Authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(TAGM, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(TAGM, "Authentication failed, status code: %d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGD(TAGM, "ESP_BT_GAP_CFM_REQ_EVT, value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGD(TAGM, "Discovery status changed, now: %d", param->disc_st_chg.state);
            break;
        default:
            break;
    }
    return;
}

static void esp_spp_cb_master(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_bt_dev_set_device_name(ROBOT0_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_NAME);
            ESP_LOGI(TAGM, "SPP server initialised");
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGD(TAGM, "SPP discovery completed");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAGM, "SPP connection opened");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGW(TAGM, "Slave has disconnected (SPP connection closed), deleting tasks");
            ESP_LOGI(TAGM, "Status: %d, Port status: %d, Async: %d", param->close.status, param->close.port_status,
                    param->close.async);
            comms_bt_stop_tasks();
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAGM, "SPP server started");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAGM, "SPP client connection initiated");
            break;
        case ESP_SPP_DATA_IND_EVT:
            bt_pb_decode_and_push(param->data_ind.len, param->data_ind.data, param->data_ind.handle);
            break;
        case ESP_SPP_CONG_EVT:
            if (param->cong.cong && sendTaskHandle){
                ESP_LOGW(TAGM, "Suspending send task due to congestion");
                vTaskSuspend(sendTaskHandle);
            } else if (sendTaskHandle) {
                ESP_LOGW(TAGM, "Resuming send task as congestion has cleared");
                vTaskResume(sendTaskHandle);
            }
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAGM, "SPP server opened, starting tasks");
            bt_start_tasks(param);
            break;
        default:
            break;
    }
}

//////////////////////////// SLAVE (INITIATOR) //////////////////////////// 

static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static const char remote_device_name[] = ROBOT0_NAME;
static esp_bd_addr_t peer_bd_addr;

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len){
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_bt_gap_cb_slave(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch(event){
        case ESP_BT_GAP_DISC_RES_EVT:
            // ESP_LOGI(TAGS, "ESP_BT_GAP_DISC_RES_EVT");
            // esp_log_buffer_hex(TAGS, param->disc_res.bda, ESP_BD_ADDR_LEN);

            for (int i = 0; i < param->disc_res.num_prop; i++){
                if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                    && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                        // ESP_LOGD(TAGS, "Discovered device: %s", peer_bdname);
                    
                        if (strlen(remote_device_name) == peer_bdname_len
                            && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {
                                ESP_LOGI(TAGS, "Found other robot (name: %s). Attempting to establish SPP connection...",
                                        peer_bdname);
                                memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                                esp_spp_start_discovery(peer_bd_addr);
                                esp_bt_gap_cancel_discovery();
                        }
                }
            }
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAGS, "Authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(TAGS, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(TAGS, "Authentication failed, status: %d", param->auth_cmpl.stat);
                btErrors++;
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAGS, "ESP_BT_GAP_CFM_REQ_EVT, value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        default:
            break;
    }
}

static void esp_spp_cb_slave(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_bt_dev_set_device_name(ROBOT1_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
            ESP_LOGI(TAGS, "SPP client initialised");
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(TAGS, "SPP discovery completed, status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
            
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAGS, "Connecting to SPP server...");
                esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, param->disc_comp.scn[0], peer_bd_addr);
            } else {
                ESP_LOGE(TAGS, "Can't connect to SPP due to error code: %d", param->disc_comp.status);
                btErrors++;
                bt_gap_restart_disc();
            }
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAGS, "SPP connection opened, creating tasks");
            bt_start_tasks(param);
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGW(TAGS, "Master has disconnected (SPP connection closed), deleting tasks");
            comms_bt_stop_tasks();
            bt_gap_restart_disc();
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAGS, "SPP client started");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAGS, "SPP client connection initiated");
            break;
        case ESP_SPP_DATA_IND_EVT:
            bt_pb_decode_and_push(param->data_ind.len, param->data_ind.data, param->data_ind.handle);
            break;
        case ESP_SPP_CONG_EVT:
            if (param->cong.cong && sendTaskHandle){
                ESP_LOGW(TAGS, "Suspending send task due to congestion");
                vTaskSuspend(sendTaskHandle);
            } else if (sendTaskHandle) {
                ESP_LOGW(TAGS, "Resuming send task as congestion has cleared");
                vTaskResume(sendTaskHandle);
            }
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAGS, "SPP server opened");
            break;
        default:
            break;
    }
}

static void comms_bt_init_generic(esp_bt_gap_cb_t gap_cb, esp_spp_cb_t spp_cb){
    bt_init();
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
    
    // Simple Secure Paring (SSP) params
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // legacy pairing params
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    // create the BT packet queue
    if (packetQueue == NULL){
        packetQueue = xQueueCreate(PACKET_QUEUE_LENGTH, BTProvide_size);
    }
}

// waits for connection, acceptor
void comms_bt_init_master(){
    isMaster = true;
    comms_bt_init_generic(esp_bt_gap_cb_master, esp_spp_cb_master);
    ESP_LOGI(TAGM, "Bluetooth master init OK");
}

// connects to master, initiator
void comms_bt_init_slave(){
    isMaster = false;
    comms_bt_init_generic(esp_bt_gap_cb_slave, esp_spp_cb_slave);
    ESP_LOGI(TAGS, "Bluetooth slave init OK");
}