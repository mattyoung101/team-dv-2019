#include "comms_bluetooth.h"

// Based on the ESP32 BT SPP initiator example (https://git.io/fj465) and the BT SPP acceptor example (https://git.io/fj46d)

static const char *TAGM = "CommsBT_M";
static const char *TAGS = "CommsBT_S";
static const char *TAG = "CommsBT";

static uint8_t materData[] = {0xA, 0xB, 0xC, 0xD, 0xE, 0xF};
static uint8_t slaveData[] = {0xF, 0xE, 0xD, 0xC, 0xB, 0xA};

/** Initialises Bluetooth stack **/
static void bt_init(){
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = ESP_OK;
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Bluetooth stack init OK");
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
        case ESP_BT_GAP_PIN_REQ_EVT:{
            // ESP_LOGD(TAGM, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit: %d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                ESP_LOGD(TAGM, "Sending 16 digit pin code");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGD(TAGM, "Sending 4 digit pin code");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGD(TAGM, "ESP_BT_GAP_CFM_REQ_EVT, value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGD(TAGM, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %d", param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGD(TAGM, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
        default: {
            // ESP_LOGI(TAGM, "bt_gap_cb event: %d", event);
            break;
        }
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
            ESP_LOGD(TAGM, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAGM, "SPP connection opened");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAGM, "SPP connection closed");
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAGM, "SPP server started");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAGM, "SPP client connection initiated");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAGM, "SPP data received len=%d handle=%d",
                    param->data_ind.len, param->data_ind.handle);
            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(TAGM, "SPP congestion status changed");
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(TAGM, "SPP write operation completed");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAGM, "SPP server opened");
            break;
        default:
            // ESP_LOGI(TAGM, "spp_cb event: %d", event);
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
                                ESP_LOGI(TAGS, "Found other robot! Attempting to establish SPP connection...");
                                memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                                esp_spp_start_discovery(peer_bd_addr);
                                esp_bt_gap_cancel_discovery();
                        }
                }
            }
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            // ESP_LOGI(TAGS, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
            break;
        case ESP_BT_GAP_RMT_SRVCS_EVT:
            // ESP_LOGI(TAGS, "ESP_BT_GAP_RMT_SRVCS_EVT");
            break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            // ESP_LOGI(TAGS, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAGS, "Authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(TAGS, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(TAGS, "Authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT:{
            // ESP_LOGI(TAGS, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                ESP_LOGI(TAGS, "Sending 16 digit pin code");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGI(TAGS, "Sending 4 digit pin code");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAGS, "ESP_BT_GAP_CFM_REQ_EVT, value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(TAGS, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %d", param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(TAGS, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
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
                ESP_LOGW(TAGS, "Can't connect to SPP due to error code: %d", param->disc_comp.status);
                // TODO restart entire bluetooth stack here, or at least try to disconnect then reconnect
            }
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAGS, "SPP connection opened");
            esp_spp_write(param->srv_open.handle, 6, slaveData);
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAGS, "SPP connection closed");
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAGS, "SPP client started");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAGS, "SPP client connection initiated");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAGS, "SPP data received");
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(TAGS, "SPP client congestion status changed");
            break;
        case ESP_SPP_WRITE_EVT:
            if (param->write.cong == 0) {
                esp_spp_write(param->write.handle, 6, slaveData);
            }
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAGS, "SPP server opened");
            break;
        default:
            break;
    }
}

// waits for connection, acceptor
void comms_bt_init_master(){
    bt_init();

    esp_err_t ret = ESP_OK;
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb_master)) != ESP_OK) {
        ESP_LOGE(TAGM, "GAP register failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb_master)) != ESP_OK) {
        ESP_LOGE(TAGM, "SPP register failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(TAGM, "SPP init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAGM, "Bluetooth master init OK");
}

// connects to master, initiator
void comms_bt_init_slave(){
    bt_init();

    esp_err_t ret = ESP_OK;
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb_slave)) != ESP_OK) {
        ESP_LOGE(TAGS, "GAP register failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb_slave)) != ESP_OK) {
        ESP_LOGE(TAGS, "SPP register failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(TAGS, "SPP init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAGS, "Bluetooth slave init OK");
}