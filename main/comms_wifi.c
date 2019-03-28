#include "comms_wifi.h"

static EventGroupHandle_t wifi_event_group;

static int send_packet(int socket, int32_t *txBuf, const char *TAG){
    int err = send(socket, txBuf, sizeof(txBuf), 0);
    if (err < 0){
        ESP_LOGE(TAG, "Unable to send data: error %s", strerror(errno));
        return -1;
    }
    return 0;
}

static int receive_packet(int socket, int32_t *rxBuf, const char *TAG){
    int len = recv(socket, rxBuf, sizeof(rxBuf), 0);
    if (len < 0){
        ESP_LOGE(TAG, "Unable to receive data: error %s", strerror(errno));
        return -1;
    }

    ESP_LOGI(TAG, "Received %d bytes successfully", len);
    return len;
}

static void socket_server(void *pvParameter){
    static const char *TAG = "SocketServer";
    int32_t txBuf[WIFI_BUF_SIZE];
    int32_t rxBuf[WIFI_BUF_SIZE];
    char addr_str[128];

    while (true){
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(SOCK_PORT);
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listenSock < 0){
            ESP_LOGE(TAG, "Unable to create socket: error %s", strerror(errno));
            break;
        }
        ESP_LOGI(TAG, "Socket created successfully");

        int err = bind(listenSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0){
            ESP_LOGE(TAG, "Unable to bind socket: error %s", strerror(errno));
            break;
        }
        ESP_LOGI(TAG, "Socket bound successfully");
        
        err = listen(listenSock, 1);
        if (err != 0){
            ESP_LOGE(TAG, "Unable to listen: error %s", strerror(errno));
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in sourceAddr;
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listenSock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0){
            ESP_LOGE(TAG, "Unable to accept connection: error %s", strerror(errno));
            break;
        }
        ESP_LOGI(TAG, "Accepted client");

        while (true){
            int len = recv(sock, rxBuf, sizeof(rxBuf), 0);
            if (len < 0){
                ESP_LOGE(TAG, "Unable to receive transmission, error %s", strerror(errno));
                break;
            } else if (len == 0){
                ESP_LOGW(TAG, "Connection closed (client off for damage)");
                // switch to defence then restart the server
                break;
            } else {
                ESP_LOGI(TAG, "Data received successfully, data %d %d", rxBuf[0], rxBuf[1]);
                
                // just respond with notify ok for now
                memset(txBuf, 0, sizeof(txBuf));
                txBuf[0] = WTYPE_NOTIFY;
                txBuf[1] = WMSG_OK;
                if (send_packet(sock, txBuf, TAG) == -1){
                    break;
                }
            }
        }

        if (sock != -1) {
            // restart the server and wait for the client to rejoin
            // might not actually have to restart but just in case its a failure on our end (dodgy server) may as well:
            // we wouldn't want a corrupt state
            ESP_LOGW(TAG, "Socket failure, restarting server...");
            shutdown(sock, 0);
            close(sock);
            // don't we need to close the listen socket here as well? docs don't do it...
        }
    }

    ESP_LOGE(TAG, "Error occurred, destroying task");
    vTaskDelete(NULL);
}

static void socket_client(void *pvParameter){
    static const char *TAG = "SocketClient";
    int32_t txBuf[WIFI_BUF_SIZE];
    int32_t rxBuf[WIFI_BUF_SIZE];
    char addr_str[128];

    // attempt to establish connection and block until we can
    // once connected, run comms with our host
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");

    while (true){
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(SOCK_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(SOCK_PORT);
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0){
            ESP_LOGE(TAG, "Unable to create socket: error %s", strerror(errno));
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: error %s", strerror(errno));
            // we could keep retrying this connection, but at this point, the host's network should be up
            // so we should be able to connect, this means something else has happened
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (true){
            // decide which packet to send
            ESP_LOGI(TAG, "Sending QUERY PING");
            memset(txBuf, 0, sizeof(txBuf));
            txBuf[0] = WTYPE_QUERY;
            txBuf[1] = WMSG_PING;
            if (send_packet(sock, txBuf, TAG) == -1){
                break;
            }

            // await response
            memset(rxBuf, 0, sizeof(rxBuf));
            if (receive_packet(sock, rxBuf, TAG) == -1){
                break;
            }

            ESP_LOGI(TAG, "Exchange complete, first two bytes: %d %d", rxBuf[0], rxBuf[1]);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // socket failed
        if (sock != -1) {
            ESP_LOGW(TAG, "Socket failure, restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

    ESP_LOGE(TAG, "Error occurred, destroying task");
    vTaskDelete(NULL);
}

static esp_err_t event_handler_host(void *ctx, system_event_t *event){
    static const char *TAG = "WifiEventHandler_H";

    switch(event->event_id) {
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Station: "MACSTR" connected, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "Station: "MACSTR" disconnected, AID=%d",
                    MAC2STR(event->event_info.sta_disconnected.mac),
                    event->event_info.sta_disconnected.aid);
            // client must be off for damage - socket code should detect this
            break;
        default:
            break;
    }
    return ESP_OK;
}

static esp_err_t event_handler_client(void *ctx, system_event_t *event){
    static const char *TAG = "WifiEventHandler_C";

    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "Station started, attemtping connection...");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "Connected with IP: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // connection dropped, host is off for damage - socket will handle this
            ESP_LOGI(TAG, "Disconnected from AP, retrying...");
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void comms_wifi_init_host(){
    wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler_host, NULL));
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = WIFI_MAXCON,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("CommsWiFi_H", "WiFi AP init OK");

    // xTaskCreate(socket_server, "SocketServerTask", 8192, NULL, configMAX_PRIORITIES - 2, NULL);
}

void comms_wifi_init_client(){
    wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler_client, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("CommsWifi_C", "WiFi station init OK");

    xTaskCreate(socket_client, "SocketClientTask", 8192, NULL, configMAX_PRIORITIES - 1, NULL);
}