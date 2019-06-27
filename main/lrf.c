#include "lrf.h"

uint16_t lrfDistance = 0;

static void lrf_task(void *pvParameter){
    static const char *TAG = "LRFTask";
    uint8_t buf[8] = {0};

    ESP_LOGI(TAG, "LRF task init OK");
    esp_task_wdt_add(NULL);

    while (true){
        memset(buf, 0, 8);
        uart_read_bytes(UART_NUM_1, buf, 8, portMAX_DELAY);

        if (buf[0] == 0x5A && buf[1] == 0x5A){
            lrfDistance = UNPACK_16(buf[4], buf[5]);
            // TODO check checksum here
        } else {
            ESP_LOGW(TAG, "Invalid buffer: [0x%X, 0x%X], expected [0x5A, 0x5A]", buf[0], buf[1]);
        }

        esp_task_wdt_reset();
    }
}

void lrf_driver_install(){
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    // TODO set pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 0, 0, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 256, 8, NULL, 0));

    // TODO write 0xA5+0xAF+0x54 to make it 115200

    xTaskCreate(lrf_task, "LRFTask", 2048, NULL, configMAX_PRIORITIES - 3, NULL);
}