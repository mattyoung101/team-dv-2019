#include "comms_i2c.h"

SemaphoreHandle_t rdSem = NULL;
i2c_data_t receivedData = {0};

static void comms_i2c_receive_task(void *pvParameters){
    static const char *TAG = "I2CReceiveTask";
    uint8_t *buf = malloc(9);
    rdSem = xSemaphoreCreateBinary();
    xSemaphoreGive(rdSem);

    ESP_LOGI(TAG, "Slave I2C task init OK");
    esp_task_wdt_add(NULL);

    while (true){
        memset(buf, 0, 9);

        esp_task_wdt_reset();

        // wait a long time for our bytes to come in
        i2c_slave_read_buffer(I2C_NUM_0, buf, 9, 2048);

        if (buf[0] == I2C_BEGIN_BYTE){
            // acquire semaphore: stop other threads from changing data while we modify it
            if (xSemaphoreTake(rdSem, pdMS_TO_TICKS(25))){
                receivedData.tsopAngle = UNPACK_16(buf[1], buf[2]);
                receivedData.tsopStrength = UNPACK_16(buf[3], buf[4]);
                receivedData.lineAngle = UNPACK_16(buf[5], buf[6]);
                receivedData.lineSize = UNPACK_16(buf[7], buf[8]);
            
                ESP_LOGV(TAG, "Received: %d, %d, %d, %d", receivedData.tsopAngle, receivedData.tsopStrength, 
                        receivedData.lineAngle, receivedData.lineSize);    
                // unlock the semaphore, other tasks can use the new data now
                xSemaphoreGive(rdSem);
            } else {
                ESP_LOGW(TAG, "Failed to acquire semaphore in time!");
            }
        } else {
            ESP_LOGE(TAG, "Discarding invalid buffer, first byte is: %d, expected: %d", buf[0], I2C_BEGIN_BYTE);
        }

        esp_task_wdt_reset();
    }
}

void comms_i2c_init_master(i2c_port_t port){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 500000, // 0.5 MHz, max is 1 MHz, unit is Hz
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));

    // default = 64000, this is a hack to make the BNO work with its clock stretching
    i2c_set_timeout(I2C_NUM_1, 640000);

    ESP_LOGI("CommsI2C_M", "I2C init OK as master (RL slave) on bus %d", port);
}

void comms_i2c_init_slave(){
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_ESP_SLAVE_ADDR
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    // min size is 100 bytes, so we use a 32 * 9 byte buffer
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 288, 288, 0));
    // we don't need to pin this task (no need to access FPU), so the scheduler will put it on whichever core is doing 
    // the least amount of work (at least I think it will)
    xTaskCreate(comms_i2c_receive_task, "I2CReceiveTask", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI("CommsI2C_S", "I2C init OK as slave (RL master)");
}

// TODO make an I2C struct
void comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE));

    ESP_LOGV("CommsI2C_M", "Sending: %d, %d, %d, %d", tsopAngle, tsopStrength, lineAngle, lineSize);
    
    // temp 9 byte buffer on the stack to expand out 4 16 bit integers into 8 8 bit integers + 1 start byte
    uint8_t* buf = alloca(9);
    buf[0] = I2C_BEGIN_BYTE;
    buf[1] = HIGH_BYTE_16(tsopAngle);
    buf[2] = LOW_BYTE_16(tsopAngle);
    buf[3] = HIGH_BYTE_16(tsopStrength);
    buf[4] = LOW_BYTE_16(tsopStrength);
    buf[5] = HIGH_BYTE_16(lineAngle);
    buf[6] = LOW_BYTE_16(lineAngle);
    buf[7] = HIGH_BYTE_16(lineSize);
    buf[8] = LOW_BYTE_16(lineSize);

    ESP_ERROR_CHECK(i2c_master_write(cmd, buf, 9, I2C_ACK_MODE));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));

    if (err != ESP_OK){
        ESP_LOGE("CommsI2C_M", "Send failed! Error: %s. Attemtping bus reset.", esp_err_to_name(err));
        i2c_reset_tx_fifo(I2C_NUM_0);
        i2c_reset_rx_fifo(I2C_NUM_0);
    }

    i2c_cmd_link_delete(cmd);
}