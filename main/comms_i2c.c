#include "comms_i2c.h"

static void comms_i2c_receive_task(void *pvParameters){
    uint8_t* buf = malloc(9);
    static const char *TAG = "I2CReceiveTask";
    ESP_LOGI(TAG, "Slave I2C task init OK");

    while (true){
        memset(buf, 0, 9);

        // wait indefinitely for our bytes to come in
        size_t count = i2c_slave_read_buffer(I2C_NUM_0, buf, 9, portMAX_DELAY);

        if (buf[0] == I2C_BEGIN_BYTE){
            // the buffer is valid (we have the begin byte), so process it
            ESP_LOGD(TAG, "Received %d bytes successfully", count);
            receivedData.tsopAngle = UNPACK_16(buf[1], buf[2]);
            receivedData.tsopStrength = UNPACK_16(buf[3], buf[4]);
            receivedData.lineAngle = UNPACK_16(buf[5], buf[6]);
            receivedData.lineSize = UNPACK_16(buf[7], buf[8]);
        } else {
            ESP_LOGW(TAG, "Invalid buffer, first byte is: %d", buf[0]);
        }

        // TODO how are you supposed to reset the buffer? is it done automatically?
    }
}

void comms_i2c_init_master(){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    ESP_LOGI("CommsI2C_M", "I2C init OK as master (RL slave)");
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
    // buf size = 2 * packet size = 18
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 18, 18, 0));
    // we don't need to pin this task (no need to access FPU), so the scheduler will put it on whichever core is doing 
    // the least work (at least I think it will)
    xTaskCreate(comms_i2c_receive_task, "I2CReceiveTask", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI("CommsI2C_S", "I2C init OK as slave (RL master)");
}

void comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE);

    ESP_LOGD("CommsI2C_M", "Sending: %d, %d, %d, %d", tsopAngle, tsopStrength, lineAngle, lineSize);
    
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

    i2c_master_write(cmd, buf, 9, I2C_ACK_MODE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));

    if (err != ESP_OK){
        ESP_LOGE("CommsI2C_M", "Send failed! Error: %s. Attemtping bus reset.", esp_err_to_name(err));
        i2c_reset_tx_fifo(I2C_NUM_0);
        i2c_reset_rx_fifo(I2C_NUM_0);
    }

    i2c_cmd_link_delete(cmd);
}