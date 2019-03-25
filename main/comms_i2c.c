#include "comms_i2c.h"

void comms_i2c_init_master(){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 69, // TODO put in pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 69, // TODO put in pin
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 10000 // TODO is this in Hz or KHz or what?
    };
    // TODO what is the master port?
    ESP_ERROR_CHECK(i2c_param_config(69, &conf));
    // TODO do we really want to disable FIFO buffer?
    ESP_ERROR_CHECK(i2c_driver_install(69, conf.mode, 0, 0, 0));
}

void comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE);
    
    // temp 9 byte buffer on the stack to expand out 4 16 bit integers into 8 8 bit integers + 1 start byte
    uint8_t* buf = alloca(9);

    ESP_LOGD("CommsI2C_M", "Sending: %d, %d, %d, %d", tsopAngle, tsopStrength, lineAngle, lineSize);
    i2c_master_write(cmd, buf, 9, I2C_ACK_MODE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));

    if (err != ESP_OK){
        ESP_LOGE("CommsI2C_M", "Send failed! Error: %s. Attemtping bus reset.", esp_err_to_name(err));
        i2c_reset_tx_fifo(I2C_NUM_0);
    }

}