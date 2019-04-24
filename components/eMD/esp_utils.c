#include "esp_utils.h"
#include "defines.h"

// Code to integrate eMD with ESP32
// TODO rename to esp_mpu_i2c_impl.c
// Based on https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/tree/master/src/util

static const char *TAG = "MPU9250";

int esp_get_clock_ms(unsigned long *count){
    *count = esp_timer_get_time() / 1000;
    return 0;
}

int esp_delay_ms(unsigned long num_ms){
    ets_delay_us(num_ms / 1000);
    return 0;
}

int esp_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data){
    ESP_LOGV(TAG, "Writing to 0x%X reg addr 0x%X length %d data[0]=0x%X", slave_addr, reg_addr, length, data[0]);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, length, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);

    I2C_ERR_CHECK(err);
    ESP_LOGV(TAG, "Write succeeded!");
    return 0;
}

int esp_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * data){
    ESP_LOGV(TAG, "Reading from 0x%X reg addr 0x%X length %d", slave_addr, reg_addr, length);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (slave_addr << 1), I2C_ACK_MODE);
    // send register we want
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_MODE);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, I2C_ACK_MODE);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, 0x0);
    }
    i2c_master_read_byte(cmd, data + length - 1, 0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);

    I2C_ERR_CHECK(ret);
    ESP_LOGV(TAG, "Read successful");

    return 0;
}