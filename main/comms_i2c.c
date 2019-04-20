#include "comms_i2c.h"
#include "HandmadeMath.h"

SemaphoreHandle_t rdSem = NULL;
i2c_data_t receivedData = {0};
static const char *TAG = "CommsI2C";

static void comms_i2c_receive_task(void *pvParameters){
    static const char *TAG = "I2CReceiveTask";
    uint8_t *buf = calloc(11, sizeof(uint8_t));
    rdSem = xSemaphoreCreateMutex();
    xSemaphoreGive(rdSem);

    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Slave I2C task init OK");

    while (true){
        memset(buf, 0, 11);

        esp_task_wdt_reset();

        // wait slightly shorter than the task watchdog timeout for us to get some data
        i2c_slave_read_buffer(I2C_NUM_0, buf, 11, pdMS_TO_TICKS(4000));

        if (buf[0] == I2C_BEGIN_DEFAULT){
            // we got sensor data
            if (xSemaphoreTake(rdSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                receivedData.tsopAngle = UNPACK_16(buf[1], buf[2]);
                receivedData.tsopStrength = UNPACK_16(buf[3], buf[4]);
                receivedData.lineAngle = UNPACK_16(buf[5], buf[6]);
                receivedData.lineSize = UNPACK_16(buf[7], buf[8]);
                receivedData.heading = UNPACK_16(buf[9], buf[10]);
            
                // ESP_LOGD(TAG, "Received: %d, %d, %d, %d, %d", receivedData.tsopAngle, receivedData.tsopStrength, 
                    // receivedData.lineAngle, receivedData.lineSize, receivedData.heading);

                // ESP_LOGD(TAG, "Highbyte: %d, Lowbyte: %d, Value: %d", buf[7], buf[8], receivedData.lineSize);

                xSemaphoreGive(rdSem);
            } else {
                ESP_LOGW(TAG, "Failed to acquire semaphore in time!");
            }
        } else {
            ESP_LOGE(TAG, "Invalid buffer, first byte is: 0x%X", buf[0]);
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
        .master.clk_speed = 50000, // 0.5 MHz, max is 1 MHz, unit is Hz --- NOTE: 1MHz is so fast that every second packet has 4 bits dropped off from it lol
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));

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

/** Internal send data function **/
static int comms_i2c_send_data(uint8_t *buf, size_t bufSize){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE));

    ESP_ERROR_CHECK(i2c_master_write(cmd, buf, bufSize, I2C_ACK_MODE));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    I2C_ERR_CHECK(err);

    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

int comms_i2c_send(uint16_t tsopAngle, uint16_t tsopStrength, uint16_t lineAngle, uint16_t lineSize, uint16_t heading){
    // ESP_LOGV("CommsI2C_M", "Sending: %d, %d, %d, %d", tsopAngle, tsopStrength, lineAngle, lineSize);
    
    // temp 9 byte buffer on the stack to expand out 4 16 bit integers into 8 8 bit integers + 1 start byte
    uint8_t *buf = alloca(11);
    buf[0] = I2C_BEGIN_DEFAULT;
    buf[1] = HIGH_BYTE_16(tsopAngle);
    buf[2] = LOW_BYTE_16(tsopAngle);
    buf[3] = HIGH_BYTE_16(tsopStrength);
    buf[4] = LOW_BYTE_16(tsopStrength);
    buf[5] = HIGH_BYTE_16(lineAngle);
    buf[6] = LOW_BYTE_16(lineAngle);
    buf[7] = HIGH_BYTE_16(lineSize);
    buf[8] = LOW_BYTE_16(lineSize);
    buf[9] = HIGH_BYTE_16(heading);
    buf[10] = LOW_BYTE_16(heading);

    // ESP_LOGD("CommsI2C_M", "Sending: Highbyte: %d, Lowbyte: %d", buf[7], buf[8]);

    return comms_i2c_send_data(buf, 11);
}