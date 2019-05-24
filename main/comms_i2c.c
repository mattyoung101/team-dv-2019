#include "comms_i2c.h"
#include "HandmadeMath.h"
#include "states.h"

SemaphoreHandle_t rdSem = NULL;
i2c_data_t receivedData = {0};
nano_data_t nanoData = {0};
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
        // ESP_LOG_BUFFER_HEX("I2C", buf, 11);

        if (buf[0] == I2C_BEGIN_DEFAULT){
            // we got sensor data
            if (xSemaphoreTake(rdSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                receivedData.tsopAngle = UNPACK_16(buf[1], buf[2]);
                receivedData.tsopStrength = UNPACK_16(buf[3], buf[4]);
                receivedData.lineAngle = UNPACK_16(buf[5], buf[6]);
                receivedData.lineSize = UNPACK_16(buf[7], buf[8]);
                receivedData.heading = UNPACK_16(buf[9], buf[10]);
                xSemaphoreGive(rdSem);
            } else {
                ESP_LOGW(TAG, "Failed to acquire received data semaphore in time!");
            }
        } else {
            ESP_LOGW(TAG, "Invalid buffer, first byte is: 0x%X", buf[0]);
        }

        esp_task_wdt_reset();
    }
}

/** sends/receives data from the Arduino Nano LS slave **/
static void nano_comms_task(void *pvParameters){
    static const char *TAG = "NanoCommsTask";
    uint8_t buf[9] = {0};
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Nano comms task init OK");

    while (true){
        /*
        float inLineAngle: 2 bytes
        float inLineSize: 2 bytes
        bool inOnLine: 1 byte
        bool inLineOver: 1 byte
        float inLastAngle: 2 bytes
        = 8 bytes + 1 start byte 
        = 9 bytes total
        */
        memset(buf, 0, NANO_PACKET_SIZE);
        nano_read(I2C_NANO_SLAVE_ADDR, NANO_PACKET_SIZE, buf);

        if (buf[0] == I2C_BEGIN_DEFAULT){
            if (xSemaphoreTake(robotStateSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                // buf[0] is the begin byte, so start from buf[1]
                nanoData.lineAngle = UNPACK_16(buf[1], buf[2]) / IMU_MULTIPLIER;
                nanoData.lineSize = UNPACK_16(buf[3], buf[4]) / IMU_MULTIPLIER;
                nanoData.isOnLine = (bool) buf[5];
                nanoData.isLineOver = (bool) buf[6];
                nanoData.lastAngle = UNPACK_16(buf[7], buf[8]) / IMU_MULTIPLIER;
                xSemaphoreGive(robotStateSem);
            } else {
                ESP_LOGW(TAG, "Failed to acquire robot state semaphore in time!");
            }
        } else {
            ESP_LOGW(TAG, "Invalid buffer, first byte is: 0x%X", buf[0]);
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
        // 0.5 MHz, max is 1 MHz, unit is Hz
        // NOTE: 1MHz tends to break the i2c packets - use with caution!!
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));

    // xTaskCreate(nano_comms_task, "NanoCommsTask", 3096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI("CommsI2C_M", "I2C init OK as master (RL slave) on bus %d", port);
}

void comms_i2c_init_slave(void){
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
    // min size is of i2c fifo buffer is 100 bytes, so we use a 32 * 9 byte buffer
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 288, 288, 0));
    xTaskCreate(comms_i2c_receive_task, "I2CReceiveTask", 3096, NULL, configMAX_PRIORITIES - 1, NULL);

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
    // temp 11 byte buffer on the stack to expand out 5 16 bit integers into 10 8 bit integers + 1 start byte
    // TODO use a static array and not alloca, saves memory concerns/stack overflow issues
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

    return comms_i2c_send_data(buf, 11);
}