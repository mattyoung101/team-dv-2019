#include "comms_i2c.h"
#include "HandmadeMath.h"
#include "states.h"
#include "pb_decode.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"

i2c_data_t receivedData = {0};
nano_data_t nanoData = {0};
SensorUpdate lastSensorUpdate = SensorUpdate_init_zero;
SemaphoreHandle_t pbSem = NULL;
SemaphoreHandle_t nanoDataSem = NULL;

static const char *TAG = "CommsI2C";

// TODO packet timeout if we haven't received a packet on the master in x ms

static void comms_i2c_receive_task(void *pvParameters){
    static const char *TAG = "I2CReceiveTask";
    uint8_t buf[PROTOBUF_SIZE] = {0};
    uint8_t msg[PROTOBUF_SIZE] = {0};
    uint16_t goodPackets = 0;
    pbSem = xSemaphoreCreateMutex();
    xSemaphoreGive(pbSem);

    // esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Slave I2C task init OK");

    while (true){
        uint8_t byte = 0;
        uint8_t i = 0;
        memset(buf, 0, PROTOBUF_SIZE);
        memset(msg, 0, PROTOBUF_SIZE);

        // attempt to read in bytes one by one
        while (true){
            i2c_slave_read_buffer(I2C_NUM_0, &byte, 1, portMAX_DELAY);
            buf[i++] = byte;

            // if we've got the end byte (0xEE) then quit
            if (byte == 0xEE){
                break;
            }
        }

        if (buf[0] == 0xB){
            uint8_t msgId = buf[1];
            uint8_t msgSize = buf[2];

            // remove the header by copying from byte 3 onwards, excluding the end byte (0xEE)
            memcpy(msg, buf + 3, msgSize);

            pb_istream_t stream = pb_istream_from_buffer(msg, msgSize);
            void *dest = NULL;
            void *msgFields = NULL;

            // assign destination struct based on message ID
            switch (msgId){
                case MSG_SENSORUPDATE_ID:
                    dest = (void*) &lastSensorUpdate;
                    msgFields = (void*) &SensorUpdate_fields;
                    break;
                default:
                    ESP_LOGW(TAG, "main task: Unknown message ID: %d", msgId);
                    continue;
            }

            SensorUpdate oldUpdate = lastSensorUpdate;

            // semaphore required since we use the protobuf messages outside this thread
            if (xSemaphoreTake(pbSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                if (!pb_decode(&stream, msgFields, dest)){
                    ESP_LOGE(TAG, "Protobuf decode error for message ID %d: %s", msgId, PB_GET_ERROR(&stream));
                } else {
                    // to save the values from being 0, if heading or TSOP looks wrong, reject the message and restore
                    // the last one
                    // NOTE: this solution is far from ideal, but I'm in a rush and am unable to find whereabouts
                    // or why the valid is being set to zero
                    if (lastSensorUpdate.heading <= 0.01f && 
                        (lastSensorUpdate.tsopStrength <= 0.01f || lastSensorUpdate.tsopAngle <= 0.01f)){
                        // ESP_LOGW(TAG, "Rejecting invalid message, restoring last message");
                        lastSensorUpdate = oldUpdate;
                    } else {
                        goodPackets++;
                    }
                }
                
                xSemaphoreGive(pbSem);
            } else {
                ESP_LOGE(TAG, "Failed to unlock Protobuf semaphore!");
            }
        } else {
            // ESP_LOGW(TAG, "Invalid buffer, first byte is: 0x%X, previous good packets: %d", buf[0], goodPackets);
            goodPackets = 0;

            // reset I2C and try to correct the issue by waiting
            i2c_reset_rx_fifo(I2C_NUM_0);
            vTaskDelay(pdMS_TO_TICKS(15));
        }

        esp_task_wdt_reset();
    }
}

/** sends/receives data from the Arduino Nano LS slave **/
static void nano_comms_task(void *pvParameters){
    static const char *TAG = "NanoCommsTask";
    uint8_t buf[NANO_PACKET_SIZE] = {0};
    nanoDataSem = xSemaphoreCreateMutex();
    xSemaphoreGive(nanoDataSem);

    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Nano comms task init OK");

    while (true){
        /*
        float inLineAngle: 2 bytes
        float inLineSize: 2 bytes
        bool inOnLine: 1 byte
        bool inLineOver: 1 byte
        float inLastAngle: 2 bytes
        float batteryVoltage: 2 bytes
        = 10 bytes + 1 start byte 
        = 11 bytes total
        */
        memset(buf, 0, NANO_PACKET_SIZE);
        nano_read(I2C_NANO_SLAVE_ADDR, NANO_PACKET_SIZE, buf);

        if (buf[0] == I2C_BEGIN_DEFAULT){
            if (xSemaphoreTake(nanoDataSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                nanoData.lineAngle = UNPACK_16(buf[1], buf[2]) / I2C_MULTIPLIER;
                nanoData.lineSize = UNPACK_16(buf[3], buf[4]) / I2C_MULTIPLIER;
                nanoData.isOnLine = (bool) buf[5];
                nanoData.isLineOver = (bool) buf[6];
                nanoData.lastAngle = UNPACK_16(buf[7], buf[8]) / I2C_MULTIPLIER;
                nanoData.batteryVoltage = UNPACK_16(buf[9], buf[10]) / I2C_MULTIPLIER;
                xSemaphoreGive(nanoDataSem);
            } else {
                ESP_LOGE(TAG, "Failed to unlock nano data semaphore!");
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
        // 0.8 MHz, max is 1 MHz, unit is Hz
        // NOTE: 1MHz tends to break the i2c packets - use with caution!!
        .master.clk_speed = 800000,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));
    // Nano keeps timing out, so fuck it, let's yeet the timeout value. default value is 1600, max is 0xFFFFF
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFF));

    xTaskCreate(nano_comms_task, "NanoCommsTask", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
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
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 512, 128, 0));

    xTaskCreate(comms_i2c_receive_task, "I2CReceiveTask", 8192, NULL, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI("CommsI2C_S", "I2C init OK as slave (RL master)");
}

int comms_i2c_write_protobuf(uint8_t *buf, size_t msgSize, uint8_t msgId){
    uint8_t header[] = {0xB, msgId, msgSize};
    uint8_t finish = 0xEE;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE));

    ESP_ERROR_CHECK(i2c_master_write(cmd, header, 3, I2C_ACK_MODE)); // write header
    ESP_ERROR_CHECK(i2c_master_write(cmd, buf, msgSize, I2C_ACK_MODE)); // write buffer
    ESP_ERROR_CHECK(i2c_master_write(cmd, &finish, 1, I2C_ACK_MODE)); // write end byte

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    if (err != ESP_OK){
        ESP_LOGE(TAG, "I2C error in comms_i2c_write_protobuf");
        i2c_reset_tx_fifo(I2C_NUM_0);
        i2c_reset_rx_fifo(I2C_NUM_0);
        return err;
    }

    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}