#include "comms_i2c.h"
#include "HandmadeMath.h"
#include "states.h"
#include "pb_decode.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"

i2c_data_t receivedData = {0};
nano_data_t nanoData = {0};
SensorUpdate lastSensorUpdate = SensorUpdate_init_zero;
// Semaphore for Protobuf messages
SemaphoreHandle_t pbSem = NULL;
SemaphoreHandle_t nanoDataSem = NULL;
// current position in the buffer that the I2C interrupt is writing to, must only be modified by the interrupt
static uint8_t bufPos = 0;
// I2C buffer
uint8_t buf[PROTOBUF_SIZE] = {0};
static buffer_status_t bstatus = BS_AVAILABLE;

static const char *TAG = "CommsI2C";

static void IRAM_ATTR i2c_copy_buffer(){
    int rx_fifo_cnt = I2C0.status_reg.rx_fifo_cnt;
    // ets_printf("\n[int] copying %d\n", rx_fifo_cnt);

    // copy bytes from hardware FIFO, based on https://git.io/fj0fl
    for (uint8_t i = 0; i < rx_fifo_cnt; i++) {
        uint8_t byte = I2C0.fifo_data.data;
        // ets_printf("\n[int] i: %d, bufpos: %d, byte: %d\n", i, bufPos, byte);
        buf[i + bufPos++] = byte;
    }
}

static bool IRAM_ATTR acquire_buf(){
    // ets_printf("\n[int] entering acquire_buf\n");
    if (bstatus == BS_TASK_WORKING){
        // task is currently working on the buffer, so skip the interrupt
        ets_printf("\n[int] skipping\n");
        return false;
    } else if (bstatus == BS_AVAILABLE){
        // claim the buffer for us
        ets_printf("\n[int] acquiring buf\n");
        bstatus = BS_ISR_WORKING;
        return true;
    } else {
        // we already own the buffer
        ets_printf("\n[int] already own buf\n");
        return true;
    }
}

// TODO: this isr needs to include bus errors, etc as the regular ISR isn't going off anymore
// TODO: could we alleviate these problems by using two buffers and memcpying with a semaphore?

/** used to fix problems with default interrupt in the I2C library that causes bytes to be out of order **/
static void IRAM_ATTR i2c_interrupt(void *arg){
    uint32_t status = I2C0.int_status.val;

    // TODO why the fuck does this time out?
    // I'm pretty sure it's to do with the fact that we need to service each and every single interrupt

    ets_printf("\n[int] entering intr\n");

    if (status & I2C_SLAVE_TRAN_COMP_INT_ST_M){
        if (acquire_buf()){
            ets_printf("\n[int] transmission complete\n");
            i2c_copy_buffer();

            // main task can safely read the buffer now
            bstatus = BS_AVAILABLE;
        }
    } else if (status & I2C_RXFIFO_FULL_INT_ST_M){
        if (acquire_buf()){
            ets_printf("\n[int] rx fifo full\n");
            i2c_copy_buffer();
        }
    }
}

static void comms_i2c_receive_task(void *pvParameters){
    static const char *TAG = "I2CReceiveTask";
    pbSem = xSemaphoreCreateMutex();
    xSemaphoreGive(pbSem);

    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Slave I2C task init OK");

    // TASK_HALT;

    while (true){
        if (bstatus == BS_AVAILABLE){
            ESP_LOGD(TAG, "Buffer available, acquiring it");
            bstatus = BS_TASK_WORKING;

            if (buf[0] == 0xB){
                uint8_t msgId = buf[1];
                uint8_t msgSize = buf[2];

                // remove the header by copying from byte 3 onwards, using memmove to save creating a new buffer
                memmove(&buf[3], buf, msgSize);

                ESP_LOG_BUFFER_HEX("I2CBuf", buf, PROTOBUF_SIZE);

                pb_istream_t stream = pb_istream_from_buffer(buf, PROTOBUF_SIZE);
                void *dest = NULL;
                void *msgFields = NULL;

                // assign destination struct based on message ID
                switch (msgId){
                    case MSG_SENSORUPDATE_ID:
                        dest = (void*) &lastSensorUpdate;
                        msgFields = (void*) &SensorUpdate_fields;
                        break;
                    default:
                        ESP_LOGE(TAG, "main task: Unknown message ID: %d", msgId);
                        continue;
                }

                // semaphore required since we use the protobuf messages outside this thread
                if (xSemaphoreTake(pbSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                    if (!pb_decode(&stream, msgFields, dest)){
                        ESP_LOGE(TAG, "Protobuf decode error for message ID %d: %s", msgId, PB_GET_ERROR(&stream));
                    } else {
                        ESP_LOGI(TAG, "Protobuf decode successful. Ball strength: %d, Ball dir: %d", lastSensorUpdate.tsopStrength,
                        lastSensorUpdate.tsopAngle);
                    }
                    
                    xSemaphoreGive(pbSem);
                } else {
                    ESP_LOGE(TAG, "Failed to unlock Protobuf semaphore!");
                }

                // reset buffer and position for ISR since we've read it
                memset(buf, 0, PROTOBUF_SIZE);
                bufPos = 0;
            } else {
                ESP_LOGE(TAG, "Invalid buffer, first byte is: 0x%X", buf[0]);
            }

            bstatus = BS_AVAILABLE;
        }
        
        esp_task_wdt_reset();
    }
}

/** sends/receives data from the Arduino Nano LS slave **/
static void nano_comms_task(void *pvParameters){
    static const char *TAG = "NanoCommsTask";
    uint8_t buf[9] = {0};
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
        = 8 bytes + 1 start byte 
        = 9 bytes total
        */
        memset(buf, 0, NANO_PACKET_SIZE);
        nano_read(I2C_NANO_SLAVE_ADDR, NANO_PACKET_SIZE, buf);

        if (buf[0] == I2C_BEGIN_DEFAULT){
            if (xSemaphoreTake(nanoDataSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
                nanoData.lineAngle = UNPACK_16(buf[1], buf[2]) / IMU_MULTIPLIER;
                nanoData.lineSize = UNPACK_16(buf[3], buf[4]) / IMU_MULTIPLIER;
                nanoData.isOnLine = (bool) buf[5];
                nanoData.isLineOver = (bool) buf[6];
                nanoData.lastAngle = UNPACK_16(buf[7], buf[8]) / IMU_MULTIPLIER;
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
        // 0.5 MHz, max is 1 MHz, unit is Hz
        // NOTE: 1MHz tends to break the i2c packets - use with caution!!
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));
    // Nano keeps timing out, so fuck it, let's yeet the timeout value. default value is 1600, max is 0xFFFFF
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF));

    // xTaskCreate(nano_comms_task, "NanoCommsTask", 3096, NULL, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI("CommsI2C_M", "I2C init OK as master (RL slave) on bus %d", port);
}

void comms_i2c_init_slave(void){
    intr_handle_t i2cInt;
    ESP_ERROR_CHECK(i2c_isr_register(I2C_NUM_0, &i2c_interrupt, NULL, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, &i2cInt));

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
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 256, 128, 0));

    xTaskCreate(comms_i2c_receive_task, "I2CReceiveTask", 8192, NULL, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI("CommsI2C_S", "I2C init OK as slave (RL master)");
}

int comms_i2c_write_protobuf(uint8_t *buf, size_t msgSize, uint8_t msgId){
    uint8_t header[] = {0xB, msgId, msgSize};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE));

    ESP_ERROR_CHECK(i2c_master_write(cmd, header, 3, I2C_ACK_MODE)); // write header
    ESP_ERROR_CHECK(i2c_master_write(cmd, buf, msgSize, I2C_ACK_MODE)); // write buffer

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
    I2C_ERR_CHECK(err);

    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}