#pragma once
#include "driver/adc.h"
#include "defines.h"
#include <stdint.h>
#include <math.h>
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/** x squared **/
#define sq(x) (x * x)
/** first 8 bits of unsigned 16 bit int **/
#define HIGH_BYTE_16(num) ((uint8_t) ((num >> 8) & 0xF))
/** second 8 bits of unsigned 16 bit int **/
#define LOW_BYTE_16(num)  ((uint8_t) ((num & 0xFF)))
/** unpack two 8 bit integers into a 16 bit integer **/
#define UNPACK_16(a, b) ((uint16_t) ((a << 8) | b))
/** halt in case of irrecoverable error **/
#define TASK_HALT do { ESP_LOGW(pcTaskGetTaskName(NULL), "Task halting!"); vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY)); } while(0);
/** Automated I2C error checking code for the sensor use ONLY **/
#define I2C_ERR_CHECK(err) do { if (err != ESP_OK){ \
        ESP_LOGE(TAG, "I2C failure! Error: %s. Attempting bus reset.", esp_err_to_name(err)); \
        i2c_reset_tx_fifo(I2C_NUM_0); \
        i2c_reset_rx_fifo(I2C_NUM_0); \
        return 1; \
    } } while (0);
/** Starts counting on the performance timer. The variable "pfBegin" must be undefined **/
#define PERF_TIMER_START int64_t pfBegin = esp_timer_get_time();
/** Stops the performance timer and logs to UART. **/
#define PERF_TIMER_STOP ESP_LOGD("PerfTimer", "%lld ms", (esp_timer_get_time() - pfBegin) / 1000);
/** Cosine in degrees of x in degrees **/
#define cosfd(x) (cosf(x * DEG_RAD) * RAD_DEG)
/** Sin in degrees of x in degrees **/
#define sinfd(x) (sinf(x * DEG_RAD) * RAD_DEG)

int32_t mod(int32_t x, int32_t m);
float floatMod(float x, float m);
int number_comparator_descending(const void *a, const void *b);
float angleBetween(float angleCounterClockwise, float angleClockwise);
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void i2c_scanner();