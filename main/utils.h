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
#define HIGH_BYTE_16(num) ((uint8_t) ((num >> 8) & 0xFF))
/** second 8 bits of unsigned 16 bit int **/
#define LOW_BYTE_16(num)  ((uint8_t) ((num & 0xFF)))
/** unpack two 8 bit integers into a 16 bit integer **/
#define UNPACK_16(a, b) ((uint16_t) ((a << 8) | b))
/** halt in case of irrecoverable error **/
#define TASK_HALT do { ESP_LOGW(pcTaskGetTaskName(NULL), "Task halting!"); vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY)); } while(0);
/** Automated I2C error checking code **/
#define I2C_ERR_CHECK(err) do { if (err != ESP_OK){ \
        ESP_LOGE(TAG, "I2C failure in %s! Error: %s. Attempting bus reset.", __FUNCTION__, esp_err_to_name(err)); \
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
/** Sign function **/
#define sign(x) (copysignf(1.0f, x))
/** Brake the motors in the FSM **/
#define FSM_MOTOR_BRAKE do { \
    robotState.outSpeed = 0; \
    robotState.outShouldBrake = true; \
    return; \
} while (0);
/** Switch to a state in the FSM **/
#define FSM_CHANGE_STATE(STATE) do { fsm_change_state(fsm, &stateAttack ##STATE); return; } while (0);
/** Revert state in FSM **/
#define FSM_REVERT do { fsm_revert_state(fsm); return; } while (0);

int32_t mod(int32_t x, int32_t m);
float floatMod(float x, float m);
int number_comparator_descending(const void *a, const void *b);
float angleBetween(float angleCounterClockwise, float angleClockwise);
float smallestAngleBetween(float angle1, float angle2);
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void i2c_scanner();
/** Source: https://stackoverflow.com/a/11412077/5007892 **/
bool is_angle_between(float target, float angle1, float angle2);