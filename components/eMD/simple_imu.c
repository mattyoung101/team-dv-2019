#include "simple_imu.h"

static float calibrationGyro = 0.0f;
float heading = 0.0f;
static int64_t previousTimeGyro = 0;

// hack to fix shit
static inline void esp_hack(uint8_t addr, uint8_t reg, uint8_t count, uint8_t data){
    esp_i2c_write(addr, reg, count, &data);
}

void simu_init(void){
    esp_hack(MPU9250_ADDRESS, 29, 1, 0x06);
    esp_hack(MPU9250_ADDRESS, 26, 1, 0x06);
    esp_hack(MPU9250_ADDRESS, 27, 1, GYRO_FULL_SCALE_1000_DPS);
    esp_hack(MPU9250_ADDRESS, 28, 1, ACC_FULL_SCALE_2_G);
    esp_hack(MPU9250_ADDRESS, 0x37, 1, 0x02);
    esp_hack(MPU9250_ADDRESS, 0x0A, 1, 0x16);

    // make it go fast, 900 MHz
    uint16_t rate = 950;
    unsigned char data = 1000 / rate - 1;
    esp_i2c_write(MPU9250_ADDRESS, 0x19, 1, &data);

    previousTimeGyro = esp_timer_get_time();

    ESP_LOGI("SIMU", "MPU9250 init OK");
}

// static float convertRawAcceleration(int raw) {
//     // Since we are using 2G range
//     // -2g maps to a raw value of -32768
//     // +2g maps to a raw value of 32767

//     float a = (raw * 2.0) / 32768.0;
//     return a;
// }

static inline float convertRawGyro(int raw) {
    // Since we are using 1000 degrees/seconds range
    // -1000 maps to a raw value of -32768
    // +1000 maps to a raw value of 32767

    float g = (raw * 1000.0) / 32768.0;
    return g;
}

vec3d_t simu_read_gyro(void){
    uint8_t buffer[14];
    // I2Cread(MPU9250_ADDRESS, 0x3B, 14, buffer);
    esp_i2c_read((uint8_t) MPU9250_ADDRESS, (uint8_t) 0x3B, (uint8_t) 14, buffer);

    int16_t gx = -(buffer[8] << 8 | buffer[1]);
    int16_t gy = -(buffer[10] << 8 | buffer[11]);
    int16_t gz = buffer[12] << 8 | buffer[13];

    vec3d_t returnVector = {convertRawGyro(gx), convertRawGyro(gy), convertRawGyro(gz)};
    return returnVector;
}

void simu_calibrate(void){
    ESP_LOGI("SimpleIMU", "Calibration in progress...");
    
    for (int i = 0; i < IMU_CALIBRATION_COUNT; i++) {
        float readingGyro = (float) simu_read_gyro().z;
        calibrationGyro += readingGyro;
        vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_TIME));
    }

    calibrationGyro /= IMU_CALIBRATION_COUNT;
    ESP_LOGI("SimpleIMU", "Final calibration: %f", calibrationGyro);
    
    // calibrationGyro = -10.864258;

    ESP_LOGI("SIMU", "Calibration complete");
}

void inline simu_calc(){
    float reading = (float) simu_read_gyro().z;

	int64_t currentTime = esp_timer_get_time();
    heading += -(((float)(currentTime - previousTimeGyro) / 1000000.0f) * (reading - calibrationGyro));
	heading = floatMod(heading, 360.0f);

	previousTimeGyro = currentTime;
}