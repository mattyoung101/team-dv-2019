#include "mpu_wrapper.h"
#include "MadgwickAHRS.h"
#include "esp_timer.h"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "rom/ets_sys.h"

static const char *TAG = "MPU9250_W";
float mpuYaw = 0.0f;
float mpuMagYaw = 0.0f;
static float gyroSens = 1.0f;
static uint16_t accelSens = 1;
static float lastYaw = 0.0f;
static int dmpErrors = 0;
float gyroReal[3] = {0}, accelReal[3] = {0};
long gyroBias[3] = {0}, accelBias[3] = {0};
static int64_t previousTimeGyro = 0;

// Borrowed from Sparkfun lib, converts Q-float values into normal values
static float q_to_float(long number, unsigned char q){
	unsigned long mask = 0;
	for (int i = 0; i < q; i++){
		mask |= (1 << i);
	}
	return (number >> q) + ((number & mask) / (float) (2 << (q - 1)));
}

/** Just initialises the DMP **/
static void mpuw_dmp_init(){
    int16_t err = dmp_load_motion_driver_firmware();
    err += dmp_enable_gyro_cal(true);
    // there's a known bug where if tap detection is not enabled, the DMP FIFO rate is stuck at 200 Hz
    err += dmp_enable_feature(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | 
            DMP_FEATURE_TAP);
    err += dmp_set_fifo_rate(DMP_RATE);
    err += mpu_set_dmp_state(true);

    if (err == 0){
        ESP_LOGI(TAG, "DMP init OK!");
    } else {
        ESP_LOGE(TAG, "DMP init error! Status codes: %d", err);
    }
}

void mpuw_init(){
    struct int_param_s int_param;
    int16_t err = mpu_init(&int_param);
    err += mpu_set_bypass(true);
    err += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    err += mpu_set_sample_rate(DMP_RATE);
    
    if (err == 0){
        ESP_LOGI(TAG, "MPU9250 init OK!");
    } else {
        ESP_LOGE(TAG, "MPU9250 init error! Status codes: %d", err);
    }
    
    ESP_LOGI(TAG, "Initialising the DMP...");
    mpuw_dmp_init();

    // run self test to determine bias values (i.e. calibration)
    ESP_LOGI(TAG, "Running self test...");
    int16_t status = mpu_run_6500_self_test(gyroBias, accelBias, true);
    ESP_LOGI(TAG, "Self test status code: %d", status); // should be 0b111 if 3 working sensors (7 dec)

    // get sens values
    mpu_get_accel_sens(&accelSens);
    mpu_get_gyro_sens(&gyroSens);

    // tell the DMP about our calibration values (the gyro should be automatically calibrated however)
    dmp_set_gyro_bias(gyroBias);
    dmp_set_accel_bias(accelBias);

    uint16_t gyroFsr, compassFsr;
    uint8_t accelFsr;
    mpu_get_gyro_fsr(&gyroFsr);
    mpu_get_accel_fsr(&accelFsr);
    mpu_get_compass_fsr(&compassFsr);
}

static void mpuw_calc_mag_heading(float *magReal){
    float my = magReal[1];
    float mx = magReal[0];

    if (my == 0)
		mpuMagYaw = (mx < 0) ? PI : 0;
	else
		mpuMagYaw = atan2f(mx, my);
	
	if (mpuMagYaw > PI) mpuMagYaw -= (2.0f * PI);
	else if (mpuMagYaw < -PI) mpuMagYaw += (2.0f * PI);
	else if (mpuMagYaw < 0) mpuMagYaw += 2.0f * PI;
	
	mpuMagYaw *= 180.0f / PI;

    ESP_LOGD(TAG, "Mag yaw: %f", mpuMagYaw);
}

// Source: https://github.com/LuckyCIover/CFilter/blob/master/src/CFilter.h
// gyroRads = gyro rads per second
// magAngle = mag yaw (I assume, in the original its accel angle)
// k = some constant, p = period
static float complementary_filter_update(float k, float p, float gyroRads, float magAngle){
    lastYaw = (1 - k) * (lastYaw + gyroRads * p * 0.001) + k * magAngle;
    return lastYaw;
}

void mpuw_update(){
    int16_t gyro[3] = {0};
    int16_t accel[3] = {0};
    long quat[4] = {0};
    unsigned long timestamp;
    int16_t sensors;
    uint8_t more;

    // read the FIFO buffer
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0){
        if (dmpErrors++ > 24){
            ESP_LOGE(TAG, "Too many DMP errors!");
            abort();
        }
        return;
    }
    dmpErrors = 0;

    // make sure we got the values we need
    if (!(sensors & INV_XYZ_GYRO) || !(sensors & INV_XYZ_ACCEL)) {
        ESP_LOGW(TAG, "Did not receive correct sensor data! Got: %d", sensors);
        return;
    }

    // calculate legit values
    for (int i = 0; i < 3; i++){
        gyroReal[i] = ((float) gyro[i] / gyroSens) + (i == 2 ? 1.707f : 0.0f);
        accelReal[i] = ((float) accel[i] / accelSens);
    }

    // ESP_LOGD(TAG, "Gyro: %f, %f, %f ..... Accel: %f, %f, %f", gyroReal[0], gyroReal[1], gyroReal[2],
    // accelReal[0], accelReal[1], accelReal[2]);

    // calculate yaw
    int64_t currentTime = esp_timer_get_time();
    mpuYaw += -(((float)(currentTime - previousTimeGyro) / 1000000.0f) * (gyroReal[2]));
	mpuYaw = floatMod(mpuYaw, 360.0f);

    printfln("Current yaw: %f", mpuYaw);

    // ESP32 is little endian
}