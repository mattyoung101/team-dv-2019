#include "mpu_wrapper.h"
#include "MadgwickAHRS.h"
#include "esp_timer.h"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "rom/ets_sys.h"

// Note: this file is commented out so its not compiled and doesn't generate warnings and doesn't fill up
// space on the ESP32

// static const char *TAG = "MPU9250_W";
// float mpuYaw = 0.0f;
// float mpuMagYaw = 0.0f;
// static float gyroSens = 1.0f;
// static uint16_t accelSens = 1;
// static float magSens = 32760.0f / 4915.0f; // some constant from Sparkfun docs
// static float lastYaw = 0.0f;
// static int8_t accuracy = 3;

// static const int8_t gyroOrientation[] = { 1, 0, 0,
//                                         0, 1, 0,
//                                         0, 0, 1};

// static const int8_t compassOrientation[] = {-1, 0, 0,
//                                             0, 1, 0,
//                                             0, 0,-1};

/** Just initialises the DMP **/
// static void mpuw_dmp_init(){
    // int16_t err = dmp_load_motion_driver_firmware();
    // err += dmp_enable_gyro_cal(true);
    // // there's a known bug where if tap detection is not enabled, the DMP FIFO rate is stuck at 200 Hz
    // err += dmp_enable_feature(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_TAP);
    // err += dmp_set_fifo_rate(DMP_RATE);
    // err += mpu_set_dmp_state(true);

    // if (err == 0){
    //     ESP_LOGI(TAG, "DMP init OK!");
    // } else {
    //     ESP_LOGE(TAG, "DMP init error! Status codes: %d", err);
    // }
// }

// TODO split these up into a bunch of smaller functions
// void mpuw_init(){
//     struct int_param_s int_param;
//     int16_t err = mpu_init(&int_param);
//     err += mpu_set_bypass(true);
//     err += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
//     err += mpu_set_sample_rate(DMP_RATE);
//     err += mpu_set_compass_sample_rate(100); // max out compass speed
    
//     if (err == 0){
//         ESP_LOGI(TAG, "MPU9250 init OK!");
//     } else {
//         ESP_LOGE(TAG, "MPU9250 init error! Status codes: %d", err);
//     }
    
//     ESP_LOGI(TAG, "Initialising the DMP...");
//     mpuw_dmp_init();

//     // run self test to determine bias values (i.e. calibration)
//     ESP_LOGI(TAG, "Running self test...");
//     long gyroBias[3], accelBias[3];
//     int16_t status = mpu_run_6500_self_test(gyroBias, accelBias, true);
//     ESP_LOGI(TAG, "Self test status code: %d", status); // should be 0b111 if 3 working sensors (7 dec)

//     // send the values off to the MPL
//     mpu_get_accel_sens(&accelSens);
//     accelBias[0] *= accelSens;
//     accelBias[1] *= accelSens;
//     accelBias[2] *= accelSens;
//     inv_set_accel_bias(accelBias, 3);
//     mpu_get_gyro_sens(&gyroSens);
//     gyroBias[0] = (long) (gyroBias[0] * gyroSens);
//     gyroBias[1] = (long) (gyroBias[1] * gyroSens);
//     gyroBias[2] = (long) (gyroBias[2] * gyroSens);
//     inv_set_gyro_bias(gyroBias, 3);
    
//     ESP_LOGI(TAG, "(soft) Gyro bias: %ld %ld %ld .... Accel bias: %ld %ld %ld", gyroBias[0], gyroBias[1], gyroBias[2], 
//     accelBias[0], accelBias[1], accelBias[2]);

//     // tell the DMP about our calibration values (the gyro should be automatically calibrated however)
//     dmp_set_gyro_bias(gyroBias);
//     dmp_set_accel_bias(accelBias);

//     // TODO use hardware cal registers see main.c stm32 example line 352

//     // setup the MPL
//     inv_set_accel_sample_rate(1000000L / DMP_RATE);
//     inv_set_quat_sample_rate(1000000L / DMP_RATE);
//     inv_set_gyro_sample_rate(1000000L / DMP_RATE);
//     inv_set_compass_sample_rate(5000); // 100 Hz in microseconds

//     uint16_t gyroFsr, compassFsr;
//     uint8_t accelFsr;
//     mpu_get_gyro_fsr(&gyroFsr);
//     mpu_get_accel_fsr(&accelFsr);
//     mpu_get_compass_fsr(&compassFsr);

//     inv_set_gyro_orientation_and_scale(
//             inv_orientation_matrix_to_scalar(gyroOrientation),
//             (long)gyroFsr<<15);
//     inv_set_accel_orientation_and_scale(
//             inv_orientation_matrix_to_scalar(gyroOrientation),
//             (long)accelFsr<<15);
//     inv_set_compass_orientation_and_scale(
//             inv_orientation_matrix_to_scalar(compassOrientation),
//             (long)compassFsr<<15);

//     // inv_enable_compass_soft_iron_matrix();
//     long unsigned int cur_time = esp_timer_get_time() / 1000;
//     // printf("Enable hal outputs: %d\n", inv_enable_hal_outputs());
//     // TODO we could use inv_get_compass_correction
// }

// static int dmpErrors = 0;

// static void mpuw_calc_mag_heading(float *magReal){
//     float my = magReal[1];
//     float mx = magReal[0];

//     if (my == 0)
// 		mpuMagYaw = (mx < 0) ? PI : 0;
// 	else
// 		mpuMagYaw = atan2f(mx, my);
	
// 	if (mpuMagYaw > PI) mpuMagYaw -= (2.0f * PI);
// 	else if (mpuMagYaw < -PI) mpuMagYaw += (2.0f * PI);
// 	else if (mpuMagYaw < 0) mpuMagYaw += 2.0f * PI;
	
// 	mpuMagYaw *= 180.0f / PI;

//     ESP_LOGD(TAG, "Mag yaw: %f", mpuMagYaw);
// }

// // Source: https://github.com/LuckyCIover/CFilter/blob/master/src/CFilter.h
// // gyroRads = gyro rads per second
// // magAngle = mag yaw (I assume, in the original its accel angle)
// // k = some constant, p = period
// static float complementary_filter_update(float k, float p, float gyroRads, float magAngle){
//     lastYaw = (1 - k) * (lastYaw + gyroRads * p * 0.001) + k * magAngle;
//     return lastYaw;
// }

// void mpuw_update(){
//     int16_t gyro[3] = {0};
//     int16_t accel[3] = {0};
//     long quat[4] = {0};
//     unsigned long timestamp;
//     int16_t sensors;
//     uint8_t more;

//     // read the FIFO buffer
//     if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0){
//         if (dmpErrors++ > 24){
//             ESP_LOGE(TAG, "Too many DMP errors!");
//             abort();
//         }
//         return;
//     }
//     dmpErrors = 0;

//     // make sure we got the values we need
//     if (!(sensors & INV_XYZ_GYRO) || !(sensors & INV_XYZ_ACCEL)) {
//         ESP_LOGW(TAG, "Did not receive correct sensor data! Got: %d", sensors);
//         return;
//     }

//     // get magnetometer readings
//     int16_t mag[3];
//     mpu_get_compass_reg(mag, &timestamp);
//     ESP_LOGI(TAG, "Raw mag: %d %d %d", mag[0], mag[1], mag[2]);

//     // get the values using the MPL
//     inv_build_gyro(gyro, timestamp);
//     inv_build_accel((const long*)accel, 0, timestamp);
//     inv_build_compass((const long*)mag, 0, timestamp);
//     inv_execute_on_data();

//     float gyroFinal[3] = {0};
//     inv_get_sensor_type_gyroscope(gyroFinal, &accuracy, &timestamp);
//     ESP_LOGI(TAG, "Gyro: %f %f %f", gyroFinal[0], gyroFinal[1], gyroFinal[2]);

//     float accelFinal[3] = {0};
//     inv_get_sensor_type_accelerometer(accelFinal, &accuracy, &timestamp);
//     ESP_LOGI(TAG, "Accel: %f %f %f", accelFinal[0], accelFinal[1], accelFinal[2]);

//     float magFinal[3] = {0};
//     inv_get_sensor_type_magnetic_field(magFinal, &accuracy, &timestamp);
//     ESP_LOGI(TAG, "Mag: %f %f %f", magFinal[0], magFinal[1], magFinal[2]);

//     puts("");

//     // ESP32 is little endian
// }

// void mpuw_mag_calibrate(){
//     // TODO ellipsoid magnetometer calibration
// }