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
float mpuErrorCorrection = 0.0f;
static float gyroSens = 1.0f;
static uint16_t accelSens = 1;
static float magSens = 32760.0f / 4915.0f; // some constant from Sparkfun docs

/** Just initialises the DMP **/
static void mpuw_dmp_init(){
    int16_t err = dmp_load_motion_driver_firmware();
    err += dmp_enable_6x_lp_quat(true);
    err += dmp_enable_gyro_cal(true);
    // Sparkfun says you have to enable tap detection because there's a bug where the FIFO rate is incorrect if it's off
    err += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO |
                                DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_TAP);
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
    err += mpu_set_compass_sample_rate(100); // default is 10 Hz, let's run the compass faster than that
    
    if (err == 0){
        ESP_LOGI(TAG, "MPU9250 init OK!");
    } else {
        ESP_LOGE(TAG, "MPU9250 init error! Status codes: %d", err);
    }
    
    ESP_LOGI(TAG, "Initialising the DMP...");
    mpuw_dmp_init();

    // get unit conversion values (hardware units to actual units)
    mpu_get_gyro_sens(&gyroSens);
    mpu_get_accel_sens(&accelSens);

    // run self test to determine bias values (i.e. calibration)
    ESP_LOGI(TAG, "Running self test...");
    long gyroBias[3], accelBias[3];
    int16_t status = mpu_run_6500_self_test(gyroBias, accelBias, true);
    ESP_LOGI(TAG, "Self test status code: %d", status); // should be 0b111 if 3 working sensors (7 dec)

    // tell the DMP about our calibration values (the gyro should be automatically calibrated however)
    dmp_set_gyro_bias(gyroBias);
    dmp_set_accel_bias(accelBias);
}

// static float q_to_float(long number, unsigned char q){
// 	unsigned long mask = 0;
// 	for (int i = 0; i < q; i++){
// 		mask |= (1 << i);
// 	}
// 	return (number >> q) + ((number & mask) / (float) (2 << (q - 1)));
// }

void mpuw_update(){
    int16_t gyro[3];
    int16_t accel[3];
    long quat[4];
    unsigned long timestamp;
    int16_t sensors;
    uint8_t more;

    float gyroReal[3];
    float accelReal[3];
    float magReal[3];

    PERF_TIMER_START;

    // read the FIFO buffer
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0){
        // ESP_LOGW(TAG, "DMP FIFO read failed.");
        return;
    }

    // make sure we got the values we need
    if (!(sensors & INV_XYZ_GYRO) || !(sensors & INV_WXYZ_QUAT) || !(sensors & INV_XYZ_ACCEL)) {
        ESP_LOGW(TAG, "Did not receive correct sensor data!");
        return;
    }

    // get magnetometer readings
    int16_t mag[3];
    long unsigned int cur_time = esp_timer_get_time() / 1000;
    mpu_get_compass_reg(mag, &cur_time);

    // convert from hardware units to actual units
    for (int i = 0; i < 3; i++){
        gyroReal[i] = ((float) gyro[i] / (float) gyroSens);
    }
    for (int i = 0; i < 3; i++){
        accelReal[i] = ((float) accel[i] / (float) accelSens);
    }
    for (int i = 0; i < 3; i++){
        magReal[i] = ((float) mag[i] / (float) magSens);
    }
    ESP_LOGD(TAG, "Gyro: %f, %f, %f .... Accel: %f, %f, %f .... Mag: %f, %f, %f", 
    gyroReal[0], gyroReal[1], gyroReal[2], accelReal[0], accelReal[1], accelReal[2], magReal[0], magReal[1], magReal[2]);

    // run Madgwick's 9-axis sensor fusion algorithm
    MadgwickAHRSupdate(gyroReal[0] * DEG_RAD, gyroReal[1] * DEG_RAD, gyroReal[2] * DEG_RAD, accelReal[0], accelReal[1], 
    accelReal[2], magReal[0], magReal[1], magReal[2]);

    ESP_LOGD(TAG, "Quaternion: %f, %f, %f, %f", q0, q1, q2, q3);

    // convert the quaternion to Euler angles - we only care about yaw
    // w = q0, x = q1, y = q2, z = q3 
    float siny_cosp = +2.0f * (q0 * q3 + q1 * q2);
	float cosy_cosp = +1.0f - 2.0f * (q2 * q2 + q3 * q3);  
	mpuYaw = atan2f(siny_cosp, cosy_cosp);
    // the 11.0f is the magnetic declination in Brisbane, it's 12.6 in Sydney for the comp
    mpuYaw = fmodf(mpuYaw * RAD_DEG + 360.0f - 11.0f, 360.0f); 

    ESP_LOGD(TAG, "Final yaw: %f", mpuYaw);

    PERF_TIMER_STOP;
}

void mpuw_mag_calibrate(){
    // Based on https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

    ESP_LOGI(TAG, "Wave device around in a figure 8 until done");
    vTaskDelay(pdMS_TO_TICKS(4000));

    uint16_t sample_count = 1500; // 100 Hz sample rate
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
    float calibration[3] = {0};
    float scale[3] = {0};

    int16_t mag[3];

    for (int i = 0; i < sample_count; i++){  
        long unsigned int cur_time = esp_timer_get_time() / 1000;
        // as far as I can see from his code we don't need to convert these into real units yet
        // note that mpu_get_compass_reg applies the hardware factory calibration automatically
        mpu_get_compass_reg(mag, &cur_time);

        for (int j = 0; j < 3; j++) {
            if(mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
            if(mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
        }

        ets_delay_us(12000); // 100 Hz, delay 12 seconds (accuracy is important here)
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;

    // Convert to proper units
    calibration[0] = (float) mag_bias[0] / magSens; // get average x mag bias in counts
    calibration[1] = (float) mag_bias[1] / magSens; // get average y mag bias in counts
    calibration[2] = (float) mag_bias[2] / magSens; // get average z mag bias in counts

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    scale[0] = avg_rad / ((float) mag_scale[0]);
    scale[1] = avg_rad / ((float) mag_scale[1]);
    scale[2] = avg_rad / ((float) mag_scale[2]);

    ESP_LOGI(TAG, "Magnetometer calibration completed");
    ESP_LOGI(TAG, "Calibration values (hard iron): %f, %f, %f", calibration[0], calibration[1], calibration[2]);
    ESP_LOGI(TAG, "Scale values (soft iron): %f, %f, %f", scale[0], scale[1], scale[2]);
}