#include "mpu_wrapper.h"

static const char *TAG = "MPU9250_W";
float mpuYaw = 0.0f;
float mpuErrorCorrection = 0.0f;

/** Just initialises the DMP **/
static void mpuw_dmp_init(){
    int err = dmp_load_motion_driver_firmware();
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
    int err = mpu_init(&int_param);
    err += mpu_set_bypass(true);
    err += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    err += mpu_set_compass_sample_rate(100); // default is 10 Hz, let's run the compass faster than that
    
    if (err == 0){
        ESP_LOGI(TAG, "MPU9250 init OK!");
    } else {
        ESP_LOGE(TAG, "MPU9250 init error! Status codes: %d", err);
    }

    ESP_LOGI(TAG, "Running self test...");
    long gyroBias[3], accelBias[3];
    int status = mpu_run_6500_self_test(gyroBias, accelBias, true);
    ESP_LOGI(TAG, "Self test status code: %d", status); // should be 0b111 if 3 working sensors (7 dec)
    
    ESP_LOGI(TAG, "Initialising the DMP...");
    mpuw_dmp_init();

    // tell the DMP about our calibration values
    dmp_set_gyro_bias(gyroBias);
    dmp_set_accel_bias(accelBias);
}

static float q_to_float(long number, unsigned char q){
	unsigned long mask = 0;
	for (int i = 0; i < q; i++){
		mask |= (1 << i);
	}
	return (number >> q) + ((number & mask) / (float) (2 << (q - 1)));
}

void mpuw_update(){
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float qw, qx, qy, qz;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0){
        // ESP_LOGW(TAG, "DMP FIFO read failed.");
        return;
    }

    // convert quaternion values in Q long format to floats
    if (sensors & INV_WXYZ_QUAT){
        qw = q_to_float(quat[0], 30);
        qx = q_to_float(quat[1], 30);
        qy = q_to_float(quat[2], 30);
        qz = q_to_float(quat[3], 30);
    } else {
        ESP_LOGW(TAG, "Did not receive quaternion values!");
        return;
    }

    // convert the quaternion to Euler angles - we only care about yaw
    float ysqr = qy * qy;
    float t0 = -2.0f * (ysqr + qz * qz) + 1.0f;
    float t1 = +2.0f * (qx * qy - qw * qz);
    mpuYaw = atan2f(t1, t0);
    mpuYaw *= (180.0f / PI);
    if (mpuYaw < 0) mpuYaw = 360.0f + mpuYaw;
    mpuYaw -= mpuErrorCorrection;

    // let's see if we can get magnetometer readings
    short data[3];
    int err = mpu_get_compass_reg(data, NULL);
    ESP_LOGD(TAG, "Did it work? %d (%d, %d, %d)", err, data[0], data[1], data[2]);

    ESP_LOGD(TAG, "Yaw: %f", mpuYaw);
}