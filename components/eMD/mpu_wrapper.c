#include "mpu_wrapper.h"

static const char *TAG = "MPU9250_W";

/** Just initialises the DMP **/
static void mpuw_dmp_init(){
    int dmpInitCode = dmp_load_motion_driver_firmware();
    ESP_LOGI(TAG, "DMP init code: %d", dmpInitCode);
    
    int err = dmp_enable_6x_lp_quat(true);
    err += dmp_enable_gyro_cal(true);
    // Sparkfun says you enable to tap because there's a bug where the FIFO rate is incorrect if it's disabled
    err += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
    err += dmp_set_fifo_rate(180); // rate in Hz, max is 200
    err += mpu_set_dmp_state(true);
    ESP_LOGI(TAG, "DMP status: %s", err > 0 ? "Bad" : "Good");
}

void mpuw_init(){
    struct int_param_s int_param;
    int initErr = mpu_init(&int_param);
    ESP_LOGI(TAG, "MPU9250 init code: %d", initErr);
    
    int err = mpu_set_bypass(true);
    err += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    ESP_LOGI(TAG, "MPU basic init status: %s", err > 0 ? "Bad" : "Good");
    
    ESP_LOGI(TAG, "Initialising the DMP...");
    mpuw_dmp_init();

    #ifdef IMU_DEBUG
        ESP_LOGI(TAG, "Running self test...");
        long gyroCal[3], accelCal[3];
        int status = mpu_run_6500_self_test(gyroCal, accelCal, true);
        ESP_LOGI(TAG, "Self test status code: %d", status);
    #endif
}

// static int fifoErrors = 0; // only print warning if significant number of errors

static float q_to_float(long number, unsigned char q){
	unsigned long mask = 0;
	for (int i = 0; i < q; i++){
		mask |= (1 << i);
	}
	return (number >> q) + ((number & mask) / (float) (2 << (q - 1)));
}

static uint16_t counter = 0;

void mpuw_update(){
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float qw, qx, qy, qz;

    // TODO probably worth checking whether or not we have data available in the buffer
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0){
        // happens heaps and doesn't matter
        ESP_LOGW(TAG, "DMP FIFO read failed.");
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
    float yaw = atan2f(t1, t0);
    yaw *= (180.0f / PI);
    if (yaw < 0) yaw = 360.0f + yaw;

    ESP_LOGD(TAG, "Yaw: %f", yaw);
}