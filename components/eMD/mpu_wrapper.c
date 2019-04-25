#include "mpu_wrapper.h"

static const char *TAG = "MPU9250_W";

/** Just initialises the DMP **/
static void mpuw_dmp_init(){
    ESP_LOGI(TAG, "DMP init code: %d", dmp_load_motion_driver_firmware());
    int err = dmp_enable_6x_lp_quat(true);
    err += dmp_enable_gyro_cal(true);
    err += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
    err += dmp_set_fifo_rate(128); // 128 Hz
    err += mpu_set_dmp_state(true);
    ESP_LOGI(TAG, "Did the DMP fuck up? %s", err > 0 ? "Yes it did >:(" : "No it didn't!");
}

void mpuw_init(){
    struct int_param_s int_param;
    int result = mpu_init(&int_param);
    ESP_LOGI(TAG, "MPU9250 init code: %d", result);
    
    int err = mpu_set_bypass(true);
    err += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    ESP_LOGI(TAG, "Did we init the sensor OK? %s", err > 0 ? "Nup" : "Yep");
    
    ESP_LOGI(TAG, "Initialising the DMP...");
    mpuw_dmp_init();
}

void mpuw_update(){
    
}