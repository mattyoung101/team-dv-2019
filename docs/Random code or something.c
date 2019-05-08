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
    ESP_LOGD(TAG, "Gyro: %f %f %f .... Accel: %f %f %f .... Mag: %f %f %f", 
    gyroReal[0], gyroReal[1], gyroReal[2], accelReal[0], accelReal[1], accelReal[2], magReal[0], magReal[1], magReal[2]);

    // run Madgwick's 9-axis sensor fusion algorithm
    MadgwickAHRSupdate(gyroReal[0] * DEG_RAD, gyroReal[1] * DEG_RAD, gyroReal[2] * DEG_RAD, accelReal[0], accelReal[1], 
    accelReal[2], magReal[0], magReal[1], magReal[2]);

    ESP_LOGD(TAG, "Quaternion: %f %f %f %f", q0, q1, q2, q3);

    // convert the quaternion to Euler angles - we only care about yaw
    // w = q0, x = q1, y = q2, z = q3 
    float siny_cosp = +2.0f * (q0 * q3 + q1 * q2);
	float cosy_cosp = +1.0f - 2.0f * (q2 * q2 + q3 * q3);  
	mpuYaw = atan2f(siny_cosp, cosy_cosp);
    // the 11.0f is the magnetic declination in Brisbane, it's 12.6 in Sydney for the comp
    mpuYaw = fmodf(mpuYaw * RAD_DEG + 360.0f, 360.0f); 

    mpuw_calc_mag_heading(magReal);

    ESP_LOGD(TAG, "Final yaw: %f\n", mpuYaw);