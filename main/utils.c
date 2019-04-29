#include "utils.h"

pid_config_t goalPID = {GOAL_KP, GOAL_KI, GOAL_KD, GOAL_MAX_CORRECTION};
pid_config_t headingPID = {HEADING_KP, HEADING_KI, HEADING_KD, HEADING_MAX_CORRECTION};
pid_config_t idlePID = {IDLE_KP, IDLE_KI, IDLE_KD, IDLE_MAX_CORRECTION};
pid_config_t coordPID = {COORD_KP, COORD_KI, COORD_KP, COORD_MAX};

int32_t mod(int32_t x, int32_t m){
    int32_t r = x % m;
    return r < 0 ? r + m : r;
}

float floatMod(float x, float m) {
    float r = fmodf(x, m);
    return r < 0 ? r + m : r;
}

int number_comparator_descending(const void *a, const void *b){
    // descending order so b - a
    return (*(int*)b - *(int*)a);
}

// TODO rename to getAngleBetween
float angleBetween(float angleCounterClockwise, float angleClockwise){
    return mod(angleClockwise - angleCounterClockwise, 360);
}

float smallestAngleBetween(float angle1, float angle2){
    float ang = angleBetween(angle1, angle2);
    return fminf(ang, 360 - ang);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise){
    return mod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0, 360);
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool is_angle_between(float target, float angle1, float angle2){
	// make the angle from angle1 to angle2 to be <= 180 degrees
	// int rAngle = ((angle2 - angle1) % 360 + 360) % 360;
	float rAngle = fmodf(fmodf(angle2 - angle1, 360.0f) + 360.0f, 360.0f);
	if (rAngle >= 180.0f){
		// std::swap(angle1, angle2);
		float a1 = angle1;
		angle1 = angle2;
		angle2 = a1;
	}

	// check if it passes through zero
	if (angle1 <= angle2){
		return target >= angle1 && target <= angle2;
	} else {
		return target >= angle1 || target <= angle2;
	}
}

void imu_correction(robot_state_t *robotState){
    if (robotState->outSpeed == 0){
        robotState->outOrientation = (int16_t) -pid_update(&idlePID, floatMod(floatMod((float)robotState->inHeading, 360.0f) 
                                + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
        // printf("IDLE PID");
    } else {
        robotState->outOrientation = (int16_t) -pid_update(&headingPID, floatMod(floatMod((float)robotState->inHeading, 360.0f) 
                                + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
        // printf("HEADING PID");
    }
    
    // printf("IMU Correcting: %d\n", robotState->outOrientation);
}

void goal_correction(robot_state_t *robotState){
    if (robotState->inGoalVisible && robotState->inGoalLength < GOAL_TRACK_DIST){
        // if the goal is visible use goal correction
        robotState->outOrientation = (int16_t) pid_update(&goalPID, floatMod(floatMod((float)robotState->inGoalAngle, 360.0f) 
                                    + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
        // printf("Goal correction");
    } else {
        // otherwise just IMU correct
        imu_correction(robotState);
        // printf("Cannot see goal");
    }
}

void move_to_xy(robot_state_t *robotState, int16_t x, int16_t y){
    if(robotState->inGoalVisible){
        // Goal is visible, moving to coordinate
        if(sqrtf(powf(abs(x - robotState->inX), 2) + powf(abs(y - robotState->inY), 2)) < COORD_THRESHOLD){
            // We are at the coordinate
            robotState->outSpeed = 0;
            robotState->outShouldBrake = true;
        }else{
            // Move with PID
            robotState->outDirection = fmodf(fmodf(90.0 - DEG_RAD * (atan2(y - robotState->inY, x - robotState->inX)), 360) - robotState->inHeading, 360.0);
            robotState->outSpeed = abs(pid_update(&coordPID, sqrtf(powf(abs(x - robotState->inX), 2) + powf(abs(y - robotState->inY), 2)), 0, 0));
        }
    }else{
        robotState->outSpeed = 0;
        robotState->outShouldBrake = true;
    }
}

void i2c_scanner(){
    ESP_LOGI("I2CScanner", "Scanning...");

    int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 75 / portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		// ESP_LOGD("I2CScanner", "Addr 0x%X, RC %s", i, esp_err_to_name(espRc));
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
}