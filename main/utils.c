#include "utils.h"

// Hecking PIDs
// Orientation Correction PIDs
pid_config_t goalPID = {GOAL_KP, GOAL_KI, GOAL_KD, GOAL_MAX_CORRECTION};
pid_config_t headingPID = {HEADING_KP, HEADING_KI, HEADING_KD, HEADING_MAX_CORRECTION};
pid_config_t idlePID = {IDLE_KP, IDLE_KI, IDLE_KD, IDLE_MAX_CORRECTION};
pid_config_t goaliePID = {GOALIE_KP, GOALIE_KI, GOALIE_KD, GOALIE_MAX};

// Movement PIDs
pid_config_t coordPID = {COORD_KP, COORD_KI, COORD_KP, COORD_MAX};
pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};
pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};

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
        if (robotState->outIsAttack){
            robotState->outOrientation = (int16_t) pid_update(&goalPID, floatMod(floatMod((float)robotState->inGoalAngle, 360.0f) 
                                    + 180.0f, 360.0f) - 180.0f, 0.0f, 0.0f);
            // printf("Attack goal correction");
        } else {
            robotState->outOrientation = (int16_t) pid_update(&goaliePID, floatMod(floatMod((float)robotState->inGoalAngle, 360.0f)
                                    + 180.0f, 360.0f) - 180.0f, 0.0f, 0.0f); // Also I don't remember how the hell this works but apparently it did
            // printf("Defend goal correction");
        }
    } else {
        // otherwise just IMU correct
        imu_correction(robotState);
        // printf("Cannot see goal");
    }
}

float get_magnitude(int16_t x, int16_t y){
    return sqrtf((float) (x * x + y * y));
}

float get_angle(int16_t x, int16_t y){
    return fmodf(90 - RAD_DEG * (atan2(y, x)), 360.0f);
}

void move_by_difference(robot_state_t *robotState, int16_t x, int16_t y){
    if (get_magnitude(x, y) < COORD_THRESHOLD){
        robotState->outSpeed = 0;
        robotState->outShouldBrake = true;
    }else{
        robotState->outDirection = fmodf(get_angle(x, y) - robotState->inHeading, 360.0f);
        robotState->outSpeed = fabsf(pid_update(&coordPID, get_magnitude(x, y), 0.0f, 0.0f));
    }
}

void move_to_xy(robot_state_t *robotState, int16_t x, int16_t y){
    if (robotState->inGoalVisible){
        return move_by_difference(robotState, x - robotState->inX, y - robotState->inY);
    }else{
        robotState->outShouldBrake = true;
        robotState->outSpeed = 0;
    }
}

float lerp(float fromValue, float toValue, float progress){
    return fromValue + (toValue - fromValue) * progress;
}

void orbit(robot_state_t *robotState){
    // orbit requires angles in -180 to +180 range
    int16_t tempAngle = robotState->inBallAngle > 180 ? robotState->inBallAngle - 360 : robotState->inBallAngle;

    // ESP_LOGD(TAG, "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(tempAngle)) * fminf(90, 0.1 * powf(E, 0.1 * (float)smallestAngleBetween(tempAngle, 0))));
    float strengthFactor = constrain(((float)robotState->inBallStrength - (float)BALL_FAR_STRENGTH) / ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.1 * strengthFactor * powf(E, 2 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState->outDirection = floatMod(robotState->inBallAngle + angleAddition, 360);
    robotState->outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * (1.0 - (float)fabsf(angleAddition) / 90.0);
}

hmm_vec2 vec2_polar_to_cartesian(hmm_vec2 vec){
    // r cos theta, r sin theta
    // where r = X, theta = Y
    float r = vec.X;
    float theta = vec.Y;
    return HMM_Vec2(r * cosfd(theta), r * sinfd(theta));
}

hmm_vec2 vec2_cartesian_to_polar(hmm_vec2 vec){
    // from TSOP code: sqrtf(sq(sumX) + sq(sumY)), fmodf((atan2f(sumY, sumX) * RAD_DEG) + 360.0f, 360.0f)
    float x = vec.X;
    float y = vec.Y;
    return HMM_Vec2(sqrtf(sq(x) + sq(y)), fmodf((atan2f(y, x) * RAD_DEG) + 360.0f, 360.0f));
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