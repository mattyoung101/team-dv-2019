#include "states.h"
#include "simple_imu.h"

robot_state_t robotState = {0};
SemaphoreHandle_t robotStateSem = NULL;

pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};
pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};
pid_config_t goalPID = {GOAL_KP, GOAL_KI, GOAL_KD, GOAL_MAX_CORRECTION};

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_nothing_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
fsm_state_t stateAttackDribble = {&state_nothing_enter, &state_nothing_exit, &state_attack_dribble_update, "AttackDribble"};

// shortcut lol
#define rs robotState

static void imu_correction(){
    robotState.outOrientation = (int16_t) -pid_update(&headingPID, floatMod(floatMod((float)robotState.inHeading, 360.0f) 
                                + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
    // printf("IMU Correcting: %d\n", robotState.outOrientation);
}

static void goal_correction(){
    if (!robotState.inGoalVisible){
        // if the goal is visible use goal correction
        robotState.outOrientation = (int16_t) pid_update(&goalPID, floatMod(floatMod((float)robotState.inGoalAngle, 360.0f) 
                                    + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
    } else {
        // otherwise just IMU correct
        imu_correction();
    }
}

// Idle
void state_attack_idle_update(state_machine_t *fsm){
    printf("Do not use the idle state! It's not implemented yet.\n");
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    imu_correction();

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball is not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength >= ORBIT_DIST){
        ESP_LOGD(TAG, "Ball too close, switching to orbit");
        FSM_CHANGE_STATE(Orbit);
    }

    ESP_LOGD(TAG, "Ball is visible, pursuing");
    // Quickly approach the ball
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inBallAngle;
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    static const char *TAG = "OrbitState";
    imu_correction();

    // Check criteria:
    // Ball too far away, Ball too close (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallStrength);
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength < ORBIT_DIST){
        ESP_LOGD(TAG, "Ball too far away, reverting, strength: %d", robotState.inBallStrength);
        // printf("Ignoring switch request\n");
        FSM_CHANGE_STATE(Pursue);
    } else if (rs.inBallStrength >= ORBIT_DIST && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD(TAG, "Ball and angle in correct spot, switching to dribble, strength: %d, angle: %d",
        robotState.inBallStrength, robotState.inBallAngle);
        // printf("Ignoring switch request\n");
        FSM_CHANGE_STATE(Dribble);
    }

    int16_t tempAngle = rs.inBallAngle > 180 ? rs.inBallAngle - 360 : rs.inBallAngle;

    ESP_LOGD(TAG, "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(tempAngle)) * fminf(90, 
                                0.2 * powf(E, 0.2 * (float)smallestAngleBetween(tempAngle, 0))));
    float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / 
                            ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.2 * strengthFactor * powf(E, 2.5 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
    robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * 
                            (1.0 - (float)fabsf(angleAddition) / 90.0);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    imu_correction();

    // Check criteria:
    // Ball too far away, Ball not in front of us, Goal not visible, Ball not visible
    if (robotState.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallAngle);
        FSM_MOTOR_BRAKE;
    } else if (!robotState.inGoalVisible){
        ESP_LOGD(TAG, "Goal not visible, braking");
        // FSM_MOTOR_BRAKE;
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD(TAG, "Ball not in front, reverting, angle: %d", robotState.inBallAngle);
        FSM_CHANGE_STATE(Orbit);
    } else if (rs.inBallStrength <= DRIBBLE_BALL_TOO_FAR){
        ESP_LOGD(TAG, "Ball too far away, reverting, strength: %d", robotState.inBallStrength);
        FSM_CHANGE_STATE(Pursue);
    }

    // rush towards goal
    ESP_LOGD(TAG, "Rushing goal");
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inBallAngle;//robotState.inGoalAngle;
}

// done with this macro
#undef rs