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
    robotState.outOrientation = (int16_t) -pid_update(&headingPID, floatMod(floatMod((float)robotState.inHeading, 360.0f) + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
    // printf("IMU Correcting: %d\n", robotState.outOrientation);
}

static void goal_correction(){
    if (!robotState.inGoalVisible){
        // if the goal is visible use goal correction
        robotState.outOrientation = (int16_t) pid_update(&goalPID, floatMod(floatMod((float)robotState.inGoalAngle, 360.0f) + 180.0f, 360.0f) - 180, 0.0f, 0.0f);
    } else {
        // otherwise just IMU correct
        imu_correction();
    }
}

// Idle
void state_attack_idle_update(state_machine_t *fsm){
    printf("Do not use the idle state!\n");
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){
    imu_correction();

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD("PursueState", "Ball is not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength <= PURSUE_BALL_TOO_CLOSE){
        ESP_LOGD("PursueState", "Ball too close, switching to orbit");
        FSM_CHANGE_STATE(Orbit);
    }

    ESP_LOGV("PursueState", "Ball is visible, pursuing");
    // Quickly approach the ball
    robotState.outSpeed = 40;
    robotState.outDirection = robotState.inBallAngle;
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    imu_correction();

    // Check criteria:
    // Ball too far away, Ball too close (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD("OrbitState", "Ball not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength > ORBIT_BALL_TOO_FAR){
        ESP_LOGD("OrbitState", "Ball too far away, reverting");
        printf("Ignoring switch request\n");
        // FSM_REVERT;
    } else if (rs.inBallStrength < ORBIT_BALL_TOO_CLOSE 
                && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD("OrbitState", "Ball and angle in correct spot, switching to dribble");
        printf("Ignoring switch request\n");
        // FSM_CHANGE_STATE(Dribble);
    }

    int16_t tempAngle = rs.inBallAngle > 180 ? rs.inBallAngle - 360 : rs.inBallAngle;

    ESP_LOGV("OrbitState", "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(tempAngle)) * fminf(90, 0.5 * powf(E, 0.5 * (float)smallestAngleBetween(tempAngle, 0))));
    float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.3 * strengthFactor * powf(E, 2.5 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
    robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * (1.0 - (float)fabsf(angleAddition) / 90.0);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    imu_correction();

    // Check criteria:
    // Ball too far away, Ball not in front of us, Goal not visible, Ball not visible
    if (robotState.inBallStrength <= 0.0f){
        ESP_LOGD("DribbleState", "Ball not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (!robotState.inGoalVisible){
        ESP_LOGD("DribbleState", "Goal not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD("DribbleState", "Ball not in front, reverting");
        FSM_REVERT;
    } else if (rs.inBallStrength <= DRIBBLE_BALL_TOO_FAR){
        ESP_LOGD("DribbleState", "Ball too far away, reverting");
        FSM_REVERT;
    }

    // rush towards goal
    ESP_LOGV("DribbleState", "Rushing goal");
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inGoalAngle;
}

// done with this macro
#undef rs