#include "states.h"

robot_state_t robotState = {0};
SemaphoreHandle_t robotStateSem = NULL;

pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};
pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_nothing_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
fsm_state_t stateAttackDribble = {&state_nothing_enter, &state_nothing_exit, &state_attack_dribble_update, "AttackDribble"};

// shortcut lol
#define rs robotState

static inline void goal_correct_attack(){
    // TODO PID?
    robotState.outOrientation = robotState.inGoalAngle;
}

// Idle
void state_attack_idle_update(state_machine_t *fsm){
    // Check criteria:
    // Ball visible (switch to pursue), goal not visible (can't centre correctly so brake)
    if (robotState.inBallStrength > 0){
        ESP_LOGD("CentreState", "Changing to pursue: ball found (angle %d)", robotState.inBallAngle);
        FSM_CHANGE_STATE(Pursue);
    } else if (!robotState.inGoalVisible){
        ESP_LOGD("CentreState", "Goal not visible, braking");
        FSM_MOTOR_BRAKE;
    }

    ESP_LOGD("CentreState", "Ball visible, going to centre");
    // fixed goal angle
    float g = robotState.inGoalAngle < 0 ? robotState.inGoalAngle + 360 : robotState.inGoalAngle;
    g = floatMod(g + robotState.inHeading, 360.0f);

    float vDist = robotState.inGoalLength * cosf(DEG_RAD * g);
    float hDist = robotState.inGoalLength * sinf(DEG_RAD * g);
    
    float distanceMovement = -pid_update(&forwardPID, vDist, IDLE_DISTANCE, 0.0f);
    float sidewaysMovement = -pid_update(&sidePID, hDist, IDLE_OFFSET, 0.0f);

    robotState.outDirection = mod(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)) - (robotState.inHeading), 360);
    robotState.outSpeed = sqrtf(distanceMovement * distanceMovement + sidewaysMovement * sidewaysMovement);
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){
    goal_correct_attack();

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGI("PursueState", "Ball is not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength <= PURSUE_BALL_TOO_CLOSE){
        ESP_LOGD("PursueState", "Ball too close, switching to orbit");
        FSM_CHANGE_STATE(Orbit);
    }

    ESP_LOGD("PursueState", "Ball is visible, pursuing");
    // Quickly approach the ball
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inBallAngle;
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    // TODO do we want to goal correct?

    // Check criteria:
    // Ball too far away, Ball too close (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD("OrbitState", "Ball not visible, braking");
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength > ORBIT_BALL_TOO_FAR){
        ESP_LOGD("OrbitState", "Ball too far away, reverting");
        FSM_REVERT;
    } else if (rs.inBallStrength < ORBIT_BALL_TOO_CLOSE 
                && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD("OrbitState", "Ball and angle in correct spot, switching to dribble");
        FSM_CHANGE_STATE(Dribble);
    }

    ESP_LOGD("OrbitState", "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(robotState.inBallAngle)) * fminf(90, 0.4 * powf(E, 0.5 * (float)smallestAngleBetween(robotState.inBallAngle, 0))));
    float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.1 * strengthFactor * powf(E, 2.5 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
    robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * (1.0 - (float)fabsf(angleAddition) / 90.0);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    goal_correct_attack();

    // Check criteria:
    // Ball too far away, Ball not in front of us, Goal not visible, Ball not visible
    if (robotState.inBallStrength <= 0.0f){
        ESP_LOGI("DribbleState", "Ball not visible, braking");
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
    ESP_LOGD("DribbleState", "Rushing goal");
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inGoalAngle;
}

// done with this macro
#undef rs