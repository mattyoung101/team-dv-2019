#include "states.h"

robot_state_t robotState = {0};
extern SemaphoreHandle_t robotStateSem = NULL;

static pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};
static pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_nothing_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
// NOTE: need to do one for Dribble
fsm_state_t stateAttackShoot= {&state_nothing_enter, &state_nothing_exit, &state_attack_shoot_update, "AttackShoot"};

// Centre
void state_attack_idle_update(state_machine_t *fsm){
    // if ball is visible, pursue it
    if (robotState.inBallStrength > 0){
        ESP_LOGD("CentreState", "Changing to pursue: ball found (angle %d)", robotState.inBallAngle);
        fsm_change_state(fsm, &stateAttackPursue);
        return;
    }

    if (robotState.inGoalVisible){
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
    } else {
        ESP_LOGD("CentreState", "Goal not visible, braking");
        // can't see goal, brake
        robotState.outSpeed = 0;
        robotState.outDirection = 0;
    }
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){
    // Quickly approach the ball
    if (robotState.inBallAngle == TSOP_NO_BALL_ANGLE){
        ESP_LOGI("Pursue state", "Ball is not visible, braking");
        // Can't see ball, brake
        robotState.outSpeed = 0;
        robotState.outDirection = 0;
    } else {
        ESP_LOGI("Pursue state", "Ball is visible, pursuing");
        // Quickly approach the ball
        robotState.outSpeed = 100;
        robotState.outDirection = robotState.inBallAngle;
    }
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    // Simple orbit based on ball angle and strength
    if (robotState.inBallAngle == TSOP_NO_BALL_ANGLE){
        ESP_LOGI("Orbit state", "Ball not visible, braking");
        // Can't see ball, brake
        robotState.outSpeed = 0;
        robotState.outDirection = 0;
    } else {
        ESP_LOGI("Orbit state", "Ball is visible, orbiting");

        float ballAngleDifference = ((-(robotState.inBallAngle - 180) > 0) - (-(robotState.inBallAngle - 180) < 0)) * fminf(90, 0.4 * powf(E, 0.5 * (float)smallestAngleBetween(robotState.inBallAngle, 0)));
        float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
        float distanceMultiplier = constrain(0.1 * strengthFactor * powf(E, 2.5 * strengthFactor), 0, 1);
        float angleAddition = ballAngleDifference * distanceMultiplier;

        robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
        robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * (1.0 - (float)abs(angleAddition) / 90.0);
    }
}

// Dribble

// NOTE: we will probs call the orbit update but make it goal correct here

// Shoot
void state_attack_shoot_update(state_machine_t *fsm){
    // Literally run forwards at max speed (and kick if we got a kicker)
    ESP_LOGI("Shoot state", "Shooting for goal");
    robotState.outSpeed = 100;
}