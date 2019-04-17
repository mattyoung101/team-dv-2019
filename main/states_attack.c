#include "states.h"

robot_state_t robotState = {0};
extern SemaphoreHandle_t robotStateSem = NULL;

static pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};
static pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_nothing_enter, &state_nothing_enter, &state_attack_pursue_update, "AttackPursue"};

// Centre
void state_attack_idle_update(state_machine_t *fsm){
    // if ball is visible, pursue it
    if (robotState.ballStrength > 0){
        ESP_LOGD("CentreState", "Changing to pursue: ball found (angle %d)", robotState.ballAngle);
        fsm_change_state(fsm, &stateAttackPursue);
        return;
    }

    if (robotState.goalVisible){
        ESP_LOGD("CentreState", "Ball visible, going to centre");
        // fixed goal angle
        float g = robotState.goalAngle < 0 ? robotState.goalAngle + 360 : robotState.goalAngle;
        g = floatMod(g + robotState.heading, 360.0f);

        float vDist = robotState.goalLength * cosf(DEG_RAD * g);
        float hDist = robotState.goalLength * sinf(DEG_RAD * g);
        
        float distanceMovement = -pid_update(&forwardPID, vDist, DEFEND_DISTANCE, 0.0f);
        float sidewaysMovement = -pid_update(&sidePID, hDist, 0.0f, 0.0f);

        robotState.direction = mod(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)) - (robotState.heading), 360);
        robotState.speed = sqrtf(distanceMovement * distanceMovement + sidewaysMovement * sidewaysMovement);
    } else {
        ESP_LOGD("CentreState", "Goal not visible, braking");
        // can't see goal, brake
        robotState.speed = 0;
        robotState.direction = 0;
    }
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){

}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    // // Simple orbit based on ball angle and strength

    // if (robotState.ballAngle == TSOP_NO_BALL_ANGLE){
    //     ESP_LOGI("OrbitState", "Ball not visible, braking");
    //     // Can't see ball, brake
    //     robotState.speed = 0;
    //     robotState.direction = 0;
    // } else {
    //     ESP_LOGI("OrbitState", "Ball is visible, orbiting");

    //     float ball_angle_difference = -sign(robotState.ballAngle - 180) * fminf(90, 0.4 * pow(E, 0.5 * (float)smallestAngleBetween(robotState.ballAngle, 0)));
        
    //     // FYI: Need to finish
    // }
}