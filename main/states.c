#include "states.h"

// This file holds all the states of the robot to be used in the finite state machine

static pid_config_t forwardPID = {FORWARD_KP, FORWARD_KI, FORWARD_KD, FORWARD_MAX};
static pid_config_t sidePID = {SIDE_KP, SIDE_KI, SIDE_KD, SIDE_MAX};

fsm_state_t centreState = {&state_nothing_enter, &state_nothing_exit, &state_centre_update, "Centre"};

// Centre
void state_centre_update(state_machine *fsm){
    // if ball is visible, pursue it
    if (robotState.ballAngle != TSOP_NO_BALL_ANGLE){
        ESP_LOGI("CentreState", "Changing to pursue: ball found (angle %d)", robotState.ballAngle);
        fsm_change_state(&fsm, &pursueState);
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
        ESP_LOGD("CentreState", "Ball not visible, braking");
        // can't see goal, brake
        robotState.speed = 0;
        robotState.direction = 0;
    }
}

// Pursue
void state_pursue_update(state_machine *fsm){

}