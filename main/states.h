#pragma once
#include "fsm.h"
#include <math.h>
#include "pid.h"
#include "utils.h"
#include "defines.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Struct which holds highly processed info about the robot's state. Shared resource, should be synced with a mutex.
typedef struct {
    // inputs
    bool inGoalVisible;
    int16_t inGoalAngle;
    float inHeading;
    int16_t inGoalLength;
    int16_t inBallAngle;
    int16_t inBallStrength;
    
    // outputs
    int16_t outSpeed;
    int16_t outDirection;
    int16_t outOrientation;
    bool outShouldBrake;
} robot_state_t;

extern SemaphoreHandle_t robotStateSem;
extern robot_state_t robotState;

// PIDs
extern pid_config_t forwardPID;
extern pid_config_t sidePID;
extern pid_config_t headingPID;
extern pid_config_t goalPID;

// Generic do nothing states (used for example if no "enter" method is needed on a state)
void state_nothing_enter(state_machine_t *fsm);
void state_nothing_exit(state_machine_t *fsm);
void state_nothing_update(state_machine_t *fsm);

/////////// ATTACK FSM /////////
// Centre state: moves the robot to the centre of the field. Only update.
void state_attack_idle_update(state_machine_t *fsm);
extern fsm_state_t stateAttackIdle;

// Pursue ball state: moves directly towards the ball
void state_attack_pursue_update(state_machine_t *fsm);
extern fsm_state_t stateAttackPursue;

// Orbit ball state: orbits the ball
void state_attack_orbit_enter(state_machine_t *fsm);
void state_attack_orbit_update(state_machine_t *fsm);
extern fsm_state_t stateAttackOrbit;

// Dribble state: rushes towards goal and dribbles if we had a dribbler
void state_attack_dribble_update(state_machine_t *fsm);
extern fsm_state_t stateAttackDribble;

// Shoot state: NOTE this is not currently used since we don't have a kicker
void state_attack_shoot_update(state_machine_t *fsm);
extern fsm_state_t stateAttackShoot;

/////////// DEFENCE FSM /////////
void state_defence_reverse_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceReverse;

/////////// GENERAL FSM /////////