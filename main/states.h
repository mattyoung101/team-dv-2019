#pragma once
#include "fsm.h"
#include <math.h>
#include "pid.h"
#include "defines.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

// Struct which holds highly processed info about the robot's state. Shared resource, should be synced with a mutex.
typedef struct {
    // Goal Inputs
    bool inGoalVisible;
    int16_t inGoalAngle;
    int16_t inGoalLength;
    int16_t inGoalDistance;
    int16_t inX;
    int16_t inY;
    // IMU Input
    float inHeading;
    // Ball Inputs
    int16_t inBallAngle;
    int16_t inBallStrength;
    // Line Inputs
    float inLineAngle;
    float inLineSize;
    bool inOnLine;
    bool inLineOver;
    float inlastAngle;

    // Outputs
    int16_t outSpeed;
    int16_t outDirection;
    int16_t outOrientation;
    bool outShouldBrake;
    bool outIsAttack;
} robot_state_t;

extern SemaphoreHandle_t robotStateSem;
extern robot_state_t robotState;

// Generic do nothing states (used for example if no "enter" method is needed on a state)
void state_nothing_enter(state_machine_t *fsm);
void state_nothing_exit(state_machine_t *fsm);
void state_nothing_update(state_machine_t *fsm);

/////////// ATTACK FSM /////////
// Centre state: moves the robot to the centre of the field. Only update.
void state_attack_idle_enter(state_machine_t *fsm);
void state_attack_idle_update(state_machine_t *fsm);
extern fsm_state_t stateAttackIdle;

// Pursue ball state: moves directly towards the ball
void state_attack_pursue_enter(state_machine_t *fsm);
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
// Reverse state: reverse to the back wall when goal is not visible
void state_defence_reverse_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceReverse;

// Idle state: centres on own goal
void state_defence_idle_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceIdle;

// Defend state: positions between ball and centre of goal
void state_defence_defend_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceDefend;

// Surge state: push ball away from goal
void state_defence_surge_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceSurge;

/////////// GENERAL FSM /////////
extern fsm_state_t stateGeneralNothing;