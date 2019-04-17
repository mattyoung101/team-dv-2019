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
    bool goalVisible;
    int16_t goalAngle;
    float heading;
    int16_t goalLength;
    int16_t ballAngle;
    int16_t ballStrength;
    
    // outputs
    int16_t speed;
    int16_t direction;
} robot_state_t;

extern SemaphoreHandle_t robotStateSem;
extern robot_state_t robotState;

// Generic do nothing states (used for example if no "enter" method is needed on a state)
void state_nothing_enter(state_machine_t *fsm){}
void state_nothing_exit(state_machine_t *fsm){}
void state_nothing_update(state_machine_t *fsm){}

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
extern fsm_state_t stateAttackObit;

// Dribble
void state_attack_dribble_enter(state_machine_t *fsm);
extern fsm_state_t stateShootEnter;

// Shoot
void state_attack_shoot_enter(state_machine_t *fsm);
extern fsm_state_t stateAttackShoot;

/////////// DEFENCE FSM /////////


/////////// GENERAL FSM /////////