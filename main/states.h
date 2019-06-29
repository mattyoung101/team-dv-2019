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
    // Main goal in question (home goal for defender, enemy goal for attacker)
    bool inGoalVisible;
    int16_t inGoalAngle;
    int16_t inGoalLength;
    int16_t inGoalDistance;
    // Other goal (home goal for attacker, enemy goal for defender)
    bool inOtherGoalVisible;
    int16_t inOtherGoalAngle;
    int16_t inOtherGoalLength;
    int16_t inOtherGoalDistance;
    // Coordinate System
    int16_t inX;
    int16_t inY;
    // IMU Input
    float inHeading;
    // Ball Inputs
    float inBallAngle;
    float inBallStrength;
    // Line Inputs
    float inLineAngle;
    float inLineSize;
    bool inOnLine;
    bool inLineOver;
    float inLastAngle;
    // Other stuff
    float inBatteryVoltage;
    uint8_t inRobotId;
    bool inBTConnection;

    // Outputs
    int16_t outSpeed;
    int16_t outDirection;
    int16_t outOrientation;
    bool outShouldBrake;
    bool outIsAttack;
    bool outSwitchOk;
    bool outLineBallWaiting;
} robot_state_t;

typedef struct {
    TimerHandle_t timer;
    bool running;
} dv_timer_t;

extern bool canShoot;

/** locks the robot state semaphore */
#define RS_SEM_LOCK if (xSemaphoreTake(robotStateSem, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))) {
/** unlocks the robot state semaphore */
#define RS_SEM_UNLOCK xSemaphoreGive(robotStateSem); } else { ESP_LOGW(TAG, "Failed to unlock robot state semaphore!"); }

/** start a timer if its not already started and has been instantiated */
void dv_timer_start(dv_timer_t *timer);
/** stops a timer if it has been instantiated */
void dv_timer_stop(dv_timer_t *timer);
/** 
 * checks to see if a timer needs to be created.
 * @param timeout timeout, in ms, automatically converted to ticks inside this function using pdMS_TO_TICKS
 */
void dv_timer_check_create(dv_timer_t *timer, char *timerName, int32_t timeout, void *const parameter, 
                            TimerCallbackFunction_t callback);

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

// Avoid double defence state: Does not move into the goalie box to avoid double defence
void state_attack_doubledefence_update(state_machine_t *fsm);
extern fsm_state_t stateAttackDoubleDefence;

/////////// DEFENCE FSM /////////
// Reverse state: reverse to the back wall when goal is not visible
void state_defence_reverse_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceReverse;

// Idle state: centres on own goal
void state_defence_idle_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceIdle;

// Defend state: positions between ball and centre of goal
void state_defence_defend_enter(state_machine_t *fsm);
void state_defence_defend_update(state_machine_t *fsm);
extern fsm_state_t stateDefenceDefend;

// Surge state: push ball away from goal
void state_defence_surge_update(state_machine_t *fsm);
void state_defence_surge_exit(state_machine_t *fsm);
extern fsm_state_t stateDefenceSurge;

/////////// GENERAL FSM /////////
extern fsm_state_t stateGeneralNothing;

// Shoot state: kicks the ball, then reverts to previous state
void state_general_shoot_enter(state_machine_t *fsm);
void state_general_shoot_update(state_machine_t *fsm);
extern fsm_state_t stateGeneralShoot;