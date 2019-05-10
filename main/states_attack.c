#include "states.h"
#include "simple_imu.h"
#include "utils.h"

robot_state_t robotState = {0};
SemaphoreHandle_t robotStateSem = NULL;

// TODO also just a note: can't every function in this file be static??? except for stateAttackXXX

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_attack_pursue_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
fsm_state_t stateAttackDribble = {&state_nothing_enter, &state_nothing_exit, &state_attack_dribble_update, "AttackDribble"};

static TimerHandle_t idleTimer = NULL;
static float accelProgress = 0;
static bool timerRunning = false;
#define IDLE_TIMER_CHECK do { if (robotState.inBallStrength > 0.0f) idle_timer_stop(); } while (0);

// shortcut lol
#define rs robotState

/** start the idle timer if its not already started and has been instantiated **/
static void idle_timer_start(){
    if (idleTimer != NULL && !timerRunning){
        ESP_LOGD("IdleTimer", "Started idle timer!");
        xTimerReset(idleTimer, pdMS_TO_TICKS(10));
        xTimerStart(idleTimer, pdMS_TO_TICKS(10));
        timerRunning = true;
    }
}

/** stops the idle timer if it has been instantiated **/
static void idle_timer_stop(){
    if (idleTimer != NULL){
        ESP_LOGD("IdleTimer", "Stopping idle timer");
        xTimerStop(idleTimer, pdMS_TO_TICKS(10));
        timerRunning = false;
    }
}

/** callback that goes off after idle timeout **/
static void idle_timer_callback(TimerHandle_t timer){
    static const char *TAG = "IdleTimerCallback";
    ESP_LOGD(TAG, "Idle timer has gone off, switching to idle state");

    // world-class intellectual hack: we need to get access to the state machine instance from this callback.
    // as it turns out, the timer ID is passed as a void pointer (meaning it can be any type, though in this context
    // it should probably be an integer) - so we pass the state_machine_t as the timer's ID
    state_machine_t *fsm = (state_machine_t*) pvTimerGetTimerID(timer);
    FSM_CHANGE_STATE(Idle);
}

/** instantiates the idle timer if it null **/
static void idle_timer_create_if_needed(state_machine_t *fsm){
    if (idleTimer == NULL){
        ESP_LOGD("CreateIdleTimer", "Creating idle timer");
        idleTimer = xTimerCreate("IdleTimer", pdMS_TO_TICKS(IDLE_TIMEOUT), false, (void*) fsm, idle_timer_callback);
    }
}

// Idle
void state_attack_idle_update(state_machine_t *fsm){
    static const char *TAG = "AttackIdleState";

    rs.outIsAttack = true;
    printf("Do not use the idle state! It's not implemented yet.\n");
}

// Pursue
void state_attack_pursue_enter(state_machine_t *fsm){
    // we start out in this state, so create the idle timer if needs be
    idle_timer_create_if_needed(fsm);
}

void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    accelProgress = 0; // reset acceleration Progress
    rs.outIsAttack = true;
    imu_correction(&robotState);

    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball is not visible, braking");
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength >= ORBIT_DIST){
        ESP_LOGD(TAG, "Ball close enough, switching to orbit, strength: %d, orbit dist thresh: %d", rs.inBallStrength,
        ORBIT_DIST);
        FSM_CHANGE_STATE(Orbit);
    }

    // ESP_LOGD(TAG, "Ball is visible, pursuing");
    // Quickly approach the ball
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inBallAngle;
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    static const char *TAG = "OrbitState";
    accelProgress = 0; // reset acceleration Progress
    rs.outIsAttack = true;
    goal_correction(&robotState);

    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball too far away, Ball too close and angle good (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallStrength);
        imu_correction(&robotState);
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength < ORBIT_DIST){
        ESP_LOGD(TAG, "Ball too far away, reverting, strength: %d, orbit dist thresh: %d", robotState.inBallStrength,
                 ORBIT_DIST);
        // FSM_REVERT;
        FSM_CHANGE_STATE(Pursue);
    } else if (rs.inBallStrength >= ORBIT_DIST && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE) 
                && is_angle_between(rs.inGoalAngle, 30, 330)){
        ESP_LOGD(TAG, "Ball and angle in correct spot, switching to dribble, strength: %d, angle: %d, orbit dist thresh: %d"
        " angle range: %d-%d", robotState.inBallStrength, robotState.inBallAngle, ORBIT_DIST, IN_FRONT_MIN_ANGLE,
        IN_FRONT_MAX_ANGLE);
        FSM_CHANGE_STATE(Dribble);
    }

    // orbit requires angles in -180 to +180 range
    int16_t tempAngle = rs.inBallAngle > 180 ? rs.inBallAngle - 360 : rs.inBallAngle;

    // ESP_LOGD(TAG, "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(tempAngle)) * fminf(90, 
                                0.1 * powf(E, 0.1 * (float)smallestAngleBetween(tempAngle, 0))));
    float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / 
                            ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.1 * strengthFactor * powf(E, 2 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
    robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * 
                            (1.0 - (float)fabsf(angleAddition) / 90.0);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    goal_correction(&robotState);
    rs.outIsAttack = true;

    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball too far away, Ball not in front of us, Goal not visible, Ball not visible
    if (robotState.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallAngle);
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD(TAG, "Ball not in front, reverting, angle: %d, range: %d-%d", robotState.inBallAngle,
                IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE);
        // FSM_REVERT;
        FSM_CHANGE_STATE(Orbit);
    } else if (rs.inBallStrength <= DRIBBLE_BALL_TOO_FAR){
        ESP_LOGD(TAG, "Ball too far away, reverting, strength: %d, thresh: %d", robotState.inBallStrength,
        DRIBBLE_BALL_TOO_FAR);
        // FSM_REVERT;
        FSM_CHANGE_STATE(Orbit);
    } else if (rs.inGoalAngle > 30 || rs.inGoalAngle < 330){
        ESP_LOGD(TAG, "Not facing goal, reverting");
        // FSM_REVERT;
        FSM_CHANGE_STATE(Orbit);
    }

    // linear acceleration to give robot time to goal correct and so it doesn't slip
    // ESP_LOGD(TAG, "Rushing goal");
    // TODO delta time
    robotState.outSpeed = lerp(ORBIT_SPEED_FAST, DRIBBLE_SPEED, accelProgress);
    robotState.outDirection = robotState.inBallAngle; //robotState.inGoalAngle; // (no goal correction so just use ball)

    accelProgress += ACCEL_PROG;
}

// done with this macro
#undef rs