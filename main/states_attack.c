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
        LOG_ONCE("IdleTimer", "Started idle timer!");
        xTimerReset(idleTimer, pdMS_TO_TICKS(10));
        xTimerStart(idleTimer, pdMS_TO_TICKS(10));
        timerRunning = true;
    }
}

/** stops the idle timer if it has been instantiated **/
static void idle_timer_stop(){
    if (idleTimer != NULL){
        LOG_ONCE("IdleTimer", "Stopping idle timer");
        xTimerStop(idleTimer, pdMS_TO_TICKS(10));
        timerRunning = false;
    }
}

/** callback that goes off after idle timeout **/
static void idle_timer_callback(TimerHandle_t timer){
    static const char *TAG = "IdleTimerCallback";
    ESP_LOGI(TAG, "Idle timer has gone off, switching to idle state");

    // world-class intellectual hack: we need to get access to the state machine instance from this callback.
    // as it turns out, the timer ID is passed as a void pointer (meaning it can be any type, though in this context
    // it should probably be an integer) - so we pass the state_machine_t as the timer's ID
    state_machine_t *fsm = (state_machine_t*) pvTimerGetTimerID(timer);
    FSM_CHANGE_STATE(Idle);
}

/** instantiates the idle timer if it is null **/
static void idle_timer_create_if_needed(state_machine_t *fsm){
    if (idleTimer == NULL){
        ESP_LOGI("CreateIdleTimer", "Creating idle timer");
        // this is the hack mentioned above in practice: we pass *fsm as a void pointer
        idleTimer = xTimerCreate("IdleTimer", pdMS_TO_TICKS(IDLE_TIMEOUT), false, (void*) fsm, idle_timer_callback);
    }
}

// Idle
void state_attack_idle_update(state_machine_t *fsm){
    static const char *TAG = "AttackIdleState";

    imu_correction(&robotState);
    rs.outIsAttack = true;

    if (rs.inBallStrength > 0.0f) {
        LOG_ONCE(TAG, "Ball is visible, reverting");
        FSM_REVERT;
    } else if (!rs.inGoalVisible) {
        LOG_ONCE(TAG, "Goal not visible, braking");
        FSM_MOTOR_BRAKE;
    }

    position(&robotState, IDLE_DISTANCE, IDLE_OFFSET);
}

// Pursue
void state_attack_pursue_enter(state_machine_t *fsm){
    // since we start out in pursue, create the idle timer if needs be
    idle_timer_create_if_needed(fsm);
}

void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    
    accelProgress = 0;
    rs.outIsAttack = true;
    imu_correction(&robotState);
    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball is not visible, braking");
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength >= ORBIT_DIST){
        LOG_ONCE(TAG, "Ball close enough, switching to orbit, strength: %d, orbit dist thresh: %d", rs.inBallStrength,
        ORBIT_DIST);
        FSM_CHANGE_STATE(Orbit);
    }

    // LOG_ONCE(TAG, "Ball is visible, pursuing");
    // Quickly approach the ball
    robotState.outSpeed = 100;
    robotState.outDirection = robotState.inBallAngle;
}

// Orbit
void state_attack_orbit_update(state_machine_t *fsm){
    static const char *TAG = "OrbitState";

    accelProgress = 0; // reset acceleration progress
    rs.outIsAttack = true;
    goal_correction(&robotState);
    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball too far away, Ball too close and angle good (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball not visible, braking, strength: %d", robotState.inBallStrength);
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength < ORBIT_DIST){
        LOG_ONCE(TAG, "Ball too far away, reverting, strength: %d, orbit dist thresh: %d", robotState.inBallStrength,
                 ORBIT_DIST);
        FSM_REVERT;
    } else if (rs.inBallStrength >= ORBIT_DIST && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE) 
                /*&& is_angle_between(rs.inGoalAngle, GOAL_MIN_ANGLE, GOAL_MAX_ANGLE)*/){
        LOG_ONCE(TAG, "Ball and angle in correct spot, switching to dribble, strength: %d, angle: %d, orbit dist thresh: %d"
                " angle range: %d-%d, goal angle: %d, goal angle range: %d-%d", 
                robotState.inBallStrength, robotState.inBallAngle, ORBIT_DIST, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE,
                robotState.inGoalAngle, GOAL_MIN_ANGLE, GOAL_MAX_ANGLE);
        // FSM_CHANGE_STATE(Dribble);
    }

    orbit(&robotState);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    
    rs.outIsAttack = true;
    goal_correction(&robotState);
    IDLE_TIMER_CHECK;

    // Check criteria:
    // Ball not visible, ball not in front, ball too far away, not facing goal
    if (robotState.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball not visible, braking, strength: %d", robotState.inBallAngle);
        idle_timer_start();
        FSM_MOTOR_BRAKE;
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        LOG_ONCE(TAG, "Ball not in front, reverting, angle: %d, range: %d-%d", robotState.inBallAngle,
                IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE);
        FSM_REVERT;
    } else if (rs.inBallStrength <= DRIBBLE_BALL_TOO_FAR){
        LOG_ONCE(TAG, "Ball too far away, reverting, strength: %d, thresh: %d", robotState.inBallStrength,
                DRIBBLE_BALL_TOO_FAR);
        FSM_REVERT;
    } else if (rs.inGoalAngle > GOAL_MIN_ANGLE || rs.inGoalAngle < GOAL_MAX_ANGLE){
        // LOG_ONCE(TAG, "Not facing goal, reverting, goal angle: %d, range: %d-%d", rs.inGoalAngle, GOAL_MIN_ANGLE, GOAL_MAX_ANGLE);
        // FSM_REVERT;
    }

    // Linear acceleration to give robot time to goal correct and so it doesn't slip
    robotState.outSpeed = lerp(ORBIT_SPEED_FAST, DRIBBLE_SPEED, accelProgress); 
    // Just yeet towards the ball (which is forwards)
    robotState.outDirection = robotState.inBallAngle;

    // Update progress for linear interpolation
    accelProgress += ACCEL_PROG;
}

// done with this macro
#undef rs