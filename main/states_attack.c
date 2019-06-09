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

static dv_timer_t idleTimer = {NULL, false};
static dv_timer_t dribbleTimer = {NULL, false};
static float accelProgress = 0;

// shortcut lol
#define rs robotState

//////// UTILS CODE //////////
/** start a timer if its not already started and has been instantiated **/
static void timer_start(dv_timer_t *timer){
    if (timer->timer != NULL && !timer->running){
        xTimerReset(timer->timer, pdMS_TO_TICKS(10));
        xTimerStart(timer->timer, pdMS_TO_TICKS(10));
        timer->running = false;
    }
}

/** stops a timer if it has been instantiated **/
static void timer_stop(dv_timer_t *timer){
    if (timer->timer != NULL){
        xTimerStop(timer->timer, pdMS_TO_TICKS(10));
        timer->running = false;
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

static void dribble_timer_callback(TimerHandle_t timer){
    static const char *TAG = "DribbleTimerCallback";
    ESP_LOGI(TAG, "Dribble timer has gone off, switch to dribble state");

    state_machine_t *fsm = (state_machine_t*) pvTimerGetTimerID(timer);
    FSM_CHANGE_STATE(Dribble);
}

/** instantiates the idle timer if it is null **/
static void create_timers_if_needed(state_machine_t *fsm){
    static const char *TAG = "CreateTimer";

    if (idleTimer.timer == NULL){
        ESP_LOGI(TAG, "Creating idle timer");
        idleTimer.timer = xTimerCreate("IdleTimer", pdMS_TO_TICKS(IDLE_TIMEOUT), false, (void*) fsm, idle_timer_callback);
    } else if (dribbleTimer.timer == NULL){
        ESP_LOGI(TAG, "Creating dribble timer timer");
        dribbleTimer.timer = xTimerCreate("DribbleTimer", pdMS_TO_TICKS(DRIBBLE_TIMEOUT), false, (void*) fsm, 
                            dribble_timer_callback);
    }
}

/** checks if any of the timers should be disabled based on current robot data */
static void timer_check(){
    if (robotState.inBallStrength > 0.0f){
        timer_stop(&idleTimer);
    }
}


////////// BEGIN STATE MACHINE CODE //////////
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
    create_timers_if_needed(fsm);
}

void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    
    accelProgress = 0;
    rs.outIsAttack = true;
    imu_correction(&robotState);
    timer_check();

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball is not visible, braking");
        timer_start(&idleTimer);
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength >= ORBIT_DIST){
        LOG_ONCE(TAG, "Ball close enough, switching to orbit, strength: %f, orbit dist thresh: %d", rs.inBallStrength,
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
    if(is_angle_between(rs.inBallAngle, 90, 270)) goal_correction(&robotState);
    else imu_correction(&robotState);
    timer_check();

    // Check criteria:
    // Ball too far away, Ball too close and angle good (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball not visible, switching to idle, strength: %f", robotState.inBallStrength);
        timer_start(&idleTimer);
        timer_stop(&dribbleTimer);
        FSM_CHANGE_STATE(Idle);
    } else if (rs.inBallStrength < ORBIT_DIST){
        LOG_ONCE(TAG, "Ball too far away, reverting, strength: %f, orbit dist thresh: %d", robotState.inBallStrength,
                 ORBIT_DIST);
        timer_stop(&dribbleTimer);
        FSM_REVERT;
    } else if (rs.inBallStrength >= DRIBBLE_BALL_TOO_FAR && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        LOG_ONCE(TAG, "Ball and angle in correct spot, starting dribble timer, strength: %f, angle: %f, orbit dist thresh: %d"
                " angle range: %d-%d", robotState.inBallStrength, robotState.inBallAngle, ORBIT_DIST, IN_FRONT_MIN_ANGLE, 
                IN_FRONT_MAX_ANGLE);
        timer_start(&dribbleTimer);
    }

    orbit(&robotState);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    
    rs.outIsAttack = true;
    goal_correction(&robotState);
    timer_check();

    // Check criteria:
    // Ball not visible, ball not in front, ball too far away, not facing goal
    if (robotState.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball not visible, braking, strength: %f", robotState.inBallAngle);
        timer_start(&idleTimer);
        FSM_MOTOR_BRAKE;
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE + 20, IN_FRONT_MAX_ANGLE - 20)){
        LOG_ONCE(TAG, "Ball not in front, reverting, angle: %f, range: %d-%d", robotState.inBallAngle,
                IN_FRONT_MIN_ANGLE + 20, IN_FRONT_MAX_ANGLE - 20);
        FSM_REVERT;
    } else if (rs.inBallStrength <= DRIBBLE_BALL_TOO_FAR - 30){
        LOG_ONCE(TAG, "Ball too far away, reverting, strength: %f, thresh: %d", robotState.inBallStrength,
                DRIBBLE_BALL_TOO_FAR - 30);
        FSM_REVERT;
    }
    /*} else if (!is_angle_between(rs.inGoalAngle, GOAL_MIN_ANGLE, GOAL_MAX_ANGLE) || !rs.inGoalVisible){
        LOG_ONCE(TAG, "Not facing goal, reverting, goal angle: %d, range: %d-%d", rs.inGoalAngle, GOAL_MIN_ANGLE, GOAL_MAX_ANGLE);
        FSM_REVERT;
    }*/

    // Linear acceleration to give robot time to goal correct and so it doesn't slip
    robotState.outSpeed = lerp(ORBIT_SPEED_SLOW, DRIBBLE_SPEED, accelProgress); 
    // Just yeet towards the ball (which is forwards)
    robotState.outDirection = robotState.inGoalAngle;

    // Update progress for linear interpolation
    accelProgress += ACCEL_PROG;
}

// done with this macro
#undef rs