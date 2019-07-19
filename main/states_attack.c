#include "states.h"
#include "simple_imu.h"
#include "utils.h"

robot_state_t robotState = {0};
SemaphoreHandle_t robotStateSem = NULL;
bool canShoot = true;

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_attack_pursue_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
fsm_state_t stateAttackDribble = {&state_nothing_enter, &state_nothing_exit, &state_attack_dribble_update, "AttackDribble"};
fsm_state_t stateAttackDoubleDefence = {&state_nothing_enter, &state_nothing_exit, &state_attack_doubledefence_update, "AttackDoubleDefence"};

static dv_timer_t idleTimer = {NULL, false};
static float accelProgress = 0.0f;
static float accelBegin = 0.0f;

// shortcut lol
#define rs robotState

/** callback that goes off after idle timeout **/
static void idle_timer_callback(TimerHandle_t timer){
    static const char *TAG = "IdleTimerCallback";
    ESP_LOGI(TAG, "Idle timer has gone off, switching to idle state");

    // world-class intellectual hack: we need to get access to the state machine instance from this callback.
    // as it turns out, the timer ID is passed as a void pointer (meaning it can be any type, though in this context
    // it should probably be an integer) - so we pass the state_machine_t as the timer's ID
    state_machine_t *fsm = (state_machine_t*) pvTimerGetTimerID(timer);
    dv_timer_stop(&idleTimer);
    FSM_CHANGE_STATE(Idle);
}

static void create_timers_if_needed(state_machine_t *fsm){
    dv_timer_check_create(&idleTimer, "IdleTimer", IDLE_TIMEOUT, (void*) fsm, idle_timer_callback);
}

/** checks if any of the timers should be disabled based on current robot data */
static void timer_check(){
    // if the ball is visible, stop the idle timer
    if (robotState.inBallStrength > 0.0f){
        dv_timer_stop(&idleTimer);
    }
}


////////// BEGIN STATE MACHINE CODE //////////
// Idle
void state_attack_idle_update(state_machine_t *fsm){
    static const char *TAG = "AttackIdleState";

    imu_correction(&robotState);
    rs.outIsAttack = true;
    
    RS_SEM_LOCK
    rs.outIsAttack = true;
    rs.outSwitchOk = true;
    RS_SEM_UNLOCK

    // rs.outSpeed = 0.0f;

    // Check criteria: ball must not be visible, goal must be visible (this is the root state so don't revert)
    if (!orangeBall.exists) {
        LOG_ONCE(TAG, "Ball is visible, reverting");
        FSM_REVERT;
    } else if (!rs.inOtherGoalVisible) {
        LOG_ONCE(TAG, "Goal not visible, braking");
        FSM_MOTOR_BRAKE;
    }

    // position(&robotState, IDLE_DISTANCE, IDLE_OFFSET, rs.inOtherGoalAngle, rs.inOtherGoalLength, true);

    // float verticalDistance = fabsf(robotState.inGoalLength /** cosf(DEG_RAD * goalAngle_)*/);
    // float distanceMovement = -pid_update(&forwardPID, verticalDistance, HALFWAY_DISTANCE, 0.0f); // Stay a fixed distance from the goal
    
    // rs.outDirection = fmodf(RAD_DEG * (atan2f(0.0f, distanceMovement)), 360.0f);
    rs.outSpeed = 0.0f;
}

// Pursue
void state_attack_pursue_enter(state_machine_t *fsm){
    create_timers_if_needed(fsm);
}

void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    
    accelProgress = 0;
    RS_SEM_LOCK
    rs.outIsAttack = true;
    rs.outSwitchOk = true;
    RS_SEM_UNLOCK
    imu_correction(&robotState);
    timer_check();

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (!orangeBall.exists){
        LOG_ONCE(TAG, "Ball is not visible, braking");
        dv_timer_start(&idleTimer);
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
    RS_SEM_LOCK
    rs.outIsAttack = true;
    rs.outSwitchOk = true;
    RS_SEM_UNLOCK
    // if(is_angle_between(rs.inBallAngle, 60, 300)) goal_correction(&robotState);
    // else imu_correction(&robotState);
    goal_correction(&robotState);
    timer_check();

    if (rs.inBallStrength >= DRIBBLE_BALL_TOO_FAR && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        LOG_ONCE(TAG, "Ball and angle in correct spot, changing intro dribble, strength: %f, angle: %f, orbit dist thresh: %d"
                " angle range: %d-%d", robotState.inBallStrength, robotState.inBallAngle, ORBIT_DIST, IN_FRONT_MIN_ANGLE, 
                IN_FRONT_MAX_ANGLE);
        accelBegin = rs.outSpeed;
        FSM_CHANGE_STATE(Dribble);
    }

    // Check criteria:
    // Ball too far away, Ball too close and angle good (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        LOG_ONCE(TAG, "Ball not visible, switching to idle, strength: %f", robotState.inBallStrength);
        dv_timer_start(&idleTimer);
        FSM_CHANGE_STATE(Idle);
    } else if (rs.inBallStrength < ORBIT_DIST){
        LOG_ONCE(TAG, "Ball too far away, reverting, strength: %f, orbit dist thresh: %d", robotState.inBallStrength,
                 ORBIT_DIST);
        FSM_REVERT;
    } 

    orbit(&robotState);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    
    RS_SEM_LOCK
    rs.outIsAttack = true;
    rs.outSwitchOk = false; // we're trying to shoot so piss off
    RS_SEM_UNLOCK
    goal_correction(&robotState);
    timer_check();

    // printfln("Strength: %f", rs.inBallStrength);

    // Check criteria:
    // Ball not visible, ball not in front, ball too far away, not facing goal, should we kick?
    if (rs.inBallStrength >= 180.0f && canShoot){
        LOG_ONCE(TAG, "Ball close enough and shoot permitted, shooting, strength: %f", rs.inBallStrength);
        FSM_CHANGE_STATE_GENERAL(Shoot);
    } else if (!orangeBall.exists){
        LOG_ONCE(TAG, "Ball not visible, braking, strength: %f", robotState.inBallStrength);
        dv_timer_start(&idleTimer);
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallAngle > IN_FRONT_MIN_ANGLE + IN_FRONT_ANGLE_BUFFER && rs.inBallAngle < IN_FRONT_MAX_ANGLE - IN_FRONT_ANGLE_BUFFER){
        LOG_ONCE(TAG, "Ball not in front, reverting, angle: %f, range: %d-%d", robotState.inBallAngle,
                IN_FRONT_MIN_ANGLE + IN_FRONT_ANGLE_BUFFER, IN_FRONT_MAX_ANGLE - IN_FRONT_ANGLE_BUFFER);
        FSM_REVERT;
    }

    // ESP_LOGI(TAG, "Not shooting, goal angle: %d, goal length: %d, can shoot: %s", robotState.inGoalAngle,
    // robotState.inGoalLength, canShoot ? "yes" : "no");
    
    // } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE + IN_FRONT_ANGLE_BUFFER, IN_FRONT_MAX_ANGLE - IN_FRONT_ANGLE_BUFFER)){
    //     LOG_ONCE(TAG, "Ball not in front, reverting, angle: %f, range: %d-%d", robotState.inBallAngle,
    //             IN_FRONT_MIN_ANGLE + IN_FRONT_ANGLE_BUFFER, IN_FRONT_MAX_ANGLE - IN_FRONT_ANGLE_BUFFER);
    //     FSM_REVERT;
    // }
    // } else if (is_angle_between(rs.inGoalAngle, IN_FRONT_MIN_ANGLE + IN_FRONT_ANGLE_BUFFER, 
    //             IN_FRONT_MAX_ANGLE - IN_FRONT_ANGLE_BUFFER) && robotState.inGoalLength <= GOAL_SHOOT_DIST && canShoot){
    //     LOG_ONCE(TAG, "Ball in front, goal close and shoot permitted, shooting, angle: %d, goal length: %d", rs.inGoalAngle,
    //     rs.inGoalLength);
    //     FSM_CHANGE_STATE(Shoot);
    // }

    // Linear acceleration to give robot time to goal correct and so it doesn't slip
    robotState.outSpeed = lerp(accelBegin, DRIBBLE_SPEED, accelProgress); 
    // Just yeet towards the ball (which is forwards)
    // robotState.outDirection = robotState.inGoalVisible ? robotState.inGoalAngle : robotState.inBallAngle;
    robotState.outDirection = rs.inBallAngle;

    // Update progress for linear interpolation
    accelProgress += ACCEL_PROG;
}

// Avoid Double Defence
// NOTE: DOES NOT SWITCH INTO THIS STATE YET
void state_attack_doubledefence_update(state_machine_t *fsm){
    static const char *TAG = "AvoidDoubleDefenceState";

    RS_SEM_LOCK
    rs.outIsAttack = true;
    rs.outSwitchOk = false; // we switch now, and we might enter double defence, so don't
    RS_SEM_UNLOCK
    imu_correction(&robotState);
    timer_check();

    if(rs.inOtherGoalLength < GOAL_TOO_CLOSE){
        float goalAngle = robotState.inOtherGoalAngle < 0.0f ? robotState.inOtherGoalAngle + 360.0f : robotState.inOtherGoalAngle; // Convert to 0 - 360 range
        float goalAngle_ = fmodf(goalAngle + robotState.inHeading, 360.0f);
        float verticalDistance = fabsf(robotState.inOtherGoalLength * cosf(DEG_RAD * goalAngle_));
        float distanceMovement = pid_update(&forwardPID, verticalDistance, GOAL_TOO_CLOSE + 10, 0.0f); // Stay a fixed distance from the goal
    }
}

// done with this macro
#undef rs