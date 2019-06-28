#include "states.h"
#include "simple_imu.h"
#include "utils.h"

// static pid_config_t lrfPID = {LRF_KP, LRF_KI, LRF_KD, LRF_MAX};
static pid_config_t interceptPID = {INTERCEPT_KP, INTERCEPT_KI, INTERCEPT_KD, INTERCEPT_MAX, 0};

fsm_state_t stateDefenceReverse = {&state_nothing_enter, &state_nothing_exit, &state_defence_reverse_update, "DefenceReverse"};
fsm_state_t stateDefenceIdle = {&state_nothing_enter, &state_nothing_exit, &state_defence_idle_update, "DefenceIdle"};
fsm_state_t stateDefenceDefend = {&state_defence_defend_enter, &state_nothing_exit, &state_defence_defend_update, "DefenceDefend"};
fsm_state_t stateDefenceSurge = {&state_nothing_enter, &state_nothing_exit, &state_defence_surge_update, "DefenceSurge"};

float accelProgress = 0.0f;

// shortcut lol
#define rs robotState

// Reverse
void state_defence_reverse_update(state_machine_t *fsm){
    static const char *TAG = "DefendReverseState";

    accelProgress = 0;
    rs.outIsAttack = false;
    imu_correction(&robotState);

    if (rs.inGoalVisible){
        LOG_ONCE(TAG, "Goal is visible, switching to idle");
        FSM_CHANGE_STATE_DEFENCE(Idle);
    }

    if (rs.inBallStrength > 0.0f){ 
        // Check if ball is behind
        if (!is_angle_between(rs.inBallAngle, 90.0f, 270.0f)){
            LOG_ONCE(TAG, "Ball is behind, orbiting");
            orbit(&robotState);
        } else {
            // TODO is ball in front? should we request switch?
            float distanceMovement = REVERSE_SPEED;
            float sidewaysMovement = pid_update(&sidePID, fmodf(rs.inBallAngle + 180.0f, 360.0f) - 180, 0.0f, 0.0f);

            rs.outDirection = fmodf(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)) - rs.inHeading, 360.0f);
            rs.outSpeed = get_magnitude(sidewaysMovement, distanceMovement);
        }
    } else {
        rs.outDirection = 180;
        rs.outSpeed = REVERSE_SPEED;
    }
}

// Idle
void state_defence_idle_update(state_machine_t *fsm){
    static const char *TAG = "DefendIdleState";

    accelProgress = 0;

    rs.outIsAttack = false;
    imu_correction(&robotState);

    if (!rs.inGoalVisible){
        // LOG_ONCE(TAG, "Goal not visible, switching to reverse");
        LOG_ONCE(TAG, "Cancelling state change to reverse"); // TODO change when we get LRFs
        // FSM_CHANGE_STATE_DEFENCE(Reverse);
    } else if (rs.inBallStrength >= DEFEND_MIN_STRENGTH){
        LOG_ONCE(TAG, "Ball is close enough, switching to defend");
        FSM_CHANGE_STATE_DEFENCE(Defend);
    }

    position(&robotState, DEFEND_DISTANCE, 0.0f, rs.inGoalAngle, rs.inGoalLength, true);
}

// Defend
void state_defence_defend_enter(state_machine_t *fsm){
    // no longer used
 }

 void state_defence_defend_update(state_machine_t *fsm){
    static const char *TAG = "DefendDefendState";

    accelProgress = 0;
    rs.outIsAttack = false;

    // if (is_angle_between(rs.inBallAngle, DEFEND_MIN_ANGLE, DEFEND_MAX_ANGLE)) goal_correction(&robotState);
    // else imu_correction(&robotState); // Face the back of the robot to the goal
    goal_correction(&robotState);

    // Check criteria: goal visible and ball visible, should surge?
    if (!rs.inGoalVisible){
        LOG_ONCE(TAG, "Goal not visible, NOT switching to reverse"); // NOTE: should reverse using LRFs but we dono't have those yet
        // FSM_CHANGE_STATE_DEFENCE(Reverse);
    } else if (rs.inBallStrength <= 0.01f){
        LOG_ONCE(TAG, "Ball too far away, switching to Idle");
        FSM_CHANGE_STATE_DEFENCE(Idle);
    } else if (is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE + 45, IN_FRONT_MAX_ANGLE - 45) && rs.inBallStrength >= SURGE_STRENGTH){
        LOG_ONCE(TAG, "Ball in caputre zone and strenght ok, switching to surge, angle: %f, strength: %f",
                robotState.inBallAngle, robotState.inBallStrength);
        FSM_CHANGE_STATE_DEFENCE(Surge);
    }

    if (is_angle_between(rs.inBallAngle, DEFEND_MIN_ANGLE, DEFEND_MAX_ANGLE)){
        // Ball is behind, orbit so we don't score an own goal
        orbit(&robotState);
    } else {
        float tempAngle = robotState.inBallAngle > 180 ? robotState.inBallAngle - 360 : robotState.inBallAngle; // Convert to -180 -> 180 range
        float distanceMovement = pid_update(&forwardPID, rs.inGoalLength, DEFEND_DISTANCE, 0.0f); // Stay a fixed distance from the goal
        
        // if(fabsf(sidewaysDistance) > (GOAL_WIDTH / 2) && sign(tempAngle) != sign(b)){
        //     // printf("At edge of goal\n");
        //     position(&robotState, DEFEND_DISTANCE - 5, sign(sidewaysDistance) * (GOAL_WIDTH / 2), rs.inGoalAngle, rs.inGoalLength, true);
        // } else {
            float sidewaysMovement = -pid_update(&interceptPID, tempAngle, 0.0f, 0.0f); // Position robot between ball and centre of goal (dunno if this works)
            if(fabsf(sidewaysMovement) < INTERCEPT_MIN) sidewaysMovement = 0;

            rs.outDirection = fmodf(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)), 360.0f);
            rs.outSpeed = get_magnitude(sidewaysMovement, distanceMovement);
            // printf("goalAngle_: %f, verticleDistance: %f, distanceMovement: %f, sidewaysMovement: %f\n", goalAngle_, verticalDistance, distanceMovement, sidewaysMovement);
        // }
        // printf("%f\n", sign(sidewaysDistance) * 35);
        // printf("sidewaysDistance: %f, tempAngle: %f, goalAngle_: %f\n", sidewaysDistance, tempAngle, goalAngle_);
    }
}

// Surge
void state_defence_surge_update(state_machine_t *fsm){
    static const char *TAG = "DefendSurgeState";
    imu_correction(&robotState);

    accelProgress = 0;

    RS_SEM_LOCK
    rs.outSwitchOk = true;
    rs.outIsAttack = false;
    RS_SEM_UNLOCK

    if (rs.inGoalLength >= SURGE_DISTANCE || !rs.inGoalVisible){
        LOG_ONCE(TAG, "Too far from goal, switching to defend, goal dist: %d", rs.inGoalLength);
        FSM_CHANGE_STATE_DEFENCE(Defend);
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE + 65, IN_FRONT_MAX_ANGLE - 65) || rs.inBallStrength < SURGE_STRENGTH - 30){
        LOG_ONCE(TAG, "Ball is not in capture zone, switching to defend");
        FSM_CHANGE_STATE_DEFENCE(Defend);
    } else if (!rs.inBTConnection && canShoot && robotState.inBallStrength >= KICKER_STRENGTH){
        LOG_ONCE(TAG, "Shoot conditions are good, shooting, ball strength: %f", robotState.inBallStrength);
        FSM_CHANGE_STATE_GENERAL(Shoot);
    }

    // EPIC YEET MODE
    // Linear acceleration to give robot time to goal correct and so it doesn't slip
    robotState.outSpeed = lerp(50.0f, DRIBBLE_SPEED, accelProgress); 
    // Just yeet towards the ball (which is forwards)
    // robotState.outDirection = robotState.inGoalVisible ? robotState.inGoalAngle : robotState.inBallAngle * 1.05;
    robotState.outDirection = robotState.inBallAngle;

    // Update progress for linear interpolation
    accelProgress += 0.0001;
}