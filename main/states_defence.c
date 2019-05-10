#include "states.h"
#include "simple_imu.h"
#include "utils.h"

// static pid_config_t lrfPID = {LRF_KP, LRF_KI, LRF_KD, LRF_MAX};

fsm_state_t stateDefenceReverse = {&state_nothing_enter, &state_nothing_exit, &state_defence_reverse_update, "DefenceReverse"};
fsm_state_t stateDefenceIdle = {&state_nothing_enter, &state_nothing_exit, &state_defence_idle_update, "DefenceIdle"};
fsm_state_t stateDefenceDefend = {&state_nothing_enter, &state_nothing_exit, &state_defence_defend_update, "DefenceDefend"};
fsm_state_t stateDefenceSurge = {&state_nothing_enter, &state_nothing_exit, &state_defence_surge_update, "DefenceSurge"};

// shortcut lol
#define rs robotState

// Reverse
void state_defence_reverse_update(state_machine_t *fsm){
    static const char *TAG = "DefendReverseState";

    rs.outIsAttack = false;

    imu_correction(&robotState);

    if (rs.inGoalVisible){
        ESP_LOGD(TAG, "Goal is visible, switching to idle");
        FSM_CHANGE_STATE_DEFENCE(Idle);
    }

    if (rs.inBallStrength > 0.0f){ 
        if (!is_angle_between(rs.inBallAngle, 90.0f, 270.0f)){
            orbit(&robotState);
        } else {
            float distanceMovement = REVERSE_SPEED;
            float sidewaysMovement = pid_update(&sidePID, fmodf(rs.inBallAngle + 180, 360) - 180, 0.0f, 0.0f);

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

    rs.outIsAttack = false;

    imu_correction(&robotState);

    if (!rs.inGoalVisible){
        ESP_LOGD(TAG, "Goal not visible, switching to reverse"); // NOTE: should reverse using LRFs but we dono't have those yet
        FSM_CHANGE_STATE_DEFENCE(Reverse);
    } else if (rs.inBallStrength > 0.0f){
        ESP_LOGD(TAG, "Ball is visible, switching to defend");
        FSM_CHANGE_STATE_DEFENCE(Defend);
    }

    float goalAngle = rs.inGoalAngle < 0.0f ? rs.inGoalAngle + 360.0f : rs.inGoalAngle;
    float goalAngle_ = fmodf(goalAngle + rs.inHeading, 360.0f);

    float verticalDistance = rs.inGoalLength * cosf(DEG_RAD * goalAngle_);
    float horizontalDistance = rs.inGoalLength * sinf(DEG_RAD * goalAngle_);

    float distanceMovement = pid_update(&forwardPID, verticalDistance, DEFEND_DISTANCE, 0.0f);
    float sidewaysMovement = pid_update(&sidePID, horizontalDistance, 0.0f, 0.0f);

    rs.outDirection = fmodf(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)) - rs.inHeading, 360.0f);
    rs.outSpeed = get_magnitude(sidewaysMovement, distanceMovement);
}

 // Defend
 void state_defence_defend_update(state_machine_t *fsm){
    static const char *TAG = "DefendDefendState";

    rs.outIsAttack = false;

    goal_correction(&robotState);

    if (!rs.inGoalVisible){
        ESP_LOGD(TAG, "Goal not visible, switching to reverse"); // NOTE: should reverse using LRFs but we dono't have those yet
        FSM_CHANGE_STATE_DEFENCE(Reverse);
    } else if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, switching to idle");
        FSM_CHANGE_STATE_DEFENCE(Idle);
    } else if (is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE) && rs.inBallStrength >= SURGE_STRENGTH && rs.inGoalLength < SURGE_DISTANCE){
        ESP_LOGD(TAG, "Ball is in capture zone and goal is nearby, switching to surge");
        FSM_CHANGE_STATE_DEFENCE(Surge);
    }

    if (!is_angle_between(rs.inBallAngle, DEFEND_MIN_ANGLE, DEFEND_MAX_ANGLE)){
        orbit(&robotState);
    } else {
        float distanceMovement = pid_update(&forwardPID, rs.inGoalLength, DEFEND_DISTANCE, 0.0f);
        float sidewaysMovement = pid_update(&sidePID, fmodf(rs.inBallAngle + 180, 360) - 180, 0.0f, 0.0f);

        rs.outDirection = fmodf(RAD_DEG * (atan2f(sidewaysMovement, distanceMovement)) - rs.inHeading, 360.0f);
        rs.outSpeed = get_magnitude(sidewaysMovement, distanceMovement);
    }
}

// Surge
void state_defence_surge_update(state_machine_t *fsm){
    static const char *TAG = "DefendSurgeState";

    rs.outIsAttack = false;

    if (rs.inGoalLength > SURGE_DISTANCE || !rs.inGoalVisible){
        ESP_LOGD(TAG, "Too far from goal, switching to defend");
        FSM_CHANGE_STATE_DEFENCE(Defend);
    } else if (!is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE) || rs.inBallStrength < SURGE_STRENGTH){
        ESP_LOGD(TAG, "Ball is not in capture zone, switching to defend");
        FSM_CHANGE_STATE_DEFENCE(Defend);
    }

    rs.outDirection = rs.inBallAngle;
    rs.outSpeed = SURGE_SPEED;
}