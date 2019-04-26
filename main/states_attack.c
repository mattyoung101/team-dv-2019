#include "states.h"
#include "simple_imu.h"
#include "utils.h"

robot_state_t robotState = {0};
SemaphoreHandle_t robotStateSem = NULL;

fsm_state_t stateAttackIdle = {&state_nothing_enter, &state_nothing_exit, &state_attack_idle_update, "AttackIdle"};
fsm_state_t stateAttackPursue = {&state_nothing_enter, &state_nothing_exit, &state_attack_pursue_update, "AttackPursue"};
fsm_state_t stateAttackOrbit = {&state_nothing_enter, &state_nothing_exit, &state_attack_orbit_update, "AttackOrbit"};
fsm_state_t stateAttackDribble = {&state_nothing_enter, &state_nothing_exit, &state_attack_dribble_update, "AttackDribble"};

static TimerHandle_t idleTimer = NULL;
static bool timerRunning = false;

// shortcut lol
#define rs robotState
// start timer if not null and not started
#define START_TIMER_CHECKED do { if (idleTimer != null) xTimerStart(idleTimer); } while (0);
#define STOP_TIMER_CHECKED do { if (idleTimer != null) xTimerStop(idleTimer); } while (0);

// Idle
static void idle_timer_callback(TimerHandle_t timer){
    static const char *TAG = "IdleTimerCallback";
    ESP_LOGD(TAG, "Timer gone off");
}

void state_attack_idle_enter(state_machine_t *fsm){
    static const char *TAG = "IdleStateEnter";
    if (idleTimer == NULL){
        ESP_LOGD(TAG, "Creating idle timer");
        idleTimer = xTimerCreate("IdleTimer", pdMS_TO_TICKS(IDLE_TIME), false, (void*) 0, idle_timer_callback);
    }
}

void state_attack_idle_update(state_machine_t *fsm){
    printf("Do not use the idle state! It's not implemented yet.\n");
}

// Pursue
void state_attack_pursue_update(state_machine_t *fsm){
    static const char *TAG = "PursueState";
    imu_correction(&robotState);

    // Check criteria:
    // Ball not visible (brake) and ball too close (switch to orbit)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball is not visible, braking");
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
    imu_correction(&robotState);

    // Check criteria:
    // Ball too far away, Ball too close and angle good (go to dribble), Ball too far (revert)
    if (rs.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallStrength);
        FSM_MOTOR_BRAKE;
    } else if (rs.inBallStrength < ORBIT_DIST){
        ESP_LOGD(TAG, "Ball too far away, reverting, strength: %d, orbit dist thresh: %d", robotState.inBallStrength,
                 ORBIT_DIST);
        // FSM_REVERT;
        FSM_CHANGE_STATE(Pursue);
    } else if (rs.inBallStrength >= ORBIT_DIST && is_angle_between(rs.inBallAngle, IN_FRONT_MIN_ANGLE, IN_FRONT_MAX_ANGLE)){
        ESP_LOGD(TAG, "Ball and angle in correct spot, switching to dribble, strength: %d, angle: %d, orbit dist thresh: %d"
        " angle range: %d-%d", robotState.inBallStrength, robotState.inBallAngle, ORBIT_DIST, IN_FRONT_MIN_ANGLE,
        IN_FRONT_MAX_ANGLE);
        FSM_CHANGE_STATE(Dribble);
    }

    // orbit requires angles in -180 to +180 range
    int16_t tempAngle = rs.inBallAngle > 180 ? rs.inBallAngle - 360 : rs.inBallAngle;

    // ESP_LOGD(TAG, "Ball is visible, orbiting");
    float ballAngleDifference = ((sign(tempAngle)) * fminf(90, 
                                0.2 * powf(E, 0.2 * (float)smallestAngleBetween(tempAngle, 0))));
    float strengthFactor = constrain(((float)robotState.inBallStrength - (float)BALL_FAR_STRENGTH) / 
                            ((float)BALL_CLOSE_STRENGTH - BALL_FAR_STRENGTH), 0, 1);
    float distanceMultiplier = constrain(0.2 * strengthFactor * powf(E, 2.5 * strengthFactor), 0, 1);
    float angleAddition = ballAngleDifference * distanceMultiplier;

    robotState.outDirection = floatMod(robotState.inBallAngle + angleAddition, 360);
    robotState.outSpeed = ORBIT_SPEED_SLOW + (float)(ORBIT_SPEED_FAST - ORBIT_SPEED_SLOW) * 
                            (1.0 - (float)fabsf(angleAddition) / 90.0);
}

// Dribble
void state_attack_dribble_update(state_machine_t *fsm){
    static const char *TAG = "DribbleState";
    imu_correction(&robotState);

    // Check criteria:
    // Ball too far away, Ball not in front of us, Goal not visible, Ball not visible
    if (robotState.inBallStrength <= 0.0f){
        ESP_LOGD(TAG, "Ball not visible, braking, strength: %d", robotState.inBallAngle);
        FSM_MOTOR_BRAKE;
    // } else if (!robotState.inGoalVisible){
    //     ESP_LOGD(TAG, "Goal not visible, braking");
    //     // goal correction doesn't work rn so just ignore it
    //     // FSM_MOTOR_BRAKE;
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
    }

    // rush towards goal
    // ESP_LOGD(TAG, "Rushing goal");
    robotState.outSpeed = 60;
    robotState.outDirection = robotState.inBallAngle; //robotState.inGoalAngle; // (no goal correction so just use ball)
}

// done with this macro
#undef rs