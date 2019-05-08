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

    printf("Do not use this state! It is not implemented!!!\n");
}

// Idle
void state_defence_idle_update(state_machine_t *fsm){
    static const char *TAG = "DefendIdleState";

    rs.outSpeed = 0;
    rs.outIsAttack = false;

    imu_correction(&robotState);

    if(!rs.inGoalVisible){
        ESP_LOGD(TAG, "Goal not visible, braking"); // NOTE: should reverse using LRFs but we dono't have those yet
    }

    move_to_xy(&robotState, IDLE_OFFSET, IDLE_DISTANCE);
}

 // Defend
 void state_defence_defend_update(state_machine_t *fsm){
    static const char *TAG = "DefendDefendState"; // Lmao this is stupid

    rs.outIsAttack = false;
 }