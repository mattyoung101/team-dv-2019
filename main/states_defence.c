#include "states.h"

// static pid_config_t lrfPID = {LRF_KP, LRF_KI, LRF_KD, LRF_MAX};

fsm_state_t stateDefenceReverse = {&state_nothing_enter, &state_nothing_exit, &state_defence_reverse_update, "DefenceReverse"};