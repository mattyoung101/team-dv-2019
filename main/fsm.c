#include "fsm.h"

// This file defines a very simple finite state machine (FSM) using function pointers

void fsm_update(state_machine_t *fsm){
    fsm->currentState->update(fsm);
}

void fsm_change_state(state_machine_t *fsm, fsm_state_t *newState){
    ESP_LOGD("FSM", "Switching states from %s to %s", fsm->currentState->name, newState->name);

    fsm->previousState = fsm->currentState;
    fsm->currentState->exit(fsm);
    
    fsm->currentState = newState;
    fsm->currentState->enter(fsm);
}

void fsm_revert_state(state_machine_t *fsm){
    ESP_LOGD("FSM", "Reverting to state from %s to %s", fsm->currentState->name, fsm->previousState->name);
    fsm_change_state(fsm, fsm->previousState);
}

bool fsm_in_state(state_machine_t *fsm, char *name){
    return strcmp(fsm->currentState->name, name) == 0;
}

char *fsm_get_current_state_name(state_machine_t *fsm){
    return fsm->currentState->name;
}