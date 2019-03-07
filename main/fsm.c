#include "fsm.h"

void fsm_update(state_machine *fsm){
    fsm->currentState->update(fsm);
}

void fsm_change_state(state_machine *fsm, fsm_state *newState){
    fsm->previousState = fsm->currentState;
    fsm->currentState->exit(fsm);
    
    fsm->currentState = newState;
    fsm->currentState->enter(fsm);
}

void fsm_revert_state(state_machine *fsm){
    fsm_change_state(fsm, fsm->previousState);
}

bool fsm_in_state(state_machine *fsm, char name[]){
    return strcmp(fsm->currentState->name, name) == 0;
}