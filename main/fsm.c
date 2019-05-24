#define DG_DYNARR_IMPLEMENTATION
#include "fsm.h"
#include "states.h"

// This file defines a very simple finite state machine (FSM) using function pointers

static const char *TAG = "FSM";

state_machine_t fsm_new(fsm_state_t *startState){
    // TODO this is pretty hacky, should we check if it's already created first? 
    // or move it elsewhere - this code should just handle FSMs, not other crap as well
    if (robotStateSem == NULL){
        robotStateSem = xSemaphoreCreateMutex();
        xSemaphoreGive(robotStateSem);
    }

    state_machine_t* fsm = calloc(1, sizeof(state_machine_t));
    fsm->currentState = &stateGeneralNothing;
    // change into the start state, to make sure startState->enter is called
    fsm_change_state(fsm, startState); 
    return *fsm;
}

void fsm_update(state_machine_t *fsm){
    fsm->currentState->update(fsm);
}

static void fsm_internal_change_state(state_machine_t *fsm, fsm_state_t *newState, bool pushToStack){
    // ESP_LOGD(TAG, "internal_change_state, changing to %s, push to stack? %d", newState->name, pushToStack);
    
    if (pushToStack) da_push(fsm->stateHistory, fsm->currentState);
    fsm->currentState->exit(fsm);
    
    fsm->currentState = newState;
    fsm->currentState->enter(fsm);
}

void fsm_change_state(state_machine_t *fsm, fsm_state_t *newState){
    ESP_LOGD(TAG, "Switching states from %s to %s", fsm->currentState->name, newState->name);
    fsm_internal_change_state(fsm, newState, true);
}

void fsm_revert_state(state_machine_t *fsm){
    // first check if it's safe to pop
    if (da_count(fsm->stateHistory) < 1){
        ESP_LOGW(TAG, "Unable to revert: state history too small, size = %d", da_count(fsm->stateHistory));
        return;
    }

    fsm_state_t *previousState = da_pop(fsm->stateHistory);
    ESP_LOGD(TAG, "Reverting to state %s from %s", previousState->name, fsm->currentState->name);
    fsm_internal_change_state(fsm, previousState, false);
}

bool fsm_in_state(state_machine_t *fsm, char *name){
    return strcmp(fsm->currentState->name, name) == 0;
}

char *fsm_get_current_state_name(state_machine_t *fsm){
    return fsm->currentState->name;
}