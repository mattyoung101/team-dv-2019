#define DG_DYNARR_IMPLEMENTATION
#include "fsm.h"
#include "states.h"
#include "utils.h"

// This file defines a very simple finite state machine (FSM) using function pointers

static const char *TAG = "FSM";

state_machine_t* fsm_new(fsm_state_t *startState){
    // TODO move this elsewhere - this code should just handle FSMs, not other crap as well
    if (robotStateSem == NULL){
        robotStateSem = xSemaphoreCreateMutex();
        xSemaphoreGive(robotStateSem);
    }

    state_machine_t* fsm = calloc(1, sizeof(state_machine_t));
    fsm->currentState = &stateGeneralNothing;
    fsm->semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(fsm->semaphore);
    // change into the start state, to make sure startState->enter is called
    fsm_change_state(fsm, startState); 
    return fsm;
}

void fsm_update(state_machine_t *fsm){
    fsm->currentState->update(fsm);
}

static void fsm_internal_change_state(state_machine_t *fsm, fsm_state_t *newState, bool pushToStack){
    if (xSemaphoreTake(fsm->semaphore, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
        if (pushToStack) da_push(fsm->stateHistory, fsm->currentState);
        fsm->currentState->exit(fsm);
        
        fsm->currentState = newState;
        fsm->currentState->enter(fsm);

        log_once_reset();
        xSemaphoreGive(fsm->semaphore);
    } else {
        ESP_LOGE(TAG, "Failed to unlock FSM semaphore, cannot change states.");
    }
}

void fsm_change_state(state_machine_t *fsm, fsm_state_t *newState){
    if (fsm->currentState == newState) return;

    ESP_LOGI(TAG, "Switching states from %s to %s", fsm->currentState->name, newState->name);    
    fsm_internal_change_state(fsm, newState, true);
}

void fsm_revert_state(state_machine_t *fsm){
    // first check if it's safe to pop
    if (da_count(fsm->stateHistory) < 1){
        ESP_LOGW(TAG, "Unable to revert: state history too small, size = %d", da_count(fsm->stateHistory));
        return;
    }

    fsm_state_t *previousState = da_pop(fsm->stateHistory);
    ESP_LOGI(TAG, "Reverting to state %s from %s", previousState->name, fsm->currentState->name);
    fsm_internal_change_state(fsm, previousState, false);
}

bool fsm_in_state(state_machine_t *fsm, char *name){
    return strcmp(fsm_get_current_state_name(fsm), name) == 0;
}

char *fsm_get_current_state_name(state_machine_t *fsm){
    if (xSemaphoreTake(fsm->semaphore, pdMS_TO_TICKS(SEMAPHORE_UNLOCK_TIMEOUT))){
        char *state = strdup(fsm->currentState->name);
        xSemaphoreGive(fsm->semaphore);
        return state;
    } else {
        ESP_LOGE(TAG, "Failed to unlock FSM semaphore, cannot return state name");
        return "ERROR";
    }
}