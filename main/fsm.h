#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"

// Based on/inspired by:
// https://github.com/libgdx/gdx-ai/blob/master/gdx-ai/src/com/badlogic/gdx/ai/fsm/DefaultStateMachine.java

typedef struct fsm_state_t fsm_state_t;
typedef struct state_machine state_machine;

// void stateFunction(state_machine *fsm);
typedef void (*fsm_func)(state_machine*);

struct fsm_state_t {
    fsm_func enter;
    fsm_func exit;
    fsm_func update;
    char *name;
};

struct state_machine {
    fsm_state_t *currentState;
    fsm_state_t *previousState;
};

void fsm_update(state_machine *fsm);
void fsm_change_state(state_machine *fsm, fsm_state_t *newState);
void fsm_revert_state(state_machine *fsm);
bool fsm_in_state(state_machine *fsm, char *name);
char *fsm_get_current_state_name(state_machine *fsm);