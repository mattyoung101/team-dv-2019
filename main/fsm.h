#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"

// Based on/inspired by:
// https://github.com/libgdx/gdx-ai/blob/master/gdx-ai/src/com/badlogic/gdx/ai/fsm/DefaultStateMachine.java
// TODO add message passing like in libGDX

typedef struct fsm_state_t fsm_state_t;
typedef struct state_machine_t state_machine_t;

// void stateFunction(state_machine_t *fsm);
typedef void (*fsm_func)(state_machine_t*);

struct fsm_state_t {
    fsm_func enter;
    fsm_func exit;
    fsm_func update;
    char *name;
};

struct state_machine_t {
    fsm_state_t *currentState;
    fsm_state_t *previousState;
};

void fsm_update(state_machine_t *fsm);
void fsm_change_state(state_machine_t *fsm, fsm_state_t *newState);
void fsm_revert_state(state_machine_t *fsm);
bool fsm_in_state(state_machine_t *fsm, char *name);
char *fsm_get_current_state_name(state_machine_t *fsm);