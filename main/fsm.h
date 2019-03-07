#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// Based on/inspired by:
// https://github.com/libgdx/gdx-ai/blob/master/gdx-ai/src/com/badlogic/gdx/ai/fsm/DefaultStateMachine.java

typedef struct fsm_state fsm_state;
typedef struct state_machine state_machine;

// void stateFunction(state_machine *fsm);
typedef void (*fsm_func)(state_machine*);

struct fsm_state {
    fsm_func enter;
    fsm_func exit;
    fsm_func update;
    char name[];
};

struct state_machine {
    fsm_state *currentState;
    fsm_state *previousState;
};

// TODO enum of all state fsm_states

void fsm_update(state_machine *fsm);
void fsm_change_state(state_machine *fsm, fsm_state *newState);
void fsm_revert_state(state_machine *fsm);
bool fsm_in_state(state_machine *fsm, char name[]);