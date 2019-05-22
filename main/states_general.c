#include "states.h"

fsm_state_t stateGeneralNothing = {&state_nothing_enter, &state_nothing_exit, &state_nothing_update, "StateNothing"};

// shortcut cos i hate typing
#define rs robotState

void state_nothing_enter(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Entered");
}
void state_nothing_exit(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Exited");
}
void state_nothing_update(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Update");
}