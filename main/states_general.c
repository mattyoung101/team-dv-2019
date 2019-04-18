#include "states.h"

void state_nothing_enter(state_machine_t *fsm){
    ESP_LOGD("StateNothing", "Entered");
}
void state_nothing_exit(state_machine_t *fsm){
    ESP_LOGD("StateNothing", "Exited");
}
void state_nothing_update(state_machine_t *fsm){
    ESP_LOGD("StateNothing", "Update");
}