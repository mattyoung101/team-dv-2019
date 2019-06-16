#include "states.h"

fsm_state_t stateGeneralNothing = {&state_nothing_enter, &state_nothing_exit, &state_nothing_update, "StateNothing"};

// shortcut cos i hate typing
#define rs robotState

/** start a timer if its not already started and has been instantiated */
void dv_timer_start(dv_timer_t *timer){
    if (timer->timer != NULL && !timer->running){
        // printf("dv_timer_start name: %s\n", pcTimerGetTimerName(timer->timer));
        xTimerReset(timer->timer, pdMS_TO_TICKS(10));
        xTimerStart(timer->timer, pdMS_TO_TICKS(10));
        timer->running = true;
    }
}

/** stops a timer if it has been instantiated */
void dv_timer_stop(dv_timer_t *timer){
    if (timer->timer != NULL){
        // printf("dv_timer_stop name: %s\n", pcTimerGetTimerName(timer->timer));
        xTimerStop(timer->timer, pdMS_TO_TICKS(10));
        timer->running = false;
    }
}

void state_nothing_enter(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Entered");
}
void state_nothing_exit(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Exited");
}
void state_nothing_update(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Update");
}