#include "states.h"
#include "utils.h"

fsm_state_t stateGeneralNothing = {&state_nothing_enter, &state_nothing_exit, &state_nothing_update, "GeneralNothing"};
fsm_state_t stateGeneralShoot = {&state_general_shoot_enter, &state_nothing_exit, &state_general_shoot_update, "GeneralShoot"};
static dv_timer_t shootTimer = {NULL, false};

// shortcut cos i hate typing
#define rs robotState

/** start a timer if its not already started and has been instantiated */
void dv_timer_start(dv_timer_t *timer){
    if (timer->timer != NULL && !timer->running){
        xTimerReset(timer->timer, pdMS_TO_TICKS(10));
        xTimerStart(timer->timer, pdMS_TO_TICKS(10));
        timer->running = true;
    }
}

/** stops a timer if it has been instantiated */
void dv_timer_stop(dv_timer_t *timer){
    if (timer->timer != NULL){
        xTimerStop(timer->timer, pdMS_TO_TICKS(10));
        timer->running = false;
    }
}

void dv_timer_check_create(dv_timer_t *timer, char *timerName, int32_t timeout, void *const parameter, 
                            TimerCallbackFunction_t callback){
    if (timer->timer == NULL && !timer->running){
        ESP_LOGI("CreateTimer", "Creating timer: %s", timerName);
        timer->timer = xTimerCreate(timerName, pdMS_TO_TICKS(timeout), false, parameter, callback);
    }
}

static void shoot_timer_callback(TimerHandle_t timer){
    static const char *TAG = "ShootTimerCallback";
    ESP_LOGW(TAG, "Shoot timer gone off, enabling shooting again");

    canShoot = true;
    dv_timer_stop(&shootTimer);
}

// Shoot
void state_general_shoot_enter(state_machine_t *fsm){
    static const char *TAG = "ShootState";
    if (!canShoot){
        ESP_LOGE(TAG, "Illegal state change: shoot not permitted at this time. Fix your code!");
        return;
    }
    
    canShoot = false;
    dv_timer_check_create(&shootTimer, "ShootTimer", SHOOT_TIMEOUT, NULL, shoot_timer_callback);
    dv_timer_start(&shootTimer);
    
    ESP_LOGW(TAG, "Activating kicker");
    gpio_set_level(KICKER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(KICKER_DELAY));
    gpio_set_level(KICKER_PIN, 0);
}

void state_general_shoot_update(state_machine_t *fsm){
    // we revert here as reverting in enter seems to cause problems
    FSM_REVERT;
}

// Nothing. Empty states for when no state function is declared.
void state_nothing_enter(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Entered");
}
void state_nothing_exit(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Exited");
}
void state_nothing_update(state_machine_t *fsm){
    ESP_LOGV("StateNothing", "Update");
}