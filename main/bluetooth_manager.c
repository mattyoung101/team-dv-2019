#include "comms_bluetooth.h"

// tasks which runs when a Bluetooth connection is established. manages sending and receiving data as well as logic.

static dv_timer_t packetTimer = {NULL, false};
static dv_timer_t cooldownTimer = {NULL, false};
static bool cooldownOn = false; // true if the cooldown timer is currently activated

static void packet_timer_callback(TimerHandle_t timer){
    static const char *TAG = "BTTimeout";

    ESP_LOGW(TAG, "Packet timeout has gone off, other robot is off for damage");
    uint32_t handle = (uint32_t) pvTimerGetTimerID(timer);

    if (robotState.outIsAttack) fsm_change_state(stateMachine, &stateDefenceDefend);
    // suspend the two logic tasks to prevent Bluetooth errors (they get confused since no connection currently exists)
    vTaskSuspend(receiveTaskHandle);
    vTaskSuspend(sendTaskHandle);
    
    esp_spp_disconnect(handle);
    dv_timer_stop(&packetTimer);
}

static void cooldown_timer_callback(TimerHandle_t timer){
    static const char *TAG = "CooldownTimer";
    ESP_LOGI(TAG, "Cooldown timer gone off, re-enabling switch");
    cooldownOn = false;
    dv_timer_stop(&cooldownTimer);
}

void comms_bt_receive_task(void *pvParameter){
    static const char *TAG = "BTReceive";
    uint32_t handle = (uint32_t) pvParameter;
    bool wasSwitchOk = false;
    bool alreadyPrinted = false;
    uint8_t switchBuffer[] = {'S', 'W', 'I', 'T', 'C', 'H'};

    // create timers and semaphore if they've not already been created in a previous run of this task
    dv_timer_check_create(&packetTimer, "BTTimeout", BT_PACKET_TIMEOUT, pvParameter, packet_timer_callback);
    dv_timer_check_create(&cooldownTimer, "CooldownTimer", BT_SWITCH_COOLDOWN, pvParameter, cooldown_timer_callback);

    ESP_LOGI(TAG, "Bluetooth receive task init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);

    while (true){
        BTProvide recvMsg = BTProvide_init_zero;
        bool isAttack = false;
        bool isInShootState = false;

        // read in a new packet from the packet queue, otherwise block this thread till one's available
        if (xQueueReceive(packetQueue, &recvMsg, portMAX_DELAY)){
            isAttack = strstr(recvMsg.fsmState, "Attack");
            isInShootState = strcmp(recvMsg.fsmState, "GeneralShoot") == 0;
            xTimerReset(packetTimer.timer, portMAX_DELAY);

            // ESP_LOGD(TAG, "Ball angle: %f, Ball strength: %f", recvMsg.ballAngle, recvMsg.ballStrength);
        }

        // required due to cross-core access (multi-threading crap)
        bool amIAttack = false;
        RS_SEM_LOCK
        amIAttack = robotState.outIsAttack;
        RS_SEM_UNLOCK

        // detect conflicts and resolve with whichever algorithm was selected
        if (((isAttack && amIAttack) || (!isAttack && !amIAttack)) && !isInShootState) {
            ESP_LOGW(TAG, "Conflict detected: I'm %s, other is %s", robotState.outIsAttack ? "ATTACK" : "DEFENCE", 
                    isAttack ? "ATTACK" : "DEFENCE");
            ESP_LOGD(TAG, "my ball distance: %f, other ball distance: %f", robotState.inBallStrength, recvMsg.ballStrength);
            
            #if BT_CONF_RES_MODE == BT_CONF_RES_DYNAMIC
                ESP_LOGI(TAG, "Dynamic conflict resolution algorithm running");

                // conflict resolution: whichever robot is closest to the ball becomes the attacker + some extra edge cases
                // if in shoot state, ignore conflict as both robots can be shooting without conflict
                if (robotState.inBallStrength <= 0.1f && recvMsg.ballStrength <= 0.1f){
                    ESP_LOGI(TAG, "Conflict resolution: both robots can't see ball, using default state");

                    if (ROBOT_MODE == MODE_ATTACK){
                        fsm_change_state(stateMachine, &stateAttackPursue);
                    } else {
                        fsm_change_state(stateMachine, &stateDefenceDefend);
                    }
                } else if (robotState.inBallStrength <= 0.1f){
                    ESP_LOGI(TAG, "Conflict resolution: I cannot see ball, becoming defender");
                    fsm_change_state(stateMachine, &stateDefenceDefend);
                } else if (recvMsg.ballStrength <= 0.1f){
                    ESP_LOGI(TAG, "Conflict resolution: other robot cannot see ball, becoming attacker");
                    fsm_change_state(stateMachine, &stateAttackPursue);
                } else {
                    // both robots can see the ball
                    if (recvMsg.ballStrength < robotState.inBallStrength){
                        ESP_LOGI(TAG, "Conflict resolution: other robot is closest to ball, switch to defence");
                        fsm_change_state(stateMachine, &stateDefenceDefend);
                    } else {
                        ESP_LOGI(TAG, "Conflict resolution: I'm closest to ball, switch to attack");
                        fsm_change_state(stateMachine, &stateAttackPursue);
                    }
                }
            #elif BT_CONF_RES_MODE == BT_CONF_RES_STATIC
                ESP_LOGI(TAG, "Static conflict resolution algorithm running...");

                // change into which ever mode was set in NVS
                if (ROBOT_MODE == MODE_ATTACK){
                    fsm_change_state(stateMachine, &stateAttackPursue);
                } else {
                    fsm_change_state(stateMachine, &stateDefenceDefend);
                }
            #endif
        }

        // decide if we should switch or not
        if (recvMsg.switchOk){
            if (!alreadyPrinted){
                ESP_LOGI(TAG, "Other robot is willing to switch");
                alreadyPrinted = true;
            }
            wasSwitchOk = true;

            // if we're OK to switch and we're not in the cooldown period and we're the switch listener, switch.
            // only one robot (robot 0) will be able to broadcast switch statements to save them both from
            // switching at the same time
            if (robotState.outSwitchOk && !cooldownOn && robotState.inRobotId == 0){
                ESP_LOGI(TAG, "========== I'm also willing to switch: switching NOW! ==========");
                esp_spp_write(handle, 6, switchBuffer);
                
                // invert state
                if (robotState.outIsAttack){
                    fsm_change_state(stateMachine, &stateDefenceDefend); 
                } else {
                    fsm_change_state(stateMachine, &stateAttackPursue);
                }

                // start cooldown timer
                cooldownOn = true;
                xTimerStart(cooldownTimer.timer, portMAX_DELAY);
                alreadyPrinted = false;
            } else {
                // TODO remove this log spam
                ESP_LOGD(TAG, "Unable to switch: am I willing to switch? %s, cooldown timer on? %s, robotId: %d",
                robotState.outSwitchOk ? "yes" : "no", cooldownOn ? "yes" : "no", robotState.inRobotId);
            }
        } else if (wasSwitchOk){
            // if the other robot is not willing to switch, but was previously willing to switch
            ESP_LOGW(TAG, "Other robot is NO LONGER willing to switch");
            wasSwitchOk = false;
            alreadyPrinted = false;
        }

        esp_task_wdt_reset();
    }
}

void comms_bt_send_task(void *pvParameter){
    static const char *TAG = "BTSend";
    uint32_t handle = (uint32_t) pvParameter;
    uint8_t buf[PROTOBUF_SIZE] = {0};

    ESP_LOGI(TAG, "Bluetooth send task init OK, handle: %d", handle);
    esp_task_wdt_add(NULL);
    
    while (true){
        memset(buf, 0, PROTOBUF_SIZE);
        BTProvide sendMsg = BTProvide_init_zero;

        RS_SEM_LOCK;
        sendMsg.onLine = robotState.inOnLine;
        
        char *stateName = fsm_get_current_state_name(stateMachine); // thread safe get name function, uses strdup
        strcpy(sendMsg.fsmState, stateName); // put into struct
        free(stateName); // since we use strdup, we gotta free it
        stateName = NULL; // good practice

        sendMsg.robotX = robotState.inX;
        sendMsg.robotY = robotState.inY;
        #ifdef BT_SWITCHING_ENABLED
            sendMsg.switchOk = robotState.outSwitchOk;
        #else
            sendMsg.switchOk = false;
        #endif
        sendMsg.goalLength = robotState.inGoalLength;
        sendMsg.ballAngle = robotState.inBallAngle;
        sendMsg.ballStrength = robotState.inBallStrength;
        RS_SEM_UNLOCK;

        pb_ostream_t stream = pb_ostream_from_buffer(buf, PROTOBUF_SIZE);
        if (pb_encode(&stream, BTProvide_fields, &sendMsg)){
            esp_spp_write(handle, stream.bytes_written, buf);
        } else {
            ESP_LOGE(TAG, "Error encoding Protobuf stream: %s", PB_GET_ERROR(&stream));
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50)); // let other tasks think for a while I guess, BT isn't that important
    }
}