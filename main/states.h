#pragma once
#include "fsm.h"
#include <math.h>
#include "pid.h"
#include "utils.h"
#include "defines.h"
#include "esp_log.h"

// Struct which holds highly processed info about the robot's state. Shared resource, should be synced with a mutex.
typedef struct {
    // inputs
    bool goalVisible;
    int16_t goalAngle;
    float heading;
    int16_t goalLength;
    int16_t ballAngle;
    
    // outputs
    int16_t speed;
    int16_t direction;
} robot_state_t;

robot_state_t robotState;

// Generic do nothing states (used for example if no "enter" method is needed on a state)
void state_nothing_enter(state_machine *fsm){}
void state_nothing_exit(state_machine *fsm){}
void state_nothing_update(state_machine *fsm){}

// Centre: moves the robot to the centre of the field. Only update.
void state_centre_update(state_machine *fsm);
extern fsm_state_t centreState;

// Pursue ball: basically an orbit
void state_pursue_enter(state_machine *fsm);
void state_pursue_update(state_machine *fsm);
extern fsm_state_t pursueState;

/*
States are:
- Move to centre (if we can't see ball)
- Pursue ball (in other words, orbit)
- Dribble ball (move ball towards goal)
- Defend (do we need sub-states here? not currently as defence is pretty simple)
- Avoid line (lineover avoidance)
- Shoot (kicker)

The ones used by Bend It Like Beckham:
- out of bounds
- looking for ball
- see ball
- have ball
*/