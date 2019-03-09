#pragma once
#include "fsm.h"

// Generic do nothing states (used for example if no "enter" method is needed on a state)
void state_nothing_enter(state_machine *fsm){}
void state_nothing_exit(state_machine *fsm){}
void state_nothing_update(state_machine *fsm){}

/*
States are:
- Move to centre
- Pursue ball
- Defend
- Avoid line
- Shoot? (only in open)

The ones used by Bend It Like Beckham:
- out of bounds
- looking for ball
- see ball
- have ball
*/