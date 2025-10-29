#include "state-machine.h"
#include "limestone-interface.h"
#include <stdlib.h>
#include <stdint.h>

const int MPV_DELAY = 0.5 * 1000; // in milliseconds
const int BURN_DURATION = 22 * 1000; // in milliseconds

uint32_t handoff_timestamp;
uint32_t ignition_timestamp;

void set_handoffTS(){
    ignition_timestamp = getTime();
}

void set_ignitionTS(){
    handoff_timestamp = getTime();
}

uint32_t time_since_handoff() {
    return getTime() - handoff_timestamp;
}

uint32_t time_since_igntion() {
    return getTime() - ignition_timestamp;
}

void state0_transition(){
    if (time_since_handoff() >= MPV_DELAY){
        set_ignitionTS();
        return true;
    }

    return false;
}

void state1_transition(){
    return time_since_igntion() >= BURN_DURATION;
}

void state0_output();

void state1_output();

void update_states();


State* create_states(){
    State* s0;
    state_setName(s0, "Open MPV1");
    state_setNumTransitions(s0);
    state_addTransition(s0, &state0_transition);


    State* s1;
    state_setName(s1, "Open both MPVs");
    state_setNumTransitions(s1);
    state_addTransition(s1, &state1_transition);

    return s0; // return first state;
}


int main(){
    State* current_state = create_states();
    set_handoffTS();

    update_states();
}