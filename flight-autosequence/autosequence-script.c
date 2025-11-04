#include "state-machine.h"
#include "limestone-interface.h"
#include <stdlib.h>
#include <stdint.h>


#define STATE0_NAME "Open MPV1"
#define STATE1_NAME "Open both MPVs"

#define STARTUP_MESSAGE "Opening Ox MPV"
#define STATE01_TRANSITION_MESSAGE "Opening Ox MPV"
//#define STATE01_TRANSITION_MESSAGE "Opening Ox MPV"   

typedef enum {
    STATE0 = 0,
    STATE1,
    STATE2,
    STATE3,
    STATE4,
    STATE5
} States;

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

bool state0_transition(){
    if (time_since_handoff() >= MPV_DELAY){
        set_ignitionTS();
        return true;
    }

    return false;
}

bool state1_transition(){
    return time_since_igntion() >= BURN_DURATION;
}

void state0_output(){
    energizeMPV1();
}

void state1_output(){
    energizeMPV1();
    energizeMPV2();
}

/*
State* create_states(){
    State* s0;
    state_setName(s0, STATE0_NAME);
    state_setNumTransitions(s0);
    state_addTransition(s0, &state0_transition);
    state_setOutput(s0, &state0_output);


    State* s1;
    state_setName(s1, STATE1_NAME);
    state_setNumTransitions(s1);
    state_addTransition(s1, &state1_transition);
    state_setOutput(s1, &state1_output);

    return s0; // return first state;
}
*/

void execute_flight_autosequence(){
    uint8_t current_state = STATE0; // starts at state0
    log_message(STARTUP_MESSAGE);

    for (;;){
        switch (current_state) {
            case STATE0:
                state0_output();

                if (state0_transition()){
                    current_state = STATE1;
                    log_message(STATE01_TRANSITION_MESSAGE);
                } 
                break;
            
            case STATE1:
                state1_output();

                if (state1_transition()){
                    current_state = STATE2;
                    log_message();
                }
                break;

            
        }
    }
}