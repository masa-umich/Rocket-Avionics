#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>


typedef struct State State;
typedef struct StateTransition Transition;

struct StateTransition {
    bool (*inputs_satisfied)(void);
    State* nextState;
};

struct State {
    const char* name;
    Transition* transitions;
    int num_transitions;

    void (*perform_output)(void);
};  

#endif
