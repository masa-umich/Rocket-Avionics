#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#define MAX_TRANSITIONS 4

typedef struct State State;
typedef struct StateTransition Transition;

struct StateTransition {
    bool (*inputs_satisfied)(void);
    State* nextState;
};

struct State {
    char* name;
    Transition transitions[MAX_TRANSITIONS];
    int num_transitions;

    void (*perform_output)(void);
};  

void state_setName(State* s, char* n){
    s->name = (char*) malloc(strlen(n) + 1);
    strcpy(s->name, n);
}

void state_setNumTransitions(State* s){
    s->num_transitions = 0;
}

void state_addTransition(State* s, bool (*function_ptr)(void)){
    s->transitions[s->num_transitions].inputs_satisfied = function_ptr;
    s->num_transitions++;
}

#endif
