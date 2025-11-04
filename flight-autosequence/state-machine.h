#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
/*
#define MAX_TRANSITIONS 4

typedef struct {
    bool (*inputs_satisfied)(void);
    State* nextState;
} Transition;

typedef struct {
    char* name;
    Transition transitions[MAX_TRANSITIONS];
    uint8_t num_transitions;

    void (*perform_output)(void);
} State;  

void state_setName(State* s, char* n){
    s->name = (char*) malloc(strlen(n) + 1);
    strcpy(s->name, n);
}

void state_setNumTransitions(State* s){
    s->num_transitions = 0;
}

void state_addTransition(State* s, bool (*transition_fcn)(void)){
    s->transitions[s->num_transitions].inputs_satisfied = transition_fcn;
    s->num_transitions++;
}

void state_setOutput(State* s, void (*output_fcn)(void)){
    s->perform_output = output_fcn;
}
*/

#endif
