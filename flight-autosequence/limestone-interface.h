#ifndef LIMESTONE_INTERFACE_H
#define LIMESTONE_INTERFACE_H

#include "main.h"
#include <stdint.h>

uint32_t getTime(){
    return HAL_GetTick();
} // gets time in ms

void energizeMPV1(){
    return;
}    // energizes whichever valve opens first

void energizeMPV2(){
    return;
}   // energizes whichever valve opens second

void blowoutPanel(){
    return;
}   // function to blowout the panel

void deployDrogue(){
    return;
} // function to deploy drogue chute

void deployMain(){
    return;
}  // function to deploy main chute

#endif