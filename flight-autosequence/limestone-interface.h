#ifndef LIMESTONE_INTERFACE_H
#define LIMESTONE_INTERFACE_H

#include "main.h"
#include <stdint.h>

uint32_t getTime(){
    return HAL_GetTick();
}

#endif