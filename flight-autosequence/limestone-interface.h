#ifndef LIMESTONE_INTERFACE_H
#define LIMESTONE_INTERFACE_H

#include "main.h"
#include <stdint.h>
#include "lmp_channels.h"

uint32_t getTime(){
    return HAL_GetTick();
}

void energizeMPV1(){
    HAL_GPIO_WritePin(GPIOB, FC_VLV1_CURRENT_I, GPIO_PIN_SET);
}

void energizeMPV2(){
    HAL_GPIO_WritePin(GPIOB, FC_VLV2_CURRENT_I, GPIO_PIN_SET);
}

#endif