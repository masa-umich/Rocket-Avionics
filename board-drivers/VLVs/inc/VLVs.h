/*
 * valves.h
 *
 *  Created on: January 10, 2025
 *      Author: jackmh
 */

#ifndef VLVS_H
#define VLVS_H

#include "main.h"

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

typedef struct {
    // CTRL pin
	GPIO_TypeDef * VLV_CTR_GPIO_Port;
    uint16_t VLV_CTR_GPIO_Pin;

    // Clock pin
    GPIO_TypeDef * VLV_CLK_GPIO_Port;
    uint16_t VLV_CLK_GPIO_Pin;

    // Clear pin
    GPIO_TypeDef * VLV_CLR_GPIO_Port;
    uint16_t VLV_CLR_GPIO_Pin;
} Shift_Reg;

typedef struct {
    // Valve Enable Pin
    GPIO_TypeDef * VLV_EN_GPIO_Port;
    uint16_t VLV_EN_GPIO_Pin;
    // Valve Open-Load Detection Pin
    GPIO_TypeDef * VLV_OLD_GPIO_Port;
    uint16_t VLV_OLD_GPIO_Pin;
} Valve;

// Sets the voltage for each valve channel through configuration of the shift register
// config is the desired output for channels Q_a through Q_h of the shift register.
// ex: config = 00000001 would make only Q_a turn on and turn off all others.
// NOTE: This function is blocking and will take a minimum of 2ms to complete.
void VLV_Set_Voltage(Shift_Reg reg, uint8_t config);

// Sort of just a HAL_GPIO wrapper, toggles a valve to the opposite state
void VLV_Toggle(Valve vlv);

// Energizes a valve
void VLV_En(Valve vlv);

// De-energizes a valve
void VLV_Den(Valve vlv);

// Uses the open-load detection feature that is assumed to be built into the board
// Returns true if the valve is being energized, false if it is currently de-energized
bool isOpenLoad(Valve vlv)

#endif
