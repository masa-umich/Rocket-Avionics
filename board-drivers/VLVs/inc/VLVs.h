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

typedef enum {
	VLV_12V = 0U,
	VLV_24V
} VLV_Voltage;

typedef enum {
	VLV_NoLoad = 0, 	// No valve connected to the channel
	VLV_Load = 1, 		 	// Valve connected
	VLV_Energized = -1		// Valve channel is currently energized. This says nothing about whether or not a valve is connected
} VLV_OpenLoad;

typedef struct {
    // Valve Enable Pin
    GPIO_TypeDef * VLV_EN_GPIO_Port;
    uint16_t VLV_EN_GPIO_Pin;
    // Valve Open-Load Detection Pin
    GPIO_TypeDef * VLV_OLD_GPIO_Port;
    uint16_t VLV_OLD_GPIO_Pin;
} Valve;

// Energizes a valve
void VLV_En(Valve vlv);

// De-energizes a valve
void VLV_Den(Valve vlv);

// Sort of just a HAL_GPIO wrapper, toggles a valve to the opposite state
void VLV_Toggle(Valve vlv);

// Uses the open-load detection feature that is assumed to be built into the board
// Returns VLV_NoLoad if no load is detected, VLV_Load if a valve is connected, and VLV_Energized if the channel is energized.
// Open-load detection only means something if the channel is in use (the channel is enabled) and not energized.
// If the channel is not enabled, this will always return VLV_NoLoad
// It is recommended to use current sensing to determine whether there is a valve connected when the channel is energized
VLV_OpenLoad VLV_isOpenLoad(Valve vlv);



#if defined(FLIGHT_COMPUTER)
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

// Sets the voltage for each valve channel through configuration of the shift register
// config is the desired output for channels Q_a through Q_h of the shift register.
// ex: config = 00000001 would make only Q_a turn on and turn off all others.
// NOTE: This function is blocking and will take a minimum of 2ms to complete.
void VLV_Set_Voltage(Shift_Reg reg, uint8_t config);

// Set shift register according to the pinout on the flight computer. 1 is enabled, 0 is disabled
void VLV_Set_Conf(Shift_Reg reg,
		uint8_t CH1_enable,
		VLV_Voltage CH1_voltage,
		uint8_t CH2_enable,
		VLV_Voltage CH2_voltage,
		uint8_t CH3_enable,
		VLV_Voltage CH3_voltage);

#elif defined(BAY_BOARD)
typedef struct {
    // CTRL pin
	GPIO_TypeDef * VLV_CTR_GPIO_Port;
    uint16_t VLV_CTR_GPIO_Pin;

    // Clock pin
    GPIO_TypeDef * VLV_CLK_GPIO_Port;
    uint16_t VLV_CLK_GPIO_Pin;
} Shift_Reg;

// Sets the voltage for each valve channel through configuration of the shift register
// config is the desired output for channels Q_a through Q_h of the shift register.
// ex: config = 00000001 would make only Q_a turn on and turn off all others.
// NOTE: This function is blocking and will take a minimum of 2ms to complete.
void VLV_Set_Voltage(Shift_Reg reg, uint16_t config);

// Set shift register according to the pinout on the bay board. 1 is enabled, 0 is disabled
void VLV_Set_Conf(Shift_Reg reg,
		uint8_t CH1_enable,
		VLV_Voltage CH1_voltage,
		uint8_t CH2_enable,
		VLV_Voltage CH2_voltage,
		uint8_t CH3_enable,
		VLV_Voltage CH3_voltage,
		uint8_t CH4_enable,
		VLV_Voltage CH4_voltage,
		uint8_t CH5_enable,
		VLV_Voltage CH5_voltage);

#else
#error "FLIGHT_COMPUTER or BAY_BOARD must be defined"

#endif


#endif
