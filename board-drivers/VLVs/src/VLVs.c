/*
 * valves.c
 *
 *  Created on: January 10, 2025
 *      Author: jackmh
 */

#include "VLVs.h"

// Driver should handle:
// Shift register voltage configuration
// Open-load detection (optional)
// Toggling Valves ofc

// Sets the voltage for each valve channel through configuration of the shift register
// config is the desired output for channels Q_a through Q_h of the shift register.
// ex: config = 00000001 would make only Q_a turn on and turn off all others.
// NOTE: This function is blocking and will take a minimum of 2ms to complete.
void VLV_Set_Voltage(Shift_Reg reg, uint8_t config) {
    // Clear the shift register
    HAL_GPIO_WritePin(reg.VLV_CLR_GPIO_Port, reg.VLV_CLR_GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(reg.VLV_CLR_GPIO_Port, reg.VLV_CLR_GPIO_Pin, GPIO_PIN_SET);

    // Shift in the configuration bits
    for (int i = 7; i >= 0; i--) {
        // Set the data (CTRL) pin
        GPIO_PinState state = config & (1 << i); // Get the i-th bit of config
        HAL_GPIO_WritePin(reg.VLV_CTR_GPIO_Port, reg.VLV_CTR_GPIO_Pin, state);
        // Generate a clock pulse
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
        // TODO: do this in a way that doesn't block, or at least blocks-less
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
    }
}

void VLV_Toggle(Valve vlv) {
	HAL_GPIO_TogglePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin);
}

void VLV_En(Valve vlv) {
	HAL_GPIO_WritePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin, GPIO_PIN_SET);
}

void VLV_Den(Valve vlv) {
	HAL_GPIO_WritePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin, GPIO_PIN_RESET);
}

bool isOpenLoad(Valve vlv) {
    return HAL_GPIO_ReadPin(vlv.VLV_OLD_GPIO_Port, vlv.VLV_OLD_GPIO_Pin);
}
