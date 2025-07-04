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

#if defined(FLIGHT_COMPUTER)
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
        GPIO_PinState state = (config & (1 << i)) != 0; // Get the i-th bit of config
        HAL_GPIO_WritePin(reg.VLV_CTR_GPIO_Port, reg.VLV_CTR_GPIO_Pin, state);
        // Generate a clock pulse
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
        // TODO: do this in a way that doesn't block, or at least blocks-less
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
    }
}

void VLV_Set_Conf(Shift_Reg reg,
		uint8_t CH1_enable,
		VLV_Voltage CH1_voltage,
		uint8_t CH2_enable,
		VLV_Voltage CH2_voltage,
		uint8_t CH3_enable,
		VLV_Voltage CH3_voltage) {

	uint8_t sr_conf = (CH3_enable << 5) | (CH3_voltage << 4) | (CH2_enable << 3) | (CH2_voltage << 2) | (CH1_enable << 1) | (CH1_voltage << 0);
	VLV_Set_Voltage(reg, sr_conf);
}

#elif defined(BAY_BOARD)
// Sets the voltage for each valve channel through configuration of the shift register
// config is the desired output for channels Q_a through Q_h of the shift register.
// ex: config = 00000001 would make only Q_a turn on and turn off all others.
// NOTE: This function is blocking and will take a minimum of 2ms to complete.
void VLV_Set_Voltage(Shift_Reg reg, uint16_t config) {

    // Shift in the configuration bits
    for (int i = 11; i >= 0; i--) {
        // Set the data (CTRL) pin
        GPIO_PinState state = (config & (1 << i)) == 0; // Get the i-th bit of config, also invert since the Bay Board shift register is open drain
        HAL_GPIO_WritePin(reg.VLV_CTR_GPIO_Port, reg.VLV_CTR_GPIO_Pin, state);
        // Generate a clock pulse
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
        // TODO: do this in a way that doesn't block, or at least blocks-less
        HAL_GPIO_WritePin(reg.VLV_CLK_GPIO_Port, reg.VLV_CLK_GPIO_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to meet timing requirements
    }
}

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
		VLV_Voltage CH5_voltage) {

	uint16_t sr_conf = (CH5_enable << 9) | (CH5_voltage << 8) | (CH4_enable << 7) | (CH4_voltage << 6) | (CH3_enable << 5) | (CH3_voltage << 4) | (CH2_enable << 3) | (CH2_voltage << 2) | (CH1_enable << 1) | (CH1_voltage << 0);
	VLV_Set_Voltage(reg, sr_conf);
}

#endif


void VLV_Toggle(Valve vlv) {
	HAL_GPIO_TogglePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin);
}

void VLV_En(Valve vlv) {
	HAL_GPIO_WritePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin, GPIO_PIN_SET);
}

void VLV_Den(Valve vlv) {
	HAL_GPIO_WritePin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin, GPIO_PIN_RESET);
}

VLV_OpenLoad isOpenLoad(Valve vlv) {
	if(HAL_GPIO_ReadPin(vlv.VLV_EN_GPIO_Port, vlv.VLV_EN_GPIO_Pin) == GPIO_PIN_SET) return VLV_Energized; // Open load circuitry always reads low if the channel is energized
    return HAL_GPIO_ReadPin(vlv.VLV_OLD_GPIO_Port, vlv.VLV_OLD_GPIO_Pin);
}
