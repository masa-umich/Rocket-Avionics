# Valve Driver
README Last Updated: 1/11/2025

Primary Author: Jackmh

## Overview
This driver is used to control valves on the board. This includes control of the shift register (SN74LV164A), a wrapper of HAL GPIO functions for toggling digital pins, and control MOSFETS attached to the valve channels.

## Example usage:
```c
Shift_Reg reg = {0};
reg.VLV_CTR_GPIO_Port = GPIOA;
reg.VLV_CTR_GPIO_Pin = VLV_CTRL_Pin;
reg.VLV_CLK_GPIO_Port = GPIOA;
reg.VLV_CLK_GPIO_Pin = BUFF_CLK_Pin;
reg.VLV_CLR_GPIO_Port = GPIOA;
reg.VLV_CLR_GPIO_Pin = BUFF_CLR_Pin;

Valve VLV1 = {0};
VLV1.VLV_EN_GPIO_Port = GPIOC;
VLV1.VLV_EN_GPIO_Pin = VLV_EN1_Pin;

VLV_Set_Voltage(reg, 0b01000000);
for (;;) {
    VLV_Toggle(VLV1);
    HAL_GPIO_TogglePin(GPIOE, LED_BLUE_Pin);
    osDelay(1000);
}
```