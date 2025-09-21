/*
 * utils.c
 *
 *  Created on: July 4th, 2025
 *      Author: felixfb
 */

#include "utils.h"

float current_sense_calc(uint16_t raw, uint16_t res, uint8_t divider) {
	float adc_v = (raw / 4095.0) * 3.3;
	if(divider) {
		adc_v = (adc_v * 5) / 3.0;
	}
	float shunt_v = adc_v / 50.0;
	return shunt_v / (res / 1000.0);
}

float bus_voltage_calc(uint16_t raw, uint32_t resA, uint32_t resB) {
	float res_divider = (resA + resB) / (float) resB;
	float adc_v = (raw / 4095.0) * 3.3;
	return adc_v * res_divider;
}

float PT_calc(PT_t PT_info, uint16_t raw) {
	float adc_v = (raw / 4095.0) * 3.3;
	float PT_v = adc_v * PT_DIVIDER;
	float PT_slope = (PT_info.max_V - PT_info.zero_V) / PT_info.pres_range;
	return (PT_v - PT_info.zero_V) / PT_slope;
}
