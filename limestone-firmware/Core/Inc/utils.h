/*
 * Current and voltage sensing math, PT math, and other utility functions
 *
 * utils.h
 *
 *  Created on: July 4th, 2025
 *      Author: felixfb
 */
#ifndef PTUTILS_H
#define PTUTILS_H

#include "main.h"

#define PT_DIVIDER		(float)((5.0 + 3.3) / 5.0)


// stores PT info
// zero_V - offset voltage, the PT output voltage at 0 pressure
// pres_range - pressure range of the PT
// max_V - the PT output voltage at pres_range
typedef struct {
	float zero_V;
	float pres_range;
	float max_V;
} PT_t;


// Calculate PT pressure from raw ADC value
//This assumes the raw value is from a MAX11128 or other 12bit ADC, and the circuit uses a 3.3:5 voltage divider
float PT_calc(PT_t PT_info, uint16_t raw);


// Calculate current from raw ADC value.
// This assumes the raw value is from a MAX11128 or other 12bit ADC, and the circuit uses a 50 V/V current sense op-amp, which all of Limelight's current sensing uses.
// res is the resistance of the shunt resistor in milliohms
// divider is whether or not the output of the op-amp goes through a voltage divider.
// If divider is 1, it is assumed that the divider is 2:3, such as a 2k and 3k resistor, or a 10k and 15k.
// All of Limelight's current sensing dividers follow this ratio
// Returns current in Amps
float current_sense_calc(uint16_t raw, uint16_t res, uint8_t divider);



// Calculate bus voltage from raw ADC value.
// Assumes raw value is from a 12 bit ADC.
// resA is the resistance between the bus and the ADC pin
// resB is the resistance between the ADC pin and GND
float bus_voltage_calc(uint16_t raw, uint32_t resA, uint32_t resB);

#endif
