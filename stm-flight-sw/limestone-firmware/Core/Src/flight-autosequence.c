/*
 * flight-autosequence.c
 *
 *  Created on: Feb 12, 2026
 *      Author: felix
 */

#include "flight-autosequence.h"

SemaphoreHandle_t arm_semap;
SemaphoreHandle_t abort_semap;
SemaphoreHandle_t ox_semap;
SemaphoreHandle_t fuel_semap;

void setup_autosequence() {
	arm_semap = xSemaphoreCreateBinary();
	abort_semap = xSemaphoreCreateBinary();
	ox_semap = xSemaphoreCreateBinary();
	fuel_semap = xSemaphoreCreateBinary();
	xSemaphoreTake(arm_semap, 0);
	xSemaphoreTake(abort_semap, 0);
	xSemaphoreTake(ox_semap, 0);
	xSemaphoreTake(fuel_semap, 0);
}

void AutosequenceTask(void *argument) {
	for(;;) {
		xSemaphoreTake(arm_semap, 0);
		xSemaphoreTake(abort_semap, 0);
		xSemaphoreTake(ox_semap, 0);
		xSemaphoreTake(fuel_semap, 0);
		if(xSemaphoreTake(arm_semap, portMAX_DELAY) == pdPASS) {
			// TODO log_message
			uint32_t arm_start = xTaskGetTickCount();
			uint8_t ox_triggered = 0;
			uint8_t fuel_triggered = 0;
			while(xTaskGetTickCount() - arm_start < ARM_TIMEOUT && xSemaphoreTake(abort_semap, 0) != pdPASS) {
				if(!ox_triggered && xSemaphoreTake(ox_semap, 0) == pdPASS) {
					ox_triggered = 1;
				}
				if(!fuel_triggered && xSemaphoreTake(fuel_semap, 0) == pdPASS) {
					fuel_triggered = 1;
				}
				if(fuel_triggered && ox_triggered){
					break;
				}
				osDelay(50);
			}
			if(fuel_triggered && ox_triggered) {
				// TODO autosequence time!!
			}
			else {
				// TODO log_message
			}
		}
		osDelay(1);
	}
}

void trigger_Ox() {
	xSemaphoreGive(ox_semap);
}

void trigger_Fuel() {
	xSemaphoreGive(fuel_semap);
}
