#ifndef LIMESTONE_INTERFACE_H
#define LIMESTONE_INTERFACE_H

#include "main.h"
#include <stdint.h>

uint32_t getTime(){
    return HAL_GetTick();
} // gets time in ms

void energizeMPV1(){
    return;
}    // energizes whichever valve opens first

void energizeMPV2(){
    return;
}   // energizes whichever valve opens second

void deployPilot(){
    return;
}   // function to deploy pilot chute

void deployDrogue(){
    return;
} // function to deploy drogue chute

void deployMain(){
    return;
}  // function to deploy main chute

void wait(uint32_t ms){
    return;
} // wait function in ms

void wait_until(uint32_t target_time_ms){
    return;
} // wait until target time in ms

void get_sensor_data(float* bar1, float* bar2,
                     float* imu1, float* imu2,
                     float* bar1_temp_C, float* bar2_temp_C,
                     float* bar1_temp_K, float* bar2_temp_K,
                     float* accel_x, float* accel_y, float* accel_z,
                     float* gyro_x, float* gyro_y){

    if (xSemaphoreTake(Rocket_h.fcState_access, pdMS_TO_TICKS(5)) == pdPASS) {
        *bar1 = Rocket_h.fcState.bar1; // convert to needed units if necessary
        *bar2 = Rocket_h.fcState.bar2;
        *imu1 = Rocket_h.fcState.imu1_A.XL_x;
        *imu2 = Rocket_h.fcState.imu2_A.XL_x;

        *bar1_temp_C = Rocket_h.fcState.bar1_temp_C;
        *bar1_temp_K = *bar1_temp_C + 273.15f;
        *bar2_temp_C = Rocket_h.fcState.bar2_temp_C;
        *bar2_temp_K = *bar2_temp_C + 273.15f;

        xSemaphoreGive(Rocket_h.fcState_access);
    }
    return;
} // function to get sensor data from IMU and barometers

int valves_open(){
    return 0;
} // returns 1 if both valves are open, 0 otherwise

void abort(){
    return;
}

void low_power_mode(){
    return;
}

#endif