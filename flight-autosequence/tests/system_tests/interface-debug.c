#include "interface-debug.h"

uint32_t getTime_dbg(){
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) == 0) {
        return 1;
    }
    uint32_t millis = ts.tv_sec * INT64_C(1000) + ts.tv_nsec / 1000000;
    return millis;
}

void deployPilot_dbg(FILE *fptr_out){
    char msg[] = "Deploying Pilot\n";
    fprintf(fptr_out, "%s", msg);
};  // function to deploy pilot chute

void deployDrogue_dbg(FILE *fptr_out){
    char msg[] = "Deploying Drogue\n";
    fprintf(fptr_out, "%s", msg);
} // function to deploy drogue chute

void deployMain_dbg(FILE *fptr_out){
    char msg[] = "Deploying Main\n";
    fprintf(fptr_out, "%s", msg);
} // function to deploy main chute

void wait_dbg(uint32_t ms){
    float time_in_s = ms/1000;
    sleep(time_in_s);
} // wait function in ms

//void wait_until_dbg(uint32_t target_time_ms); // wait until target time in ms

void get_sensor_data_dbg(float* bar1, float* bar2,
					 IMU_values* imu1, IMU_values* imu2,
                     float* bar1_temp_C, float* bar2_temp_C, FILE *fptr_in)[

                     ] // function to get sensor data from IMU and barometers

int valves_open_dbg(){
    return 1;
} // returns 1 if both valves are open, 0 otherwise

uint32_t time_since_dbg(uint32_t it){
    return getTime() - it;
}

//void low_power_mode_dbg();
