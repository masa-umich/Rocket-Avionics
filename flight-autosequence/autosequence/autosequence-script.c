#include "autosequence-script.h"
#include "helpers/simplified-curve-fit.c"
#include "apogee-detection-revised/ad-functions.h"

extern const uint32_t dt;
const int ALTITUDE_BUFFER_SIZE = 11;
const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15000; // 15 seconds

 
void execute_flight_autosequence(){
    FlightPhase phase = ST_DETECT_VALVES_OPEN; // starts at state1

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = getTime();

    Detector baro_detector = {0};
    Detector imu_detector = {0};
    Detector temp_C_detector = {0};

    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;

    uint32_t handoff_timestamp = 0; // time (in ms) when handoff occursy
    uint32_t ignition_timestamp = 0;// approximate time (in ms) when igntion occurs
    uint32_t meco_timestamp = 0;         // time (in ms) when MECO is detected
    uint32_t lockout_timestamp = 0; // will be computed when MECO_flag is set

    // ALL CALCULATED WITH KINEMATICS!
    uint32_t fallback_apogee_time = 0; // fallback time estimate
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    int altitude_buf_size = 0;
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    int heights_recorded = 0;

    float ground_temp_C = 0.0f; // to be set at handoff

    handoff_timestamp = getTime();

    for (;;){

        float bar1 = 0.0f;
        float bar2 = 0.0f;
        float imu1 = 0.0f;
        float imu2 = 0.0f;

        //temp from barometers not yet implemented in barometer driver
        //but would be helpful here, otherwise will need to estimate it
        float bar1_temp_C = 0.0f;
        float bar2_temp_C = 0.0f;
        float bar1_temp_K = 0.0f;
        float bar2_temp_K = 0.0f;

        float accel_x = 0.0f;
        float accel_y = 0.0f;
        float accel_z = 0.0f;
        float gyro_x = 0.0f;
        float gyro_y = 0.0f;

        float post_lockout_accel = 0.0f;
        float post_lockout_vel = 0.0f;
        float post_lockout_alt = 0.0f;

        get_sensor_data(&bar1, &bar2,
                        &imu1, &imu2,
                        &bar1_temp_C, &bar2_temp_C,
                        &bar1_temp_K, &bar2_temp_K,
                        &accel_x, &accel_y, &accel_z,
                        &gyro_x, &gyro_y);

        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR); // insert temp in K or C?

                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    ignition_timestamp = getTime(); // approximate ignition time

                    int idx = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    ground_temp_C = temp_C_detector.average[idx];
                }

                else if (getTime() - handoff_timestamp > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    abort();
                    //return;
                }
            }
            
            case ST_WAIT_MECO: {
                energizeMPV1();
                energizeMPV2();
                
                insert(&imu_detector, imu1, imu2, phase, IMU_DTR);
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR);

                if(imu_detector.avg_size >= AD_CAPACITY) {

                    if (detect_event(&imu_detector, phase)) {
                        MECO_flag = 1;
                        meco_timestamp = getTime();
                        phase = ST_WAIT_APOGEE;
                        
                        //COMPUTE LOCKOUT/WAIT-TIME
                        float avg_pressure;
                        float avg_temp;

                        //if baro buffer is ready just get the average
                        if(baro_detector.avg_size >= AD_CAPACITY) {
                            int idx = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                            avg_pressure = baro_detector.average[idx];
                        }
                        //emergency fallback, avg the two most recent barometer readings
                        else {
                            avg_pressure = 0.5f * ((bar1) + (bar2)); //MUST be hPa/mbar coming in from Rocket_h
                        }


                        //if temp buffer is ready just get the average
                        if (temp_C_detector.avg_size >= AD_CAPACITY) {
                            int idx = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                            avg_temp = temp_C_detector.average[idx];
                        }
                        //emergency fallback, avg the two most recent temperature readings
                        else {
                            avg_temp = 0.5f * ((bar1_temp_C) + (bar2_temp_C)); //MUST be Celsius coming in from Rocket_h
                        }

                        lockout_timestamp = pdMS_TO_TICKS(compute_wait_time(meco_timestamp, avg_pressure, avg_temp)) + meco_timestamp;
                
                    }
                }

                break;
            } 

            case ST_MACH_LOCKOUT:{   
                insert(&baro_detector,bar1, bar2, phase, BARO_DTR);

                if (getTime() >= lockout_timestamp){
                    phase = ST_WAIT_APOGEE;
                }
                
                break;

            } 

            case ST_WAIT_APOGEE: {
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) {
                    altitude_readings[altitude_buf_size] = compute_height((bar1 + bar2) / 2.0f);
                    time_readings[altitude_buf_size] = getTime() / 1000.0f; // convert ms to seconds
                    wait(50); // wait 50 ms for next reading
                    altitude_buf_size++;
                }
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                         &post_lockout_accel, &post_lockout_vel, &post_lockout_alt);

                    heights_recorded = 1;
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, post_lockout_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time);
                }

                if(baro_detector.slope_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                    }
                }

                break;
            } 

            case ST_WAIT_DROGUE: {
                deployPilot();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
            
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)){
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }
                }

                break;
            } 


            case ST_WAIT_MAIN: {
                deployDrogue();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if(baro_detector.avg_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }
                }

                break;
            } 

            case ST_WAIT_GROUND: {
                deployMain();
                low_power_mode();
                break;
            } 

            case ST_DONE: {
                low_power_mode();
                break;
            }

            default:
                break;
            
        }//end switch(phase)

        vTaskDelayUntil(&last, period);
    }
}