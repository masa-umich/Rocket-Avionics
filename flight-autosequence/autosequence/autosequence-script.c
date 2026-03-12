#include "autosequence-script.h"

const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15000; // 15 seconds
const uint32_t MAX_BURN_DURATION_MS = 22000; // 22 seconds

const uint32_t MIN_LOCKOUT_WAIT_TIME = 6000; // ms, minimum time we expect to wait in lockout phase
const uint32_t MAX_LOCKOUT_WAIT_TIME = 20000; // ms, maximum time we expect to wait in lockout phase
const int WAIT_TIME_MULTIPLIER = 3;

const uint32_t APOGEE_CONSTANT_TIMER = 100 * 1000; // 100 seconds in milliseconds
const uint32_t DROGUE_CONSTANT_TIMER = 120 * 1000; // 120 seconds in milliseconds
const uint32_t MAIN_CONSTANT_TIMER = 150 * 1000; // 150 seconds in milliseconds


// System defs
const uint32_t period = 20; // ms, sampling period

 
void execute_flight_autosequence(){
    // starts by waiting for valves to open
    FlightPhase phase = ST_DETECT_VALVES_OPEN; 

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = getTime();

    // initialize detectors for pressure, accleration, and temperature
    Detector baro_detector = {0};
    Detector imu_z_detector = {0};
    Detector temp_K_detector = {0};

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = 0;
    uint32_t meco_timestamp = 0;         
    uint32_t lockout_timestamp = 0; 
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;

    // fallback time estimates -- ALL CALCULATED WITH KINEMATICS!
    uint32_t fallback_apogee_time = 0; 
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    uint32_t approach_mach1_timestamp = 0;

    // for velocity/acceleration estimation post lockout
    int altitude_buf_size = 0;
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    int heights_recorded = 0;

    float ground_temp_C = 0.0f; // to be set at handoff

    handoff_timestamp = getTime();

    // initialize baro and imu values 
    float bar1 = 0.0f;
    float bar2 = 0.0f;
    float imu1_y = 0.0f;
    float imu2_y = 0.0f;

    IMU_values imu1_vals = {0};
    IMU_values imu2_vals = {0};

    // initilize temperature values
    float bar1_temp_C = 0.0f;
    float bar2_temp_C = 0.0f;
    float bar1_temp_K = 0.0f;
    float bar2_temp_K = 0.0f;

    // initialize accel, velocity, and altitude for kinematics-based fallback time estimates
    float post_lockout_accel = 0.0f;
    float post_lockout_vel = 0.0f;
    float post_lockout_alt = 0.0f;

    // flags TODO
    int apogee_detection_worked = 0;
    int fallback_timers_worked = 0;
    int sub_to_supersonic_flag = 0;
    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;
    int landed_flag = 0;


    float max_accel_seen = 0.0f; // for detecting sub to supersonic transition

    // infinite loop to run the sequence, broken by RTOS interrupts

    for (;;){
        // ABORT CHECK - maybe move here for testing
        // get sensor data at the beginning of each loop iteration
        get_sensor_data(&bar1, &bar2,
                        &imu1_vals, &imu2_vals,
                        &bar1_temp_C, &bar2_temp_C);
        bar1_temp_K = bar1_temp_C + 273.15;
        bar2_temp_K = bar2_temp_C + 273.15;
        
        imu1_y = imu1_vals.XL_y;
        imu2_y = imu2_vals.XL_y;


        // execute different code depending on which phase of flight we're in
        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                // insert temperature reading into temp detector
                insert(&temp_K_detector, bar1_temp_K, bar2_temp_K, phase, TEMP_DTR);
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // check if valves are open to transition to next phase
                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    ignition_timestamp = getTime();

                    // set ground temp at handoff for calculating altitude for non-standard atmosphere conditions
                    int idx_temp = (temp_K_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    T_GROUND = temp_K_detector.average[idx_temp];

                    int idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    P_GROUND = baro_detector.average[idx_baro];

                    DROGUE_DEPLOY_PRESSURE = compute_pressure(DROGUE_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
                    MAIN_DEPLOY_PRESSURE = compute_pressure(MAIN_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);

                    LAPSE_RATE = T_TROPOPAUSE - T_GROUND / ALT_TROPOPAUSE - GROUND_ALTITUDE;
                }

                // if we wait too long without valves opening, ABORT!
                else if (getTime() - handoff_timestamp > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    abort();
                    //return;
                }
            }
            
            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
#ifdef AUTOSEQUENCE_DEBUG
            	return;
#endif
                
                // insert accel and baro readings here
                insert(&imu_z_detector, imu1_y, imu2_y, phase, IMU_DTR);

                if (!sub_to_supersonic_flag) {
                    insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                    if (detect_acceleration_spike(&imu_z_detector, max_accel_seen)) {
                        sub_to_supersonic_flag = 1;
                        approach_mach1_timestamp = getTime();
                    }
                }

                // check if imu detector is full of readings
                if(imu_z_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&imu_z_detector, phase) || getTime() - ignition_timestamp > MAX_BURN_DURATION_MS) {
                        MECO_flag = 1;
                        meco_timestamp = getTime();

                        if (sub_to_supersonic_flag){
                            phase = ST_MACH_LOCKOUT;

                            // set lockout timer based on how long we were in boost phase after approaching mach 1
                            uint32_t wait = (meco_timestamp - approach_mach1_timestamp) * WAIT_TIME_MULTIPLIER;
                            if (wait < MIN_LOCKOUT_WAIT_TIME)
                                wait = MIN_LOCKOUT_WAIT_TIME;
                            else if (wait > MAX_LOCKOUT_WAIT_TIME)
                                wait = MAX_LOCKOUT_WAIT_TIME;

                            lockout_timestamp = meco_timestamp + wait; 
                            
                        }

                        else {
                            phase = ST_WAIT_APOGEE;
                        }
                        
                    }
                }

                break;
            } 

            // cannot take pressure readings while above mach 1
            case ST_MACH_LOCKOUT:{   
                // check if current time is greater than the estimated lockout timestamp
                /*
                if (!super_to_subsonic_flag) {
                    super_to_subsonic_flag = detect_acceleration_spike(&imu_z_detector, max_accel_seen);
                }
                    */

                if (getTime() - ignition_timestamp>= lockout_timestamp /* || (super_to_subsonic_flag && smooth_acceleration_readings())*/){
                    phase = ST_WAIT_APOGEE;
                }
                
                break;

            } 
            
            // after lockout, waiting for apogee detection
            case ST_WAIT_APOGEE: {
                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // insert altitude readings into buffer once 
                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) {
                    uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    altitude_readings[altitude_buf_size] = compute_height(baro_detector.average[idx_baro]);
                    time_readings[altitude_buf_size] = (getTime() - ignition_timestamp) / 1000.0f; // convert ms to seconds
                    wait(50); // wait 50 ms for next reading
                    altitude_buf_size++;
                }

                // if buffer is full -> fit a quadratic curve to get estimate of velocity 
                // and acceleration, then compute fallback times
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                         &post_lockout_accel, &post_lockout_vel, &post_lockout_alt);

                    heights_recorded = 1;
                    fallback_apogee_time = getTime();
                    fallback_5k_time = getTime();
                    fallback_1k_time = getTime();
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, post_lockout_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time);
                }

                // check if apogee detected with barometer detector, if we have enough barometer readings
                if(baro_detector.slope_size >= AD_CAPACITY) {
                    // if we detect apogee with our detector, set a flag and move to next phase
                    if (detect_event(&baro_detector, phase)) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_detection_worked = 1;
                    }

                    // if our fallback timers predict apogee, set flag and move to next phase
                    else if (getTime() - ignition_timestamp >= fallback_apogee_time) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        fallback_timers_worked = 1;
                    }

                    // if neither work, set constant timer flag
                    else if (getTime() - ignition_timestamp >= APOGEE_CONSTANT_TIMER) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                    }
                }

                break;
            } 

            // after apogee, waiting for drogue deployment detection
            case ST_WAIT_DROGUE: {
                deployPilot();

                // insert pressure reading 
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                
                // check if drogue deployment detected, if we have enough barometer readings
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    // if we detected apogee with apogee detection, detect with barometers
                    if (apogee_detection_worked && detect_event(&baro_detector, phase)) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }

                    // if we detected apogee with fallback timers, detect drogue deployment with fallback timers
                    else if (fallback_timers_worked && getTime() - ignition_timestamp >= fallback_5k_time) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }

                    // otherwise just "detect" with constant timer
                    else if (getTime() - ignition_timestamp >= DROGUE_CONSTANT_TIMER) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }
                }

                break;
            } 


            case ST_WAIT_MAIN: {
                deployDrogue();

                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // same idea as drogue deployment detection
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    if (apogee_detection_worked && detect_event(&baro_detector, phase)) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }

                    else if (fallback_timers_worked && getTime() - ignition_timestamp >= fallback_1k_time) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }

                    else if (getTime() - ignition_timestamp >= MAIN_CONSTANT_TIMER) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }
                }

                break;
            } 

            case ST_WAIT_GROUND: {
                deployMain();
                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                
                // check if landed with barometer detector, if we have enough barometer readings
                if(baro_detector.slope_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        phase = ST_DONE;
                        landed_flag = 1;
                    }
                }

                
                break;
            } 

            case ST_DONE: {
                // enter low power mode once on ground
                // return
                break;
            }

            default:
                break;
            
        }//end switch(phase)

        // wait until thread period is up before next loop iteration
        vTaskDelayUntil(&last, period);
    }
}
