#include "autosequence-script.h"
#include "helpers/simplified-curve-fit.c"
#include "apogee-detection-revised/ad-functions.h"

 
void execute_flight_autosequence(){
    // starts by waiting for valves to open
    FlightPhase phase = ST_DETECT_VALVES_OPEN; 

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = getTime();

    // initialize detectors for pressure, accleration, and temperature
    // CT: why is everything initialized to zero?
    Detector baro_detector = {0};
    Detector imu_detector = {0};
    Detector temp_C_detector = {0};

    // flags for recovery deployment events, to be set when events are detected
    int MECO_flag = 0;
    int apogee_flag = 0;
    int drogue_flag = 0;
    int main_flag = 0;
    int landed_flag = 0;

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = 0;
    uint32_t meco_timestamp = 0;         
    uint32_t lockout_timestamp = 0; 

    // fallback time estimates -- ALL CALCULATED WITH KINEMATICS!
    uint32_t fallback_apogee_time = 0; 
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    // for velocity/acceleration estimation post lockout
    int altitude_buf_size = 0; // CT: what the hellyante is this, why is the value zero?
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    int heights_recorded = 0;

    float ground_temp_C = 0.0f; // to be set at handoff

    handoff_timestamp = getTime();

    // initialize baro and imu values 
    float bar1 = 0.0f;
    float bar2 = 0.0f;
    float imu1 = 0.0f;
    float imu2 = 0.0f;

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

    // infinite loop to run the sequence, broken by RTOS interrupts
    for (;;){

        // get sensor data at the beginning of each loop iteration
        get_sensor_data(&bar1, &bar2,
                        &imu1, &imu2,
                        &bar1_temp_C, &bar2_temp_C,
                        &bar1_temp_K, &bar2_temp_K);
        

        // execute different code depending on which phase of flight we're in
        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                // insert temperature reading into temp detector
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR); // insert temp in K or C?

                // check if valves are open to transition to next phase
                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    ignition_timestamp = getTime();

                    // set ground temp at handoff for calculating altitude for non-standard atmosphere conditions
                    int idx = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    ground_temp_C = temp_C_detector.average[idx];
                }

                // if we wait too long without valves opening, ABORT!
                else if (getTime() - handoff_timestamp > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    abort();
                    //return;
                }
            }
            
            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
                // energize MPVs while in this phase
                energizeMPV1();
                energizeMPV2();
                
                // insert accel, pressure, and temp readings here
                insert(&imu_detector, imu1, imu2, phase, IMU_DTR);
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR);

                // check if imu detector is full of readings
                if(imu_detector.avg_size >= AD_CAPACITY) {

                    // check if MECO detected - when accel becomes negative
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
                            avg_temp = 0.5f * ((bar1_temp_K) + (bar2_temp_K)); // compute_wait_time expects temp in K
                        }

                        lockout_timestamp = pdMS_TO_TICKS(compute_wait_time(meco_timestamp, avg_pressure, avg_temp)) + meco_timestamp;
                
                    }
                }

                break;
            } 

            // cannot take pressure readings while above mach 1
            case ST_MACH_LOCKOUT:{   
                // insert accel reading
                insert(&imu_detector, imu1, imu2, phase, IMU_DTR); // CT: I am confused what is happening here

                // check if current time is greater than the estimated lockout timestamp
                if (getTime() >= lockout_timestamp){ // CT: is lockout entirely calculated with a timestamp?
                    phase = ST_WAIT_APOGEE;
                }
                
                break;

                // CT: I feel like we should have something that detects if we have gone supersonic at all:
                /*
                Here is my idea:
                    1. Add a flag during the MECO detect phase that detects a spike in pressure
                        - when the spike or there is a peak in pressure as seen in BPS's data that
                        Bryn has, the flag is marked true. 
                        - This could also include accelerometer data, there should also be a spike in axial acceleration
                        during the transition from subsonic to transonic to supersonic
                    2. During the Mach-Lockout phase, the flight computer searches for the exiting transonic spikes
                        - The flag is turned off after a spike in both pressure and axial acceleration 
                    3. Move onto the ST_WAIT_APOGEE phase

                    Lmk your thoughts king. I will need to lock in on MASTRAN.
                */

            } 

            case ST_WAIT_APOGEE: {
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) {
                    // CT: peep compute_height implementation
                    altitude_readings[altitude_buf_size] = compute_height((bar1 + bar2) / 2.0f);
                    // CT: are we this starved of memory that we need to use floats instead of doubles? (just curious)
                    time_readings[altitude_buf_size] = getTime() / 1000.0f; // convert ms to seconds
                    wait(50); // wait 50 ms for next reading
                    altitude_buf_size++;
                }
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                         &post_lockout_accel, &post_lockout_vel, &post_lockout_alt);

                    heights_recorded = 1;
                    // CT: where is post_lockout_vel calculated?
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, post_lockout_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time);
                }

                // CT: is this also checking if the moving average is filled?
                // also, are we waiting for all three, or if only one detects we can move on?
                if(baro_detector.slope_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_detection_worked = 1;
                    }
                    
                    // CT: is the intention to have two backup times? A hardcoded and a kinematics?
                    else if (getTime() >= fallback_apogee_time) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        fallback_timers_worked = 1;
                    }

                    else if (getTime() >= APOGEE_CONSTANT_TIMER) {
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
                    // CT: why is the detect event with baro exclusive to if the apogee detection worked?
                    /*
                        My point is that I think we should have it always prioritize the sensor 
                        detection method regardless of how the script moved onto the next phase.

                        Another thing is that I think that we should consider implementing a backup
                        deploy signal in case that it doesn't detect an increase in acceleration from the parachute.

                        If the previous parachute never deploys, the next one never will either. We shouldn't move
                        onto the next phase unless we detect a parachute's acceleration.
                    */
                    if (apogee_detection_worked && detect_event(&baro_detector, phase)) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }

                    else if (fallback_timers_worked && getTime() >= fallback_5k_time) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }

                    else if (getTime() >= DROGUE_CONSTANT_TIMER) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                    }
                }

                break;
            } 


            case ST_WAIT_MAIN: {
                deployDrogue();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // CT: Same thing as wait_drogue here!

                if(baro_detector.avg_size >= AD_CAPACITY) {
                    if (apogee_detection_worked && detect_event(&baro_detector, phase)) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }

                    else if (fallback_timers_worked && getTime() >= fallback_1k_time) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }

                    else if (getTime() >= MAIN_CONSTANT_TIMER) {
                        main_flag = 1;
                        phase = ST_DONE;
                    }
                }

                break;
            } 

            case ST_WAIT_GROUND: {
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                
                if (detect_event(&baro_detector, phase)) {
                    phase = ST_DONE;
                    landed_flag = 1;
                }

                // CT: why is this not the first function call like in drogue?
                deployMain();
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