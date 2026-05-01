#include "autosequence-script.h"


const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15 * 1000; // 15 seconds
const uint32_t LOCKOUT_END_TIME = 30 * 1000; // 30 seconds, hard cutoff for lockout phase end

const uint32_t APOGEE_CONSTANT_TIMER = 60 * 1000; // 60 seconds in milliseconds
const uint32_t DROGUE_CONSTANT_TIMER = 1200 * 1000; // 1200 seconds in milliseconds
const uint32_t MAIN_CONSTANT_TIMER = 1500 * 1000; // 1500 seconds in milliseconds


// System defs
const uint32_t period = 20; // ms, greater than sampling period of 20 ms
const uint32_t APOGEE_AGREEMENT_WINDOW = 2 * 1000; // 2 seconds -- how long after fluctus apogee detection we should trust our script's apogee detection for agreement purposes

 
int execute_flight_autosequence(Autos_boot_t *boot_params){
    // starts by waiting for valves to open
    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    FlightPhase phase = ST_DETECT_VALVES_OPEN;
    uint32_t last = getTime();

    // initialize detectors for pressure, accleration, and temperature
    Detector baro_detector = {0};
    Detector imu_y_detector = {0};
    Detector temp_C_detector = {0};

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = 0;
    uint32_t meco_timestamp = 0;         
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;
    uint32_t landed_timestamp = 0;

    // altitudes of chute deployment
    float apogee_altitude = 0.0f;
    float drogue_altitude = 0.0f;
    float main_altitude = 0.0f;

    // fallback time estimates
    uint32_t fallback_apogee_time = 0; 
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    float fallback_apogee_altitude = 0.0f;
    float fallback_5k_altitude = 0.0f;
    float fallback_1k_altitude = 0.0f;

    // for velocity/acceleration estimation post lockout
    uint8_t altitude_buf_size = 0;
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    uint8_t heights_recorded = 0;
    int loop_ctr = 0;

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

    // initialize accel, velocity, and altitude for kinematics-based fallback time estimates
    float post_lockout_accel = 0.0f;
    float post_lockout_vel = 0.0f;
    float post_lockout_alt = 0.0f;
    float approximated_accel = 0.0f;

    // flags
    uint8_t apogee_detection_worked = 0;
    uint8_t fallback_timers_worked = 0;
    uint8_t constant_timers_worked = 0;
    uint8_t MECO_flag = 0;
    uint8_t apogee_flag = 0;
    uint8_t drogue_flag = 0;
    uint8_t main_flag = 0;
    uint8_t landed_flag = 0;

    // Fluctus FC detection flags
    uint8_t fluctus_apogee_detected = 0;
    uint8_t fluctus_5k_detected = 0;
    uint8_t fluctus_1k_detected = 0;

    // Fluctus FC detection timers
    uint32_t fluctus_apogee_timestamp = 0;

    // infinite loop to run the sequence, broken by RTOS interrupts
    for (;;){
        if (should_abort()) {
            return -1;
            //return;
        }

        // get sensor data at the beginning of each loop iteration
        get_sensor_data(&bar1, &bar2,
                        &imu1_vals, &imu2_vals,
                        &bar1_temp_C, &bar2_temp_C);
        imu1_y = imu1_vals.XL_y;
        imu2_y = imu2_vals.XL_y;


        // execute different code depending on which phase of flight we're in
        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                // insert temperature reading into temp detector
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR);
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // check if valves are open to transition to next phase
                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    ignition_timestamp = getTime();

                    // set ground temp at handoff for calculating altitude for non-standard atmosphere conditions
                    uint8_t idx_temp = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    T_GROUND = temp_C_detector.average[idx_temp];

                    // set ground temp at handoff
                    uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    P_GROUND = baro_detector.average[idx_baro];

                    // calculate drogue and main deploy pressures based on ground temp and pressure
                    DROGUE_DEPLOY_PRESSURE = compute_pressure(DROGUE_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
                    MAIN_DEPLOY_PRESSURE = compute_pressure(MAIN_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
                    clear(&baro_detector);
                }

                // if we wait too long without valves opening, ABORT!
                else if (time_since(handoff_timestamp) > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    return -1;
                    //return;
                }

                break;
            }
            
            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
                // insert accel and baro readings here
                insert(&imu_y_detector, imu1_y, imu2_y, phase, IMU_DTR);

                // check if imu detector is full of readings
                if(imu_y_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&imu_y_detector, phase)) {
                        MECO_flag = 1;
                        meco_timestamp = time_since(ignition_timestamp);
                        phase = ST_MACH_LOCKOUT;
                        clear(&imu_y_detector);
                    }

                    // if accelerometers don't work just go straight to apogee detection 
                    else if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME){
                        phase = ST_WAIT_APOGEE;
                        clear(&imu_y_detector);
                    }
                }

                break;
            } 

            // cannot take pressure readings while above mach 1
            case ST_MACH_LOCKOUT:{   
                // stay in lockout until we can take reliable barometer readings again, then transition to apogee detection
                if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME){
                    phase = ST_WAIT_APOGEE;
                }
                
                break;

            } 
            
            // after lockout, waiting for apogee detection
            case ST_WAIT_APOGEE: {
                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if (!fluctus_apogee_detected){
                    if (get_fluctus_apogee()) {
                        fluctus_apogee_detected = 1;
                        fluctus_apogee_timestamp = time_since(ignition_timestamp);
                    }
                }

                // insert altitude readings into buffer once 
                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) { 
                    if (loop_ctr % ALT_BUF_ELT_SPACING == 0) {
                        altitude_readings[altitude_buf_size] = compute_height((bar1 + bar2)/ 2.0f); // in meters
                        time_readings[altitude_buf_size] = time_since(ignition_timestamp) / 1000.0f; // in seconds
                        altitude_buf_size++;
                    }
                }
                // if buffer is full -> fit a quadratic curve to get estimate of velocity 
                // and acceleration, then compute fallback times
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                        &post_lockout_accel, &post_lockout_vel, &post_lockout_alt, ALTITUDE_BUFFER_SIZE);
                    
                    if (post_lockout_accel < 0) { // sanity check to make sure curve fitting worked and we got a negative acceleration value
                        approximated_accel = -sqrt(post_lockout_accel*-G); // average acceleration during descent (assuming linear change from post-lockout accel to -G at apogee)
                    }
                    else {
                        approximated_accel = -0.1; // if something went wrong with curve fitting, set accel to -0.1 so fallback timers will never predict apogee
                    }
                    
                    heights_recorded = 1;
                    fallback_apogee_time = time_since(ignition_timestamp);
                    fallback_5k_time = time_since(ignition_timestamp);
                    fallback_1k_time = time_since(ignition_timestamp);
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, approximated_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time,
                                        &fallback_apogee_altitude, &fallback_5k_altitude, &fallback_1k_altitude);
                }

                // check if apogee detected with barometer detector, if we have enough barometer readings
                else if(baro_detectorslope_size >= AD_CAPACITY) {
                    // NEW LOGIC: Also check if baro_speed drops below zero
                    uint32_t current_time = time_since(ignition_timestamp);
                    uint8_t apogee_by_detection = detect_event(&baro_detector, phase);
                    uint8_t apogee_by_fallback = current_time >= fallback_apogee_time;
                    uint8_t apogee_by_constant_timer = current_time >= APOGEE_CONSTANT_TIMER;

                    uint8_t script_apogee_detected = apogee_by_detection || apogee_by_fallback || apogee_by_constant_timer;
                    uint8_t fluctus_apogee_agreement = script_apogee_detected && fluctus_apogee_detected;

                    uint8_t fluctus_override = fluctus_apogee_detected && time_since(fluctus_apogee_timestamp) > APOGEE_AGREEMENT_WINDOW;

                    if (fluctus_apogee_agreement || fluctus_override) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_timestamp = current_time;
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        apogee_altitude = compute_height(baro_detector.average[idx_baro]);
                        
                        if (apogee_by_detection) {
                            apogee_detection_worked = 1;
                        } 
                        else if (apogee_by_fallback){
                            fallback_timers_worked = 1;
                        }
                        else if (apogee_by_constant_timer){
                            constant_timers_worked = 1;
                        }
                    }
                }
                loop_ctr++;
                break;
            } 

            // after apogee, waiting for drogue deployment detection
            case ST_WAIT_DROGUE: {
                deployPilot();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp); 

                    uint8_t fiveK_by_detection = detect_event(&baro_detector, phase);
                    uint8_t fiveK_by_fallback = current_time >= fallback_5k_time;
                    uint8_t fiveK_by_constant_timer = current_time >= DROGUE_CONSTANT_TIMER;

                    // TIER 1: If apogee logic worked, strictly lock out ALL timers.
                    if (apogee_detection_worked) {
                        if (fiveK_by_detection) {
                            drogue_flag = 1;
                        }
                    } 
                    // TIER 2: Apogee failed, so we fall back to computed timers.
                    else if (fallback_timers_worked) {
                        if (fiveK_by_fallback) {
                            drogue_flag = 1;
                        }
                    } 
                    // TIER 3: Both primary and secondary failed. 
                    else if (constant_timers_worked) {
                        if (fiveK_by_constant_timer) {
                            drogue_flag = 1;
                        }
                    }
                    // TIER 4: Use barometer detection if nothing works, which is fine because we're trusting fluctus equally
                    else {
                        if (fiveK_by_detection) {
                            drogue_flag = 1;
                        }
                    }

                    if (get_fluctus_5k()) {
                        drogue_flag = 1;
                        fluctus_5k_detected = 1;
                    }

                    if (drogue_flag) {
                        phase = ST_WAIT_MAIN;
                        drogue_timestamp = current_time;
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        drogue_altitude = compute_height(baro_detector.average[idx_baro]);
                    }
                }
                break;
            }


            case ST_WAIT_MAIN: {
                deployDrogue();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if(baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp);

                    uint8_t oneK_by_detection = detect_event(&baro_detector, phase);
                    uint8_t oneK_by_fallback = current_time >= fallback_1k_time;
                    uint8_t oneK_by_constant_timer = current_time >= MAIN_CONSTANT_TIMER;

                    // TIER 1: Primary Method (Apogee logic worked, rely strictly on altitude)
                    if (apogee_detection_worked) {
                        if (oneK_by_detection) {
                            main_flag = 1;
                        }
                    }
                    // TIER 2: Secondary Method (Apogee failed, rely strictly on calculated fallback)
                    else if (fallback_timers_worked) {
                        if (oneK_by_fallback) {
                            main_flag = 1;
                        }
                    }
                    // TIER 3: Ultimate Backup (Neither worked, use raw constant timer)
                    else if (constant_timers_worked) {
                        if (oneK_by_constant_timer) {
                            main_flag = 1;
                        }
                    }
                    // TIER 4: Use barometer detection if nothing works, which is fine because we're trusting fluctus equally
                    else {
                        if (oneK_by_detection) {
                            main_flag = 1;
                        }
                    }

                    if (get_fluctus_1k()) {
                        main_flag = 1;
                        fluctus_1k_detected = 1;
                    }

                    if (main_flag) {
                        phase = ST_WAIT_GROUND;
                        main_timestamp = current_time; 
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = baro_detector.average[idx_baro];
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
                        landed_timestamp = time_since(ignition_timestamp);
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
        vTaskDelayUntil(&last, period);
    }
}
