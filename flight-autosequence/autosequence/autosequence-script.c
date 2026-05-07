#include "autosequence-script.h"


// SAMPLING PERIOD
const uint32_t period = 20;                                 // 20 ms, sampling period


// TIMEOUT DEFINITIONS 
const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 20 * 1000;    // 20 seconds in ms
const uint32_t LOCKOUT_END_TIME = 30 * 1000;                // 30 seconds in ms, hard cutoff for lockout phase end
const uint32_t AGREEMENT_WINDOW = 3 * 1000;          // 3 seconds in ms


int execute_flight_autosequence(Autos_boot_t boot_params){
    // starts by waiting for valves to open
    uint8_t fluctus_disabled = 0;
    FlightPhase phase = boot_params.phase;

    if (boot_params.phase > ST_DETECT_VALVES_OPEN) {
        fluctus_disabled = 1;
        boot_params.fluctus_disabled = fluctus_disabled;
    }
    else if (boot_params.phase == ST_DISARMED) {
        phase = ST_DETECT_VALVES_OPEN;
        boot_params.phase = phase;
    }

    {
    	char logmsg[sizeof(FC_STAT_AUTOS_ENTER) + 1];
    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_ENTER, (uint8_t) phase);
    	log_message(logmsg, -1);
    }
    uint32_t last = getTime();

    // initialize detectors for pressure, acceleration, and temperature
    Detector baro_detector = {0};
    Detector imu_y_detector = {0};
    Detector temp_C_detector = {0};

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = getTime();
    uint32_t ignition_timestamp = (boot_params.phase > ST_DETECT_VALVES_OPEN) ? getTime() - boot_params.current_time_in_flight : 0;
    uint32_t meco_timestamp = 0;
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;
    uint32_t landed_timestamp = 0;

    // altitudes of chute deployment
    float apogee_altitude = 0.0f;
    float drogue_altitude = 0.0f;
    float main_altitude = 0.0f;

    UNUSED(meco_timestamp);
    UNUSED(apogee_timestamp);
    UNUSED(drogue_timestamp);
    UNUSED(main_timestamp);
    UNUSED(landed_timestamp);
    UNUSED(apogee_altitude);
    UNUSED(drogue_altitude);
    UNUSED(main_altitude);


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
    uint16_t loop_ctr = 0;

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

    // Fluctus FC detection flags
    uint8_t fluctus_apogee_detected = 0;
    uint8_t fluctus_5k_detected = 0;
    uint8_t fluctus_1k_detected = 0;

    // Fluctus FC detection timers
    uint8_t override_calc_fallback_timers = 0;
    uint32_t fluctus_apogee_timestamp = 0;
    uint32_t fluctus_5k_timestamp = 0;
    uint32_t fluctus_1k_timestamp = 0;

    if (boot_params.phase > ST_DETECT_VALVES_OPEN) {
        fallback_apogee_time     = boot_params.fallback_apogee_time;
        fallback_5k_time         = boot_params.fallback_5k_time;
        fallback_1k_time         = boot_params.fallback_1k_time;
        apogee_detection_worked  = boot_params.apogee_detection_worked;
        fallback_timers_worked   = boot_params.fallback_timers_worked;
        fluctus_apogee_detected  = boot_params.fluctus_apogee_detected;
        fluctus_5k_detected = boot_params.fluctus_5k_detected;
        fluctus_1k_detected = boot_params.fluctus_1k_detected;
        fluctus_apogee_timestamp = boot_params.fluctus_apogee_timestamp;
        fluctus_5k_timestamp = boot_params.fluctus_5k_timestamp;
        fluctus_1k_timestamp = boot_params.fluctus_1k_timestamp;
        override_calc_fallback_timers = boot_params.override_calc_fallback_timers;
        heights_recorded         = boot_params.heights_recorded;
        MAIN_DEPLOY_PRESSURE     = boot_params.main_deploy_pressure;
        DROGUE_DEPLOY_PRESSURE   = boot_params.drogue_deploy_pressure;
        T_GROUND                  = boot_params.t_ground;
        P_GROUND                  = boot_params.p_ground;
    }

    // infinite loop to run the sequence, broken by RTOS interrupts
    uint32_t num_cycles = 0;
    for (;;){
        if (should_abort()) {
            return -1;
        }
        update_state_in_telem(phase);
		update_boot_params(&boot_params);

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
                    boot_params.phase = phase;
                    ignition_timestamp = getTime();

                    // set ground temp at handoff for calculating altitude
                    uint8_t idx_temp = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    T_GROUND = temp_C_detector.average[idx_temp];
                    
                    // set ground temp at handoff
                    uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    P_GROUND = baro_detector.average[idx_baro];
                
                    // calculate drogue and main deploy pressures based on ground temp and pressure
                    DROGUE_DEPLOY_PRESSURE = compute_pressure(DROGUE_DEPLOY_ALTITUDE);
                    MAIN_DEPLOY_PRESSURE = compute_pressure(MAIN_DEPLOY_ALTITUDE);

                    boot_params.drogue_deploy_pressure = DROGUE_DEPLOY_PRESSURE;
                    boot_params.main_deploy_pressure = MAIN_DEPLOY_PRESSURE;

                    boot_params.t_ground = T_GROUND;
                    boot_params.p_ground = P_GROUND;
                    clear(&baro_detector);

                    {
                    	char logmsg[sizeof(FC_STAT_AUTOS_MPV_OPEN) + 24];
                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_MPV_OPEN, ignition_timestamp / 1000, (int) T_GROUND, (int) P_GROUND, (int) DROGUE_DEPLOY_PRESSURE, (int) MAIN_DEPLOY_PRESSURE);
                    	log_message(logmsg, -1);
                    }
                }

                // if we wait too long without valves opening, ABORT!
                else if (time_since(handoff_timestamp) > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    return -2;
                }

                break;
            }
            
            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
                // insert accel readings here
                insert(&imu_y_detector, imu1_y, imu2_y, phase, IMU_DTR);

                // check if imu detector is full of readings
                if(imu_y_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&imu_y_detector, phase)) {
                        meco_timestamp = time_since(ignition_timestamp);
                        phase = ST_MACH_LOCKOUT;
                        boot_params.phase = phase;
                        clear(&imu_y_detector);

	                    {
	                    	char logmsg[sizeof(FC_STAT_AUTOS_MECO) + 8];
	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_MECO, meco_timestamp / 1000);
	                    	log_message(logmsg, -1);
	                    }
                    }

                    // if accelerometers don't work just go straight to apogee detection 
                    else if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME){
                        phase = ST_WAIT_APOGEE;
                        boot_params.phase = phase;
                        clear(&imu_y_detector);

                        heights_recorded = 0;
                        boot_params.heights_recorded = heights_recorded;

                        fluctus_apogee_detected = 0;
                        boot_params.fluctus_apogee_detected = fluctus_apogee_detected;

                        log_message(FC_STAT_AUTOS_LOCKOUT, -1);
                    }
                }

                if (get_fluctus_apogee() && !fluctus_disabled) {
                    fluctus_apogee_detected = 1;
                    override_calc_fallback_timers = 1;
                    boot_params.override_calc_fallback_timers = override_calc_fallback_timers;
                    boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                    fluctus_apogee_timestamp = time_since(ignition_timestamp);
                    boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;
                    phase = ST_WAIT_APOGEE;
                    boot_params.phase = phase;

                    {
                    	char logmsg[sizeof(FC_STAT_AUTOS_FLUCTUS_APOGEE_MECO) + 8];
                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_FLUCTUS_APOGEE_MECO, fluctus_apogee_timestamp / 1000);
                    	log_message(logmsg, -1);
                    }
                }
                break;
            } 

            // cannot take pressure readings while above mach 1
            case ST_MACH_LOCKOUT:{
                if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME){
                    phase = ST_WAIT_APOGEE;
                    boot_params.phase = phase;

                    heights_recorded = 0;
                    boot_params.heights_recorded = heights_recorded;

                    fluctus_apogee_detected = 0;
                    boot_params.fluctus_apogee_detected = fluctus_apogee_detected;

                    log_message(FC_STAT_AUTOS_LOCKOUT_END, -1);
                }

                if (get_fluctus_apogee() && !fluctus_disabled) {
                    fluctus_apogee_detected = 1;
                    override_calc_fallback_timers = 1;
                    boot_params.override_calc_fallback_timers = override_calc_fallback_timers;
                    boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                    fluctus_apogee_timestamp = time_since(ignition_timestamp);
                    boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;
                    phase = ST_WAIT_APOGEE;
                    boot_params.phase = phase;

                    {
                    	char logmsg[sizeof(FC_STAT_AUTOS_FLUCTUS_APOGEE_LOCKOUT) + 8];
                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_FLUCTUS_APOGEE_LOCKOUT, fluctus_apogee_timestamp / 1000);
                    	log_message(logmsg, -1);
                    }        
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
                        boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                        fluctus_apogee_timestamp = time_since(ignition_timestamp);
                        boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;

                        log_message(FC_STAT_AUTOS_FLUCTUS_APOGEE, -1);
                    }
                }

                // insert altitude readings into buffer once 
                if (!override_calc_fallback_timers) {
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
                            approximated_accel = -G; // geometric mean to lean towards G
                        }
                        else {
                            approximated_accel = -0.1; // if something went wrong with curve fitting, set accel to -0.1 so fallback timers will never predict apogee
                            post_lockout_vel = 50.0;
                        }
                        
                        heights_recorded = 1;
                        boot_params.heights_recorded = heights_recorded;
                        fallback_apogee_time = time_since(ignition_timestamp);
                        fallback_5k_time = time_since(ignition_timestamp);
                        fallback_1k_time = time_since(ignition_timestamp);
                        compute_fallback_times(post_lockout_alt, post_lockout_vel, approximated_accel,
                                            &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time,
                                            &fallback_apogee_altitude, &fallback_5k_altitude, &fallback_1k_altitude);

                        boot_params.fallback_apogee_time = fallback_apogee_time;
                        boot_params.fallback_5k_time = fallback_5k_time;
                        boot_params.fallback_1k_time = fallback_1k_time;

                        {
                            char logmsg[sizeof(FC_STAT_AUTOS_FALLBACK_CALC) + 16];
                        	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_FALLBACK_CALC, (uint32_t) (ignition_timestamp / 1000), (uint16_t) (fallback_apogee_time / 1000), (uint16_t) (fallback_5k_time / 1000), (uint16_t) (fallback_1k_time / 1000));
                        	log_message(logmsg, -1);
                        }
                    }

                    // check if apogee detected with barometer detector, if we have enough barometer readings
                    else if(baro_detector.avg_size >= AD_CAPACITY) {
                        uint32_t current_time = time_since(ignition_timestamp);
                        
                        uint8_t apogee_by_detection = detect_event(&baro_detector, phase);
                        uint8_t apogee_by_fallback  = (current_time >= fallback_apogee_time);
                        uint8_t script_detected     = apogee_by_detection || apogee_by_fallback;

                        uint8_t trigger_apogee = 0;

                        if (fluctus_disabled) {
                            trigger_apogee = script_detected;
                        } else {
                            uint8_t window_expired = (current_time - fluctus_apogee_timestamp) > AGREEMENT_WINDOW;
                            if (fluctus_apogee_detected && (script_detected || window_expired)) {
                                trigger_apogee = 1;
                            }
                        }
                        if (trigger_apogee) {
                            phase = ST_WAIT_DROGUE;
                            boot_params.phase = phase;
                            apogee_timestamp = current_time;
                            uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                            apogee_altitude = compute_height(baro_detector.average[idx_baro]);
                            
                            apogee_detection_worked = 0;
                            fallback_timers_worked = 0;

                            if (apogee_by_detection) {
                                apogee_detection_worked = 1;
                                log_message(FC_STAT_AUTOS_APOGEE_SCRIPT, -1);
                            } 
                            else if (apogee_by_fallback){
                                fallback_timers_worked = 1;
                                log_message(FC_STAT_AUTOS_APOGEE_FALLBACK, -1);
                            }

    	                    {
    	                    	char logmsg[sizeof(FC_STAT_AUTOS_APOGEE_DETECT) + 13];
    	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_APOGEE_DETECT, (uint16_t) apogee_altitude, apogee_timestamp / 1000);
    	                    	log_message(logmsg, -1);
    	                    }

                            boot_params.apogee_detection_worked = apogee_detection_worked;
                            boot_params.fallback_timers_worked = fallback_timers_worked;
                        }
                    }
                }
                else {
                    if ((time_since(ignition_timestamp) - fluctus_apogee_timestamp) > AGREEMENT_WINDOW) {
                        phase = ST_WAIT_DROGUE;
                        boot_params.phase = phase;
                        apogee_timestamp = time_since(ignition_timestamp);
                        apogee_altitude = compute_height((bar1 + bar2) / 2.0f);

                        apogee_detection_worked = 0;
                        fallback_timers_worked = 0;

                        boot_params.apogee_detection_worked = apogee_detection_worked;
                        boot_params.fallback_timers_worked = fallback_timers_worked;

	                    {
	                    	char logmsg[sizeof(FC_STAT_AUTOS_APOGEE_DETECT_FORCE) + 13];
	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_APOGEE_DETECT_FORCE, (uint16_t) apogee_altitude, apogee_timestamp / 1000);
	                    	log_message(logmsg, -1);
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

                if (!fluctus_5k_detected){
                    if (get_fluctus_5k()) {
                        fluctus_5k_detected = 1;
                        boot_params.fluctus_5k_detected = fluctus_5k_detected;
                        fluctus_5k_timestamp = time_since(ignition_timestamp);
                        boot_params.fluctus_5k_timestamp = fluctus_5k_timestamp;
                    }
                }
                
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp);
                    
                    uint8_t fiveK_by_detection = detect_event(&baro_detector, phase) && !fallback_timers_worked;
                    uint8_t fiveK_by_fallback  = (current_time >= fallback_5k_time) && fallback_timers_worked;
                    uint8_t script_detected     = fiveK_by_detection || fiveK_by_fallback;

                    uint8_t trigger_drogue = 0;

                    if (fluctus_disabled) {
                        trigger_drogue = script_detected;
                    } else {
                        uint8_t window_expired = (current_time - fluctus_5k_timestamp) > AGREEMENT_WINDOW;
                        if (fluctus_5k_detected && (script_detected || window_expired)) {
                            trigger_drogue = 1;
                        }
                    }

                    if (trigger_drogue) {
                        phase = ST_WAIT_MAIN;
                        boot_params.phase = phase;
                        drogue_timestamp = current_time;
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        drogue_altitude = compute_height(baro_detector.average[idx_baro]);

	                    {
	                    	char logmsg[sizeof(FC_STAT_AUTOS_5K_DETECT) + 13];
	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_5K_DETECT, (uint16_t) drogue_altitude, drogue_timestamp / 1000);
	                    	log_message(logmsg, -1);
	                    }
                    }
                }
                break;
            }


            case ST_WAIT_MAIN: {
                deployDrogue();
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                if (!fluctus_1k_detected){
                    if (get_fluctus_1k()) {
                        fluctus_1k_detected = 1;
                        boot_params.fluctus_1k_detected = fluctus_1k_detected;
                        fluctus_1k_timestamp = time_since(ignition_timestamp);
                        boot_params.fluctus_1k_timestamp = fluctus_1k_timestamp;
                    }
                }

                if(baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp);
                    
                    uint8_t oneK_by_detection = detect_event(&baro_detector, phase) && !fallback_timers_worked;
                    uint8_t oneK_by_fallback  = (current_time >= fallback_1k_time) && fallback_timers_worked;
                    uint8_t script_detected     = oneK_by_detection || oneK_by_fallback;

                    uint8_t trigger_main = 0;

                    if (fluctus_disabled) {
                        trigger_main = script_detected;
                    } else {
                        uint8_t window_expired = (current_time - fluctus_1k_timestamp) > AGREEMENT_WINDOW;
                        if (fluctus_1k_detected && (script_detected || window_expired)) {
                            trigger_main = 1;
                        }
                    }


                    if (trigger_main) {
                        phase = ST_WAIT_GROUND;
                        boot_params.phase = phase;
                        main_timestamp = current_time; 
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = compute_height(baro_detector.average[idx_baro]);

	                    {
	                    	char logmsg[sizeof(FC_STAT_AUTOS_1K_DETECT) + 13];
	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_1K_DETECT, (uint16_t) main_altitude, main_timestamp / 1000);
	                    	log_message(logmsg, -1);
	                    }
                    }
                }
                break;
            } 

            case ST_WAIT_GROUND: {
                deployMain();
                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                
                // check if landed with barometer detector, if we have enough barometer readings
                if(baro_detector.avg_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        phase = ST_DONE;
                        boot_params.phase = phase;
                        landed_timestamp = time_since(ignition_timestamp);

	                    {
	                    	char logmsg[sizeof(FC_STAT_AUTOS_LAND_DETECT) + 8];
	                    	snprintf(logmsg, sizeof(logmsg), FC_STAT_AUTOS_LAND_DETECT, landed_timestamp / 1000);
	                    	log_message(logmsg, -1);
	                    }
                    }
                }

                
                break;
            } 

            case ST_DONE: {
                // enter low power mode once on ground
                return 0;
                break;
            }

            default: {
                break;
            }
            
        }//end switch(phase)

        // boot_params update
        if (num_cycles % 20 == 0)
            boot_params.current_time_in_flight = time_since(ignition_timestamp);
        
        num_cycles++;
        vTaskDelayUntil(&last, period);
    }
}
