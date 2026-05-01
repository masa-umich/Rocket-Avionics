#include "test-autosequence-flightdata_lm3.h"

const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15 * 1000; // 15 seconds
const uint32_t LOCKOUT_END_TIME = 30 * 1000; // 30 seconds, hard cutoff for lockout phase end

const uint32_t APOGEE_CONSTANT_TIMER = 60 * 1000; // 60 seconds in milliseconds
const uint32_t DROGUE_CONSTANT_TIMER = 1200 * 1000; // 1200 seconds in milliseconds
const uint32_t MAIN_CONSTANT_TIMER = 1500 * 1000; // 1500 seconds in milliseconds

// System defs
const uint32_t period = 20; // ms, greater than sampling period of 20 ms


int execute_flight_autosequence(){
    // starts by waiting for valves to open
    FlightPhase phase = ST_DETECT_VALVES_OPEN; 

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = getTime();

    // Initialize detectors for computed altitude and vertical acceleration
    Detector alt_detector = {0};
    Detector accel_detector = {0};

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = 0;
    uint32_t meco_timestamp = 0;         
    uint32_t lockout_timestamp = 0; 
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;
    uint32_t approach_mach1_timestamp = 0;
    uint32_t landed_timestamp = 0;

    // altitudes of chute deployment
    float apogee_altitude = 0.0f;
    float drogue_altitude = 0.0f;
    float main_altitude = 0.0f;

    // fallback time estimates -- ALL CALCULATED WITH KINEMATICS!
    uint32_t fallback_apogee_time = 0; 
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    // fallback altitude estimates
    float fallback_apogee_altitude = 0.0f;
    float fallback_5k_altitude = 0.0f;
    float fallback_1k_altitude = 0.0f;

    // for velocity/acceleration estimation post lockout
    uint8_t altitude_buf_size = 0;
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    uint8_t heights_recorded = 0;
    int loop_ctr = 0;

    // --- COMPUTED VARIABLES TO REPLACE RAW SENSORS ---
    float computed_altitude = 0.0f;    // from 'baro-altitude (m)'
    float vert_accel = 0.0f;           // from 'vert-accel (m/s2)'
    float baro_speed = 0.0f;           // from 'baro-speed (m/s)'
    float amb_temp_C = 0.0f;           // from 'amb-temp (deg c)'

    float post_lockout_accel = 0.0f;
    float post_lockout_vel = 0.0f;
    float post_lockout_alt = 0.0f;
    float approximated_accel = 0.0f;

    // flags
    uint8_t apogee_detection_worked = 0;
    uint8_t fallback_timers_worked = 0;
    uint8_t sub_to_supersonic_flag = 0;
    uint8_t MECO_flag = 0;
    uint8_t apogee_flag = 0;
    uint8_t drogue_flag = 0;
    uint8_t main_flag = 0;
    uint8_t landed_flag = 0;

    float max_accel_seen = 0.0f; // for detecting sub to supersonic transition

    // infinite loop to run the sequence, broken by RTOS interrupts
    for (;;){

        // --- UPDATED DATA FETCH ---
        if (get_computed_data(&computed_altitude, &vert_accel, &baro_speed, &amb_temp_C) == 0) {
            printf("End of CSV reached. Simulation complete!\n");
            break; 
        }

        // execute different code depending on which phase of flight we're in
        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                // Check if valves are open to transition to next phase
                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    ignition_timestamp = getTime();
                }

                // if we wait too long without valves opening, ABORT!
                else if (time_since(handoff_timestamp) > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    return -1;
                }
                break;
            }
            
            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
                // insert accel and baro readings here
                insert(&accel_detector, vert_accel, vert_accel, phase, ACCEL_DTR);

                // check if accel detector is full of readings
                if(accel_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&accel_detector, phase) || time_since(ignition_timestamp) > MAX_BURN_DURATION_MS) {
                        MECO_flag = 1;
                        meco_timestamp = time_since(ignition_timestamp);
                        phase = ST_MACH_LOCKOUT;

                    }
                }

                break;
            } 

            case ST_MACH_LOCKOUT: {   
                if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME){
                    phase = ST_WAIT_APOGEE;
                    lockout_timestamp = LOCKOUT_END_TIME;
                }
                break;
            } 
            
            case ST_WAIT_APOGEE: {
                // insert ALTITUDE reading
                insert(&alt_detector, computed_altitude, computed_altitude, phase, ALT_DTR);


                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) { // only record every 5th reading to ensure we have a good spread of data points over time
                    if (loop_ctr % ALT_BUF_ELT_SPACING == 0) {
                        altitude_readings[altitude_buf_size] = computed_altitude;
                        time_readings[altitude_buf_size] = time_since(ignition_timestamp) / 1000.0f; 
                        altitude_buf_size++;
                    }
                }
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                        &post_lockout_accel, &post_lockout_vel, &post_lockout_alt, ALTITUDE_BUFFER_SIZE);
                    
                    approximated_accel = -sqrt(post_lockout_accel*-G); // average acceleration during descent (assuming linear change from post-lockout accel to -G at apogee)

                    heights_recorded = 1;
                    fallback_apogee_time = time_since(ignition_timestamp);
                    fallback_5k_time = time_since(ignition_timestamp);
                    fallback_1k_time = time_since(ignition_timestamp);
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, approximated_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time,
                                        &fallback_apogee_altitude, &fallback_5k_altitude, &fallback_1k_altitude);
                }
                else if(alt_detector.avg_size >= AD_CAPACITY) {
                    // NEW LOGIC: Also check if baro_speed drops below zero
                    uint32_t current_time = time_since(ignition_timestamp);
                    int apogee_by_detection = detect_event(&alt_detector, phase);
                    int apogee_by_fallback = current_time >= fallback_apogee_time;
                    int apogee_by_constant_timer = current_time >= APOGEE_CONSTANT_TIMER;

                    if (apogee_by_detection||
                        apogee_by_fallback || 
                        apogee_by_constant_timer) {
                        
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_timestamp = current_time;
                        apogee_altitude = computed_altitude;
                        
                        if (apogee_by_detection) {
                            apogee_detection_worked = 1;
                        } 
                        else if (apogee_by_fallback){
                            fallback_timers_worked = 1;
                        }
                    }
                }
                loop_ctr++;
                break;
            } 

            case ST_WAIT_DROGUE: {
                //deployPilot();
                insert(&alt_detector, computed_altitude, computed_altitude, phase, ALT_DTR);
                
                if(alt_detector.avg_size >= AD_CAPACITY) {
                    int deploy_drogue = 0;
                    uint32_t current_time = time_since(ignition_timestamp); 

                    int fiveK_by_detection = detect_event(&alt_detector, phase);
                    int fiveK_by_fallback = current_time >= fallback_5k_time;
                    int fiveK_by_constant_timer = current_time >= DROGUE_CONSTANT_TIMER;

                    // TIER 1: If apogee logic worked, strictly lock out ALL timers.
                    if (apogee_detection_worked) {
                        if (fiveK_by_detection) {
                            deploy_drogue = 1;
                        }
                    } 
                    // TIER 2: Apogee failed, so we fall back to computed timers.
                    else if (fallback_timers_worked) {
                        if (fiveK_by_fallback) {
                            deploy_drogue = 1;
                        }
                    } 
                    // TIER 3: Both primary and secondary failed. 
                    else {
                        if (fiveK_by_constant_timer) {
                            deploy_drogue = 1;
                        }
                    }

                    if (deploy_drogue) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                        drogue_timestamp = current_time;
                        drogue_altitude = computed_altitude;
                    }
                }
                break;
            } 

            case ST_WAIT_MAIN: {
                //deployDrogue();
                insert(&alt_detector, computed_altitude, computed_altitude, phase, ALT_DTR);

                if(alt_detector.avg_size >= AD_CAPACITY) {
                    int deploy_main = 0;
                    uint32_t current_time = time_since(ignition_timestamp);

                    int oneK_by_detection = detect_event(&alt_detector, phase);
                    int oneK_by_fallback = current_time >= fallback_1k_time;
                    int oneK_by_constant_timer = current_time >= MAIN_CONSTANT_TIMER;

                    // TIER 1: Primary Method (Apogee logic worked, rely strictly on altitude)
                    if (apogee_detection_worked) {
                        if (oneK_by_detection) {
                            deploy_main = 1;
                        }
                    }
                    // TIER 2: Secondary Method (Apogee failed, rely strictly on calculated fallback)
                    else if (fallback_timers_worked) {
                        if (oneK_by_fallback) {
                            deploy_main = 1;
                        }
                    }
                    // TIER 3: Ultimate Backup (Neither worked, use raw constant timer)
                    else {
                        if (oneK_by_constant_timer) {
                            deploy_main = 1;
                        }
                    }

                    if (deploy_main) {
                        main_flag = 1;
                        phase = ST_WAIT_GROUND;
                        main_timestamp = current_time; 
                        
                        uint8_t idx_alt = (alt_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = alt_detector.average[idx_alt];
                        main_altitude = computed_altitude;
                    }
                }
                break;
            } 

            case ST_WAIT_GROUND: {
                //deployMain();
                insert(&alt_detector, computed_altitude, computed_altitude, phase, ALT_DTR);
                
                if(alt_detector.avg_size >= AD_CAPACITY) {
                    // Detect when altitude stops changing (slope is roughly 0 over time)
                    if (detect_event(&alt_detector, phase)) {
                        phase = ST_DONE;
                        landed_flag = 1;
                        landed_timestamp = time_since(ignition_timestamp);
                    }
                }
                break;
            } 

            case ST_DONE: {
                // enter low power mode once on ground
                break;
            }

            default:
                break;
            
        }//end switch(phase)
        //vTaskDelayUntil(&last, period);
    }

    // --- WRITE RESULTS TO FILE ---
    FILE *outfile = fopen("test-autosequence-flightdata.out", "w");
    if (outfile != NULL) {
        fprintf(outfile, "=================================================\n");
        fprintf(outfile, "          FLIGHT AUTOSEQUENCE RESULTS            \n");
        fprintf(outfile, "=================================================\n\n");
        fprintf(outfile, "Start row: %d\n\n", g_start_row);
        fprintf(outfile, "--- FLAGS ---\n");
        //fprintf(outfile, "Sub to Supersonic:       %d\n", sub_to_supersonic_flag);
        fprintf(outfile, "MECO Detected:           %d\n", MECO_flag);
        fprintf(outfile, "Apogee Detected:         %d\n", apogee_flag);
        fprintf(outfile, "  - Detection Worked:    %d\n", apogee_detection_worked);
        fprintf(outfile, "  - Fallback Worked:     %d\n", fallback_timers_worked);
        fprintf(outfile, "Drogue Deployed:         %d\n", drogue_flag);
        fprintf(outfile, "Main Deployed:           %d\n", main_flag);
        fprintf(outfile, "Landed:                  %d\n\n", landed_flag);

        fprintf(outfile, "--- TIMESTAMPS (ms since ignition) ---\n");
        fprintf(outfile, "MECO Timestamp:          %u ms\n", meco_timestamp);
        fprintf(outfile, "Lockout Ends At:         %u ms\n", lockout_timestamp);
        fprintf(outfile, "Apogee Timestamp:        %u ms\n", apogee_timestamp);
        fprintf(outfile, "Drogue Timestamp:        %u ms\n", drogue_timestamp);
        fprintf(outfile, "Main Timestamp:          %u ms\n", main_timestamp);
        fprintf(outfile, "Landed Timestamp:        %u ms\n\n", landed_timestamp);


        fprintf(outfile, "--- KEY ALTITUDES ---\n");
        fprintf(outfile, "Apogee Altitude:         %.2f m\n", apogee_altitude);
        fprintf(outfile, "Drogue Deploy Altitude:  %.2f m\n", drogue_altitude);
        fprintf(outfile, "Main Deploy Altitude:    %.2f m\n\n", main_altitude);


        fprintf(outfile, "--- POST LOCKOUT KINEMATICS ---\n");
        fprintf(outfile, "Delay after lockout:     %u ms\n", altitude_buf_size * ALT_BUF_ELT_SPACING * period);
        fprintf(outfile, "Altitude:                %.2f m\n", post_lockout_alt);
        fprintf(outfile, "Velocity:                %.2f m/s\n", post_lockout_vel);
        fprintf(outfile, "Acceleration:            %.2f m/s^2\n\n", approximated_accel); // subtract gravity to get net accel


        fprintf(outfile, "--- FALLBACK TIMER PREDICTIONS ---\n");
        fprintf(outfile, "Apogee time:             %u ms\n", fallback_apogee_time);
        fprintf(outfile, "Drogue deploy time:      %u ms\n", fallback_5k_time);
        fprintf(outfile, "Main deploy time:        %u ms\n", fallback_1k_time);

        fprintf(outfile, "Apogee altitude:         %.2f m\n", fallback_apogee_altitude);
        fprintf(outfile, "Drogue deploy altitude:  %.2f m\n", fallback_5k_altitude);
        fprintf(outfile, "Main deploy altitude:    %.2f m\n", fallback_1k_altitude);

        fprintf(outfile, "=================================================\n");
        fclose(outfile);
        printf("Simulation results successfully written to 'test-autosequence-flightdata.out'\n");
    } else {
        printf("Error: Could not open 'test-autosequence-flightdata.out' for writing.\n");
    }

    return 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <csv_filename> [start_row]\n", argv[0]);
        return 1;
    }

    g_csv_filename = argv[1];

    // If start_row explicitly provided, use it; otherwise auto-detect
    if (argc >= 3) {
        g_start_row = atoi(argv[2]);
    } else {
        g_start_row = find_launch_row(g_csv_filename);
    }

    printf("Starting simulation from row %d\n", g_start_row);

    execute_flight_autosequence();
    return 0;
}