/**
 * test-autosequence.c
 *
 * Full autosequence integration test - copies the exact logic from
 * autosequence-script.c with pings added for each event.
 *
 * This test validates the complete flight state machine by running
 * synthetic flight data through the autosequence logic and verifying
 * that all critical events (MECO, apogee, drogue, main, landing) are
 * detected correctly and all hardware actions (MPV energization,
 * parachute deployment) are triggered at appropriate times.
 *
 * Compile: gcc -I.. -I../helpers -I../apogee-detection-revised -o test-autosequence test-autosequence.c -lm
 * Run: ./test-autosequence
 */

#include <stdio.h>   // printf, snprintf for test output and debugging
#include <stdlib.h>  // Standard library functions
#include <stdint.h>  // Fixed-width integer types (uint32_t, etc.)
#include <math.h>    // Mathematical functions (powf, sqrtf, fabsf, logf)

// ============================================================================
// MOCKS - Must come before source includes
// These mock functions replace embedded hardware dependencies (HAL)
// to allow the autosequence to run on a desktop computer for testing.
//
// NOTE: This is a single-threaded test, so we don't need FreeRTOS semaphores
// or the full Rocket_Handle_t structure. Sensor data comes directly from
// synthetic arrays via get_sensor_data().
// ============================================================================

// Global time counter - incremented each iteration to simulate elapsed time
static uint32_t mock_time_ms = 0;

// Current sample index into the synthetic flight data arrays
static int current_sample = 0;

// Mock HAL_GetTick() - returns the simulated milliseconds since "boot"
// In real hardware, this reads from a hardware timer peripheral
uint32_t HAL_GetTick(void) { return mock_time_ms; }

// Guard macros to prevent re-including actual hardware headers
// These headers contain hardware-specific definitions we're mocking
#define MAIN_H
#define TEMP_HEADER_H


// ============================================================================
// SOURCE INCLUDES
// Include the actual implementation files being tested
// Order matters: helpers first, then functions, then test data
// ============================================================================

#include "../apogee-detection-revised/ad-helpers.h"    // Helper functions (mean, buffer checks, etc.)
#include "../apogee-detection-revised/ad-functions.c"  // Core detection algorithms
#include "../helpers/simplified-curve-fit.c"           // Quadratic curve fitting for fallback timers
#include "synthetic-flight-data.h"                     // Test input data arrays

// ============================================================================
// MOCK HARDWARE FUNCTIONS WITH PINGS
// These functions simulate hardware actions and print diagnostic output
// to trace the execution path through the state machine
// ============================================================================

// State flags tracking which hardware actions have been triggered
// Each flag is set once and checked at the end to verify correct behavior
static int mpv1_energized = 0, mpv2_energized = 0;  // Main propellant valve states
static int pilot_deployed = 0, drogue_deployed = 0, main_deployed = 0;  // Parachute states
static int abort_called = 0, low_power = 0;  // Abort and power mode flags

// Print a timestamped event notification
// Shows both sample index and simulated time for debugging
// event: Description of what happened (e.g., "MECO DETECTED")
void ping(const char* event) {
    printf("  [%3d | %6.2fs] PING: %s\n", current_sample, mock_time_ms / 1000.0f, event);
}

// Print a timestamped event with additional detail
// event: Event name
// detail: Additional context (e.g., sensor values)
void ping_detail(const char* event, const char* detail) {
    printf("  [%3d | %6.2fs] PING: %s - %s\n", current_sample, mock_time_ms / 1000.0f, event, detail);
}

// Get current simulation time - wrapper for mock time
uint32_t getTime() { return mock_time_ms; }

// Retrieve sensor data from synthetic flight data arrays
// Populates all output parameters with values for the current sample index
// b1, b2: Barometer 1 and 2 pressure readings (hPa)
// i1, i2: IMU 1 and 2 Z-axis acceleration readings (m/s^2)
// t1c, t2c: Temperature in Celsius
// t1k, t2k: Temperature in Kelvin
void get_sensor_data(float* b1, float* b2, float* i1, float* i2,
                     float* t1c, float* t2c, float* t1k, float* t2k) {
    // Read pressure values from barometer arrays
    *b1 = bar1[current_sample];
    *b2 = bar2[current_sample];

    // Read acceleration values from IMU arrays
    *i1 = imu1_z[current_sample];
    *i2 = imu2_z[current_sample];

    // Read temperature in Kelvin and convert to Celsius
    *t1k = temp1_K[current_sample];
    *t2k = temp2_K[current_sample];
    *t1c = *t1k - 273.15f;  // Kelvin to Celsius conversion
    *t2c = *t2k - 273.15f;
}

// Check if propellant valves are open (indicates ignition)
// Returns the valve state from synthetic data for current sample
int valves_open() { return valves_open_data[current_sample]; }

// Mock MPV (Main Propellant Valve) energization functions
// These are idempotent - only trigger once even if called repeatedly
void energizeMPV1() { if (!mpv1_energized) { mpv1_energized = 1; } }
void energizeMPV2() { if (!mpv2_energized) { mpv2_energized = 1; } }

// Mock parachute deployment functions with ping output
// Each deployment is idempotent to prevent double-firing
void deployPilot() { if (!pilot_deployed) { pilot_deployed = 1; ping("DEPLOY PILOT CHUTE"); } }
void deployDrogue() { if (!drogue_deployed) { drogue_deployed = 1; ping("DEPLOY DROGUE CHUTE"); } }
void deployMain() { if (!main_deployed) { main_deployed = 1; ping("DEPLOY MAIN CHUTE"); } }

// Mock abort function - sets flag and logs event
void abort_fn() { abort_called = 1; ping("ABORT"); }

// Mock low power mode - entered after landing to conserve battery
void low_power_mode() { if (!low_power) { low_power = 1; ping("LOW POWER MODE"); } }

// non operational
void wait_ms(uint32_t ms) { (void)ms; }

// Alias for wait_ms to match production code API
#define wait wait_ms

// ============================================================================
// COMPLETE AUTOSEQUENCE (copied from autosequence-script.c with pings added)
// This is the main state machine that controls the entire flight sequence
// from valve detection through landing.
// ============================================================================

const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15000; // 15 seconds
const uint32_t MAX_BURN_DURATION_MS = 22000; // 22 seconds

const uint32_t MIN_LOCKOUT_WAIT_TIME = 6000; // ms, minimum time we expect to wait in lockout phase
const uint32_t MAX_LOCKOUT_WAIT_TIME = 20000; // ms, maximum time we expect to wait in lockout phase
const uint8_t WAIT_TIME_MULTIPLIER = 3;

const uint32_t APOGEE_CONSTANT_TIMER = 100 * 1000; // 100 seconds in milliseconds
const uint32_t DROGUE_CONSTANT_TIMER = 120 * 1000; // 120 seconds in milliseconds
const uint32_t MAIN_CONSTANT_TIMER = 150 * 1000; // 150 seconds in milliseconds


// System defs
const uint32_t period = 25; // ms, greater than sampling period of 20 ms

 
int execute_flight_autosequence(){
    // starts by waiting for valves to open
    FlightPhase phase = ST_DETECT_VALVES_OPEN; 

    //const uint32_t period = pdMS_TO_TICKS(50); // 20 Hz should be good
    uint32_t last = getTime();

    // initialize detectors for pressure, accleration, and temperature
    Detector baro_detector = {0};
    Detector imu_y_detector = {0};
    Detector temp_K_detector = {0};

    // timestamps of key events, to be set when events are detected
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = 0;
    uint32_t meco_timestamp = 0;         
    uint32_t lockout_timestamp = 0; 
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;

    // altitudes of chute deployment
    float apogee_altitude = 0.0f;
    float drogue_altitude = 0.0f;
    float main_altitude = 0.0f;

    // fallback time estimates -- ALL CALCULATED WITH KINEMATICS!
    uint32_t fallback_apogee_time = 0; 
    uint32_t fallback_5k_time = 0;
    uint32_t fallback_1k_time = 0;

    uint32_t approach_mach1_timestamp = 0;

    // for velocity/acceleration estimation post lockout
    uint8_t altitude_buf_size = 0;
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0}; // in meters
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};      // in seconds
    uint8_t heights_recorded = 0;

    float ground_temp_C = 0.0f; // to be set at handoff

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
        if (should_abort()) {
            return -1;
            //return;
        }

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
                    uint8_t idx_temp = (temp_K_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    T_GROUND = temp_K_detector.average[idx_temp];

                    // set ground temp at handoff
                    uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    P_GROUND = baro_detector.average[idx_baro];

                    // adjust lapse rate based on altitude & non-standard atmospheric conditions
                    LAPSE_RATE = (T_TROPOPAUSE - T_GROUND) / (ALT_TROPOPAUSE - GROUND_ALTITUDE);

                    // calculate drogue and main deploy pressures based on ground temp and pressure
                    DROGUE_DEPLOY_PRESSURE = compute_pressure(DROGUE_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
                    MAIN_DEPLOY_PRESSURE = compute_pressure(MAIN_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
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
                #ifdef AUTOSEQUENCE_TEST
                    return 1;
                #endif
                
                // insert accel and baro readings here
                insert(&imu_y_detector, imu1_y, imu2_y, phase, IMU_DTR);

                if (!sub_to_supersonic_flag) {
                    insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                    if (detect_acceleration_spike(&imu_y_detector, &max_accel_seen)) {
                        sub_to_supersonic_flag = 1;
                        approach_mach1_timestamp = time_since(ignition_timestamp);
                    }
                }

                // check if imu detector is full of readings
                if(imu_y_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&imu_y_detector, phase) || time_since(ignition_timestamp) > MAX_BURN_DURATION_MS) {
                        MECO_flag = 1;
                        meco_timestamp = time_since(ignition_timestamp);

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

                if (time_since(ignition_timestamp) >= lockout_timestamp){
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
                    time_readings[altitude_buf_size] = time_since(ignition_timestamp) / 1000.0f; // convert ms to seconds
                    altitude_buf_size++;
                }

                // if buffer is full -> fit a quadratic curve to get estimate of velocity 
                // and acceleration, then compute fallback times
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                        &post_lockout_accel, &post_lockout_vel, &post_lockout_alt, ALTITUDE_BUFFER_SIZE);

                    heights_recorded = 1;
                    fallback_apogee_time = time_since(ignition_timestamp);
                    fallback_5k_time = time_since(ignition_timestamp);
                    fallback_1k_time = time_since(ignition_timestamp);
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
                        apogee_timestamp = time_since(ignition_timestamp);
                        
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        apogee_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    // if our fallback timers predict apogee, set flag and move to next phase
                    else if (time_since(ignition_timestamp) >= fallback_apogee_time) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        fallback_timers_worked = 1;
                        apogee_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        apogee_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    // if neither work, set constant timer flag
                    else if (time_since(ignition_timestamp) >= APOGEE_CONSTANT_TIMER) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        apogee_altitude = compute_height(baro_detector.avg_index[idx_baro]);
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
                        drogue_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        drogue_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    // if we detected apogee with fallback timers, detect drogue deployment with fallback timers
                    else if (fallback_timers_worked && time_since(ignition_timestamp) >= fallback_5k_time) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                        drogue_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        drogue_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    // otherwise just "detect" with constant timer
                    else if (time_since(ignition_timestamp) >= DROGUE_CONSTANT_TIMER) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                        drogue_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        drogue_altitude = compute_height(baro_detector.avg_index[idx_baro]);
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
                        phase = ST_WAIT_GROUND;
                        main_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    else if (fallback_timers_worked && time_since(ignition_timestamp) >= fallback_1k_time) {
                        main_flag = 1;
                        phase = ST_WAIT_GROUND;
                        main_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = compute_height(baro_detector.avg_index[idx_baro]);
                    }

                    else if (time_since(ignition_timestamp) >= MAIN_CONSTANT_TIMER) {
                        main_flag = 1;
                        phase = ST_WAIT_GROUND;
                        main_timestamp = time_since(ignition_timestamp);

                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        main_altitude = compute_height(baro_detector.avg_index[idx_baro]);
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
        vTaskDelayUntil(&last, period);
    }
}
