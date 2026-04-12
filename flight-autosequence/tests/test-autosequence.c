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
// TEST CONFIGURATION CONSTANTS
// These values are scaled/adjusted for the test environment
// ============================================================================

// Number of altitude samples to collect for curve fitting after lockout
#define ALTITUDE_BUFFER_SIZE 11

// Fallback constant timers (milliseconds) - used if detection fails
// These provide guaranteed deployment even if sensors fail
const uint32_t APOGEE_CONSTANT_TIMER = 100 * 1000;   // 100 seconds
const uint32_t DROGUE_CONSTANT_TIMER = 120 * 1000;   // 120 seconds
const uint32_t MAIN_CONSTANT_TIMER = 150 * 1000;     // 150 seconds

// Maximum time to wait for valves to open after handoff before aborting
const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15000;  // 15 seconds

// Maximum expected burn duration - triggers MECO detection if exceeded
const uint32_t MAX_BURN_DURATION_MS = 22000;  // 22 seconds

// Mach lockout timing bounds (scaled down for faster test execution)
// These prevent premature apogee detection during transonic flight
const uint32_t MIN_LOCKOUT_WAIT_TIME = 500;   // 500ms minimum lockout
const uint32_t MAX_LOCKOUT_WAIT_TIME = 2000;  // 2000ms maximum lockout

// Multiplier for calculating lockout duration based on supersonic flight time
const int WAIT_TIME_MULTIPLIER = 2;

// Sampling rate configuration (must match synthetic data timing)
const uint32_t sampling_frequency = 50;  // 20 Hz sampling rate
const uint32_t period = 20;              // 50ms between samples

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

void execute_flight_autosequence_test() {
    //COPIED FROM SCRIPT 1-1
    
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
    float fallback_apogee_time = 0;
    float fallback_5k_time = 0;
    float fallback_1k_time = 0;

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
    float imu1_z = 0.0f;
    float imu2_z = 0.0f;

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

    // ========================================================================
    // ATMOSPHERIC MODEL INITIALIZATION
    // Set standard atmosphere values - will be updated with actual ground
    // measurements when valves open
    // ========================================================================
    P_GROUND = 1013.25f;     // Ground pressure (hPa) - standard sea level
    T_GROUND = 293.0f;       // Ground temperature (K) - approximately 20C
    GROUND_ALTITUDE = 0.0f;  // Ground altitude (m) - sea level reference
    LAPSE_RATE = 0.0065f;    // Temperature lapse rate (K/m) - troposphere

    // ========================================================================
    // TEST OUTPUT HEADER
    // ========================================================================
    printf("\n============================================================\n");
    printf("  AUTOSEQUENCE TEST - Full State Machine\n");
    printf("  %d samples @ %d Hz (%.1fs simulated)\n", DATA_SIZE, SAMPLE_RATE_HZ,
           DATA_SIZE * SAMPLE_PERIOD_MS / 1000.0f);
    printf("============================================================\n\n");

    ping("AUTOSEQUENCE START");

    // ========================================================================
    // MAIN LOOP
    // Runs for DATA_SIZE iterations (instead of infinite loop in production)
    // Each iteration processes one sample of synthetic flight data
    // ========================================================================
    for (current_sample = 0; current_sample < DATA_SIZE; current_sample++) {
        // Update simulated time based on sample index and period
        mock_time_ms = current_sample * SAMPLE_PERIOD_MS;
        // hopefully these are correct
        // ====================================================================
        // EDGE CASE ANNOTATIONS
        // Print warnings when synthetic data contains intentional anomalies
        // These test the detector's robustness to sensor glitches
        // ====================================================================
        // NOTE: for this, i multiplied it by 2.5 bc the frequency increased
        if (current_sample == 75) printf("  [EDGE CASE] Acceleration spike in IMU1\n");
        if (current_sample == 100) printf("  [EDGE CASE] Mach transition dip\n");
        if (current_sample == 210) printf("  [EDGE CASE] Pressure spike during lockout\n");
        if (current_sample == 300) printf("  [EDGE CASE] Flat apogee region begins\n");
        if (current_sample == 400) printf("  [EDGE CASE] Barometer 1 dropout (160-162)\n");

        // get sensor data at the beginning of each loop iteration
        get_sensor_data(&bar1, &bar2,
                        &imu1_z, &imu2_z,
                        &bar1_temp_C, &bar2_temp_C,
                        &bar1_temp_K, &bar2_temp_K);

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

                    LAPSE_RATE = (T_TROPOPAUSE - T_GROUND) / (ALT_TROPOPAUSE - GROUND_ALTITUDE);

                    // TEST ONLY: Log the computed atmospheric parameters
                    char detail[100];
                    snprintf(detail, sizeof(detail), "T=%.1fK, P=%.1f hPa, Drogue@%.1f, Main@%.1f",
                             T_GROUND, P_GROUND, DROGUE_DEPLOY_PRESSURE, MAIN_DEPLOY_PRESSURE);
                    ping_detail("VALVES OPEN / IGNITION", detail);
                }

                // if we wait too long without valves opening, ABORT!
                else if (getTime() - handoff_timestamp > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    abort_fn();
                    return;
                }
            }

            // enters at launch, waiting for main engine cutoff
            case ST_WAIT_MECO: {
                // energize MPVs while in this phase
                energizeMPV1();
                energizeMPV2();

                // insert accel and baro readings here
                insert(&imu_z_detector, imu1_z, imu2_z, phase, IMU_DTR);

                if (!sub_to_supersonic_flag) {
                    insert(&baro_detector, bar1, bar2, phase, BARO_DTR);
                    if (detect_acceleration_spike(&imu_z_detector, max_accel_seen)) {
                        sub_to_supersonic_flag = 1;
                        approach_mach1_timestamp = getTime();
                        ping("MACH 1 TRANSITION DETECTED");  // TEST ONLY
                    }
                }

                // check if imu detector is full of readings
                if(imu_z_detector.avg_size >= AD_CAPACITY) {
                    // check if MECO detected - when accel becomes negative
                    if (detect_event(&imu_z_detector, phase) || getTime() - ignition_timestamp > MAX_BURN_DURATION_MS) {
                        MECO_flag = 1;
                        meco_timestamp = getTime();

                        // TEST ONLY: Log MECO
                        char detail[50];
                        float avg = imu_z_detector.average[(imu_z_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY];
                        snprintf(detail, sizeof(detail), "Avg accel: %.2f m/s^2", avg);
                        ping_detail("MECO DETECTED", detail);

                        if (sub_to_supersonic_flag){
                            phase = ST_MACH_LOCKOUT;

                            // set lockout timer based on how long we were in boost phase after approaching mach 1
                            uint32_t wait = (meco_timestamp - approach_mach1_timestamp) * WAIT_TIME_MULTIPLIER;
                            if (wait < MIN_LOCKOUT_WAIT_TIME)
                                wait = MIN_LOCKOUT_WAIT_TIME;
                            else if (wait > MAX_LOCKOUT_WAIT_TIME)
                                wait = MAX_LOCKOUT_WAIT_TIME;

                            lockout_timestamp = meco_timestamp + wait;

                            // TEST ONLY
                            char lockout_detail[50];
                            snprintf(lockout_detail, sizeof(lockout_detail), "Duration: %u ms", (unsigned)wait);
                            ping_detail("ENTERING MACH LOCKOUT", lockout_detail);
                        }

                        else {
                            phase = ST_WAIT_APOGEE;
                            ping("SKIPPING LOCKOUT (subsonic flight)");  // TEST ONLY
                        }

                    }
                }

                break;
            }

            case ST_MACH_LOCKOUT: {
                // Simply wait until lockout timer expires
                // No barometer readings taken during this phase
                if (getTime() >= lockout_timestamp) {
                    phase = ST_WAIT_APOGEE;
                    ping("LOCKOUT COMPLETE - Barometer reliable");
                }

                break;
            }

            // after lockout, waiting for apogee detection
            case ST_WAIT_APOGEE: {
                // insert pressure reading
                insert(&baro_detector, bar1, bar2, phase, BARO_DTR);

                // insert altitude readings into buffer once
                if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) {
                    float idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                    altitude_readings[altitude_buf_size] = compute_height(baro_detector.average[(int)idx_baro]);
                    time_readings[altitude_buf_size] = getTime() / 1000.0f; // convert ms to seconds
                    wait(50); // wait 50 ms for next reading
                    altitude_buf_size++;
                }

                // if buffer is full -> fit a quadratic curve to get estimate of velocity
                // and acceleration, then compute fallback times
                else if (!heights_recorded) {
                    quadr_curve_fit(altitude_readings, time_readings,
                         &post_lockout_accel, &post_lockout_vel, &post_lockout_alt);

                    heights_recorded = 1;
                    compute_fallback_times(post_lockout_alt, post_lockout_vel, post_lockout_accel,
                                        &fallback_apogee_time, &fallback_5k_time, &fallback_1k_time);

                    // TEST ONLY
                    char detail[100];
                    snprintf(detail, sizeof(detail), "alt=%.0fm, vel=%.1fm/s, accel=%.1fm/s^2",
                             post_lockout_alt, post_lockout_vel, post_lockout_accel);
                    ping_detail("FALLBACK TIMERS COMPUTED", detail);
                }

                // check if apogee detected with barometer detector, if we have enough barometer readings
                if(baro_detector.slope_size >= AD_CAPACITY) {
                    // if we detect apogee with our detector, set a flag and move to next phase
                    if (detect_event(&baro_detector, phase)) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        apogee_detection_worked = 1;

                        // TEST ONLY
                        int idx = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        float pressure = baro_detector.average[idx];
                        float altitude = compute_height(pressure);
                        char detail[50];
                        snprintf(detail, sizeof(detail), "P=%.1f hPa, Alt=%.0fm", pressure, altitude);
                        ping_detail("APOGEE DETECTED (Barometer)", detail);
                    }

                    // if our fallback timers predict apogee, set flag and move to next phase
                    else if (getTime() >= fallback_apogee_time) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        fallback_timers_worked = 1;
                        ping("APOGEE DETECTED (Fallback timer)");  // TEST ONLY
                    }

                    // if neither work, set constant timer flag
                    else if (getTime() >= APOGEE_CONSTANT_TIMER) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        ping("APOGEE DETECTED (Constant timer)");  // TEST ONLY
                    }
                }

                break;
            }

        
            // Descending from apogee - deploy pilot chute and wait for 5km
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

                        // TEST ONLY
                        int idx = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        float pressure = baro_detector.average[idx];
                        char detail[50];
                        snprintf(detail, sizeof(detail), "P=%.1f hPa (threshold=%.1f)", pressure, DROGUE_DEPLOY_PRESSURE);
                        ping_detail("DROGUE ALTITUDE REACHED", detail);
                    }

                    // if we detected apogee with fallback timers, detect drogue deployment with fallback timers
                    else if (fallback_timers_worked && getTime() >= fallback_5k_time) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                        ping("DROGUE ALTITUDE REACHED (Fallback timer)");  // TEST ONLY
                    }

                    // otherwise just "detect" with constant timer
                    else if (getTime() >= DROGUE_CONSTANT_TIMER) {
                        drogue_flag = 1;
                        phase = ST_WAIT_MAIN;
                        ping("DROGUE ALTITUDE REACHED (Constant timer)");  // TEST ONLY
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
                        phase = ST_WAIT_GROUND;  // NOTE: Production goes to ST_DONE here (bug)

                        // TEST ONLY
                        int idx = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        float pressure = baro_detector.average[idx];
                        char detail[50];
                        snprintf(detail, sizeof(detail), "P=%.1f hPa (threshold=%.1f)", pressure, MAIN_DEPLOY_PRESSURE);
                        ping_detail("MAIN ALTITUDE REACHED", detail);
                    }

                    else if (fallback_timers_worked && getTime() >= fallback_1k_time) {
                        main_flag = 1;
                        phase = ST_WAIT_GROUND;  // NOTE: Production goes to ST_DONE here (bug)
                        ping("MAIN ALTITUDE REACHED (Fallback timer)");  // TEST ONLY
                    }

                    else if (getTime() >= MAIN_CONSTANT_TIMER) {
                        main_flag = 1;
                        phase = ST_WAIT_GROUND;  // NOTE: Production goes to ST_DONE here (bug)
                        ping("MAIN ALTITUDE REACHED (Constant timer)");  // TEST ONLY
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
                        ping("LANDING DETECTED");  // TEST ONLY
                    }
                }


                break;
            }


            case ST_DONE: {
                // Enter low power mode once on ground
                // This extends battery life for recovery beacon
                low_power_mode();
                break;
            }

            // Default case - should never be reached
            default:
                break;

        } // end switch(phase)
        // printout names everytime there's a new stage
        if (current_sample % 50 == 0 && current_sample > 0) {
            // Human-readable phase names for debugging output
            const char* phase_names[] = {
                "DETECT_VALVES_OPEN", "MACH_LOCKOUT", "WAIT_MECO",
                "WAIT_APOGEE", "WAIT_DROGUE", "WAIT_MAIN", "WAIT_GROUND", "DONE"
            };
            printf("  ... [%d/%d] Phase: %s\n", current_sample, DATA_SIZE, phase_names[phase]);
        }
    }

    // ========================================================================
    // TEST RESULTS SUMMARY
    // Print final status of all event flags and hardware actions
    // ========================================================================
    printf("\n============================================================\n");
    printf("  RESULTS\n");
    printf("============================================================\n\n");

    // Print event detection status
    printf("  Events Detected:\n");
    printf("    MECO:     %s\n", MECO_flag ? "YES" : "NO");
    printf("    Apogee:   %s (%s)\n", apogee_flag ? "YES" : "NO",
           apogee_detection_worked ? "Barometer" : (fallback_timers_worked ? "Fallback" : "Constant"));
    printf("    Drogue:   %s\n", drogue_flag ? "YES" : "NO");
    printf("    Main:     %s\n", main_flag ? "YES" : "NO");
    printf("    Landed:   %s\n", landed_flag ? "YES" : "NO");

    // Print hardware action status
    printf("\n  Hardware Actions:\n");
    printf("    MPV1:     %s\n", mpv1_energized ? "Energized" : "Not energized");
    printf("    MPV2:     %s\n", mpv2_energized ? "Energized" : "Not energized");
    printf("    Pilot:    %s\n", pilot_deployed ? "Deployed" : "Not deployed");
    printf("    Drogue:   %s\n", drogue_deployed ? "Deployed" : "Not deployed");
    printf("    Main:     %s\n", main_deployed ? "Deployed" : "Not deployed");
    printf("    LowPower: %s\n", low_power ? "Entered" : "Not entered");

    // ========================================================================
    // FINAL PASS/FAIL DETERMINATION
    // All critical events must be detected and all parachutes deployed
    // Abort must NOT have been called for a passing test
    // ========================================================================
    int pass = MECO_flag && apogee_flag && drogue_flag && main_flag && landed_flag &&
               pilot_deployed && drogue_deployed && main_deployed && !abort_called;

    printf("\n============================================================\n");
    printf("  %s\n", pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    printf("============================================================\n\n");
}
// main
int main() {
    execute_flight_autosequence_test();
    return 0;
}
