/**
 * test-autosequence.c
 *
 * Integration test mirroring production autosequence/autosequence-script.c.
 * Runs synthetic flight data through the exact same state machine logic
 * with mock hardware functions and Fluctus responses.
 *
 * Compile: gcc -I.. -I../apogee-detection-revised -o test-autosequence test-autosequence.c -lm
 * Run: ./test-autosequence
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// ============================================================================
// MOCK GUARDS — prevent real embedded headers from being included
// ============================================================================
#define MAIN_H
#define TEMP_HEADER_H

// ============================================================================
// CONFIGURATION — must be defined before source includes
// ============================================================================
#define ALTITUDE_BUFFER_SIZE 31
#define ALT_BUF_ELT_SPACING 3
#define UNUSED(x) (void)(x)

// ============================================================================
// MOCK GLOBAL STATE
// ============================================================================
static uint32_t mock_time_ms = 0;
static int current_sample = 0;

// ============================================================================
// SOURCE INCLUDES — pulls in Detector, FlightPhase, detection algorithms
// Order matters: helpers first (defines mean, buffer_lt, etc.),
// then functions (defines insert, detect_event, etc.)
// ============================================================================
#include "../apogee-detection-revised/ad-helpers.c"
#include "../apogee-detection-revised/ad-functions.c"
#ifndef FLIGHT_DATA_FILE
#define FLIGHT_DATA_FILE "synthetic-flight-data.h"
#endif
#include FLIGHT_DATA_FILE

// ============================================================================
// TYPE DEFINITIONS — mirrors production flight-autosequence.h
// ============================================================================
typedef struct {
    float XL_x;
    float XL_y; // y direction is axial to the rocket
    float XL_z;
    float W_x;
    float W_y;
    float W_z;
} IMU_values;

typedef struct {
    FlightPhase phase;
    uint32_t current_time_in_flight;

    uint32_t fallback_apogee_time;
    uint32_t fallback_5k_time;
    uint32_t fallback_1k_time;

    uint8_t apogee_detection_worked;
    uint8_t fallback_timers_worked;
    uint8_t constant_timers_worked;

    uint8_t fluctus_apogee_detected;
    uint32_t fluctus_apogee_timestamp;
    uint8_t fluctus_disabled;
    uint8_t heights_recorded;

    float main_deploy_pressure;
    float drogue_deploy_pressure;
} Autos_boot_t;

// ============================================================================
// TIMING CONSTANTS — adjusted for synthetic data (10s flight, 200 samples)
// Production values are too large for a 10-second test flight
// ============================================================================
const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15 * 1000;
const uint32_t LOCKOUT_END_TIME = 4 * 1000;

const uint32_t APOGEE_CONSTANT_TIMER  = 8 * 1000;
const uint32_t DROGUE_CONSTANT_TIMER  = 15 * 1000;
const uint32_t MAIN_CONSTANT_TIMER    = 18 * 1000;

const uint32_t APOGEE_AGREEMENT_WINDOW = 1 * 1000;
const uint32_t period = 50; // matches SAMPLE_PERIOD_MS

// ============================================================================
// PHASE NAME STRINGS — for test output
// ============================================================================
static const char* phase_names[] = {
    [ST_DISARMED]           = "DISARMED",
    [ST_DETECT_VALVES_OPEN] = "DETECT_VALVES_OPEN",
    [ST_WAIT_MECO]          = "WAIT_MECO",
    [ST_MACH_LOCKOUT]       = "MACH_LOCKOUT",
    [ST_WAIT_APOGEE]        = "WAIT_APOGEE",
    [ST_WAIT_DROGUE]        = "WAIT_DROGUE",
    [ST_WAIT_MAIN]          = "WAIT_MAIN",
    [ST_WAIT_GROUND]        = "WAIT_GROUND",
    [ST_DONE]               = "DONE"
};

// ============================================================================
// MOCK HARDWARE FUNCTIONS
// ============================================================================

void ping(const char* event) {
    printf("  [%3d | %6.2fs] %s\n", current_sample, mock_time_ms / 1000.0f, event);
}

void ping_detail(const char* event, float value) {
    printf("  [%3d | %6.2fs] %s (%.2f)\n", current_sample, mock_time_ms / 1000.0f, event, value);
}

uint32_t getTime(void) { return mock_time_ms; }
uint32_t time_since(uint32_t t) { return mock_time_ms - t; }

void get_sensor_data(float* b1, float* b2,
                     IMU_values* i1, IMU_values* i2,
                     float* t1c, float* t2c) {
    *b1 = bar1[current_sample];
    *b2 = bar2[current_sample];

    // Synthetic data uses z-axis as axial; production uses y-axis
    i1->XL_x = 0; i1->XL_y = imu1_z[current_sample]; i1->XL_z = 0;
    i1->W_x = 0;  i1->W_y = 0;                       i1->W_z = 0;
    i2->XL_x = 0; i2->XL_y = imu2_z[current_sample]; i2->XL_z = 0;
    i2->W_x = 0;  i2->W_y = 0;                       i2->W_z = 0;

    *t1c = temp1_K[current_sample] - 273.15f;
    *t2c = temp2_K[current_sample] - 273.15f;
}

uint8_t valves_open(void) { return valves_open_data[current_sample]; }
uint8_t should_abort(void) { return 0; }

// ============================================================================
// MOCK FLUCTUS — configurable trigger samples
// Set to -1 to disable a particular Fluctus event
// ============================================================================
#define FLUCTUS_APOGEE_SAMPLE 125
#define FLUCTUS_5K_SAMPLE     155
#define FLUCTUS_1K_SAMPLE     160

uint8_t get_fluctus_apogee(void) { return current_sample >= FLUCTUS_APOGEE_SAMPLE; }
uint8_t get_fluctus_5k(void)     { return current_sample >= FLUCTUS_5K_SAMPLE; }
uint8_t get_fluctus_1k(void)     { return current_sample >= FLUCTUS_1K_SAMPLE; }

// ============================================================================
// MOCK DEPLOYMENT — idempotent with ping on first call
// ============================================================================
static int pilot_deployed = 0, drogue_deployed = 0, main_deployed = 0;

void deployPilot(void) {
    if (!pilot_deployed) { pilot_deployed = 1; ping("DEPLOY PILOT CHUTE"); }
}
void deployDrogue(void) {
    if (!drogue_deployed) { drogue_deployed = 1; ping("DEPLOY DROGUE CHUTE"); }
}
void deployMain(void) {
    if (!main_deployed) { main_deployed = 1; ping("DEPLOY MAIN CHUTE"); }
}

// ============================================================================
// MOCK RTOS / TELEMETRY — no-ops for test
// ============================================================================
void update_state_in_telem(FlightPhase s) { (void)s; }
void update_boot_params(Autos_boot_t *p) { (void)p; }

#define vTaskDelayUntil(last_ptr, pd) do { \
    current_sample++;                       \
    mock_time_ms = (uint32_t)(current_sample * SAMPLE_PERIOD_MS); \
} while(0)

// ============================================================================
// AUTOSEQUENCE STATE MACHINE
// Mirrors production autosequence/autosequence-script.c exactly,
// with pings added at state transitions for test visibility.
// ============================================================================

int execute_flight_autosequence(Autos_boot_t boot_params) {
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
    uint32_t last = getTime();

    // initialize detectors for pressure, acceleration, and temperature
    Detector baro_detector = {0};
    Detector imu_y_detector = {0};
    Detector temp_C_detector = {0};

    // timestamps of key events
    uint32_t handoff_timestamp = 0;
    uint32_t ignition_timestamp = (boot_params.phase > ST_DETECT_VALVES_OPEN)
        ? getTime() - boot_params.current_time_in_flight : 0;
    uint32_t meco_timestamp = 0;
    uint32_t apogee_timestamp = 0;
    uint32_t drogue_timestamp = 0;
    uint32_t main_timestamp = 0;
    uint32_t landed_timestamp = 0;

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
    float altitude_readings[ALTITUDE_BUFFER_SIZE] = {0};
    float time_readings[ALTITUDE_BUFFER_SIZE] = {0};
    uint8_t heights_recorded = 0;
    int loop_ctr = 0;

    float bar1_val = 0.0f;
    float bar2_val = 0.0f;
    float imu1_y = 0.0f;
    float imu2_y = 0.0f;

    IMU_values imu1_vals = {0};
    IMU_values imu2_vals = {0};

    float bar1_temp_C = 0.0f;
    float bar2_temp_C = 0.0f;

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

    UNUSED(MECO_flag);
    UNUSED(apogee_flag);
    UNUSED(landed_flag);
    UNUSED(fluctus_5k_detected);
    UNUSED(fluctus_1k_detected);

    // Fluctus FC detection timers
    uint8_t override_calc_fallback_timers = 0;
    uint32_t fluctus_apogee_timestamp = 0;

    if (boot_params.phase > ST_DETECT_VALVES_OPEN) {
        fallback_apogee_time     = boot_params.fallback_apogee_time;
        fallback_5k_time         = boot_params.fallback_5k_time;
        fallback_1k_time         = boot_params.fallback_1k_time;
        apogee_detection_worked  = boot_params.apogee_detection_worked;
        fallback_timers_worked   = boot_params.fallback_timers_worked;
        constant_timers_worked   = boot_params.constant_timers_worked;
        fluctus_apogee_detected  = boot_params.fluctus_apogee_detected;
        fluctus_apogee_timestamp = boot_params.fluctus_apogee_timestamp;
        heights_recorded         = boot_params.heights_recorded;
        MAIN_DEPLOY_PRESSURE     = boot_params.main_deploy_pressure;
        DROGUE_DEPLOY_PRESSURE   = boot_params.drogue_deploy_pressure;
    }

    ping("AUTOSEQUENCE START");

    uint32_t num_cycles = 0;
    FlightPhase prev_phase = phase;
    for (;;) {
        if (should_abort()) {
            return -1;
        }
        // TEST: end-of-data check
        if (current_sample >= DATA_SIZE) {
            printf("\n  End of synthetic data at sample %d.\n", current_sample);
            break;
        }

        update_state_in_telem(phase);
        update_boot_params(&boot_params);

        // Print phase transitions
        if (phase != prev_phase) {
            char buf[80];
            snprintf(buf, sizeof(buf), "PHASE: %s -> %s",
                     phase_names[prev_phase], phase_names[phase]);
            ping(buf);
            prev_phase = phase;
        }

        // get sensor data
        get_sensor_data(&bar1_val, &bar2_val,
                        &imu1_vals, &imu2_vals,
                        &bar1_temp_C, &bar2_temp_C);
        imu1_y = imu1_vals.XL_y;
        imu2_y = imu2_vals.XL_y;

        switch (phase) {
            case ST_DETECT_VALVES_OPEN: {
                insert(&temp_C_detector, bar1_temp_C, bar2_temp_C, phase, TEMP_DTR);
                insert(&baro_detector, bar1_val, bar2_val, phase, BARO_DTR);

                if (valves_open()) {
                    phase = ST_WAIT_MECO;
                    boot_params.phase = phase;
                    ignition_timestamp = getTime();

                    if (temp_C_detector.avg_size >= AD_CAPACITY) {
                        T_GROUND = mean(temp_C_detector.avg_size, temp_C_detector.average) + 273.15f;
                    } else {
                        uint8_t idx_temp = (temp_C_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        T_GROUND = temp_C_detector.average[idx_temp] + 273.15f;
                    }

                    if (baro_detector.avg_size >= AD_CAPACITY) {
                        P_GROUND = mean(baro_detector.avg_size, baro_detector.average);
                    } else {
                        uint8_t idx_baro = (baro_detector.avg_index - 1 + AD_CAPACITY) % AD_CAPACITY;
                        P_GROUND = baro_detector.average[idx_baro];
                    }

                    DROGUE_DEPLOY_PRESSURE = compute_pressure(DROGUE_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);
                    MAIN_DEPLOY_PRESSURE = compute_pressure(MAIN_DEPLOY_ALTITUDE, T_GROUND, P_GROUND);

                    boot_params.drogue_deploy_pressure = DROGUE_DEPLOY_PRESSURE;
                    boot_params.main_deploy_pressure = MAIN_DEPLOY_PRESSURE;
                    clear(&baro_detector);

                    ping("VALVES OPEN — ignition");
                    ping_detail("  T_GROUND (K)", T_GROUND);
                    ping_detail("  P_GROUND (hPa)", P_GROUND);
                    ping_detail("  DROGUE_DEPLOY_PRESSURE (hPa)", DROGUE_DEPLOY_PRESSURE);
                    ping_detail("  MAIN_DEPLOY_PRESSURE (hPa)", MAIN_DEPLOY_PRESSURE);
                }
                else if (time_since(handoff_timestamp) > MAX_HANDOFF_TO_VALVE_OPEN_MS) {
                    ping("ABORT — valve timeout");
                    return -2;
                }
                break;
            }

            case ST_WAIT_MECO: {
                insert(&imu_y_detector, imu1_y, imu2_y, phase, IMU_DTR);

                if (imu_y_detector.avg_size >= AD_CAPACITY) {
                    if (detect_event(&imu_y_detector, phase)) {
                        MECO_flag = 1;
                        meco_timestamp = time_since(ignition_timestamp);
                        phase = ST_MACH_LOCKOUT;
                        boot_params.phase = phase;
                        clear(&imu_y_detector);
                        ping_detail("MECO DETECTED — entering lockout", (float)meco_timestamp);
                    }
                    else if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME) {
                        phase = ST_WAIT_APOGEE;
                        boot_params.phase = phase;
                        clear(&imu_y_detector);
                        ping("MECO TIMEOUT — skipping to apogee detection");
                    }
                }

                if (get_fluctus_apogee() && !fluctus_disabled) {
                    fluctus_apogee_detected = 1;
                    override_calc_fallback_timers = 1;
                    boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                    fluctus_apogee_timestamp = time_since(ignition_timestamp);
                    boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;
                    phase = ST_WAIT_APOGEE;
                    boot_params.phase = phase;
                    ping("FLUCTUS APOGEE during MECO phase — jumping to apogee");
                }
                else {
                    override_calc_fallback_timers = 0;
                }
                break;
            }

            case ST_MACH_LOCKOUT: {
                if (imu_y_detector.avg_size >= AD_CAPACITY) {
                    if (time_since(ignition_timestamp) >= LOCKOUT_END_TIME) {
                        phase = ST_WAIT_APOGEE;
                        boot_params.phase = phase;

                        heights_recorded = 0;
                        boot_params.heights_recorded = heights_recorded;

                        fluctus_apogee_detected = 0;
                        boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                        ping("LOCKOUT END — normal exit");
                    }
                }

                if (get_fluctus_apogee() && !fluctus_disabled) {
                    fluctus_apogee_detected = 1;
                    override_calc_fallback_timers = 1;
                    boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                    fluctus_apogee_timestamp = time_since(ignition_timestamp);
                    boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;
                    phase = ST_WAIT_APOGEE;
                    boot_params.phase = phase;
                    ping("FLUCTUS APOGEE during LOCKOUT — jumping to apogee");
                }
                else {
                    override_calc_fallback_timers = 0;
                }
                break;
            }

            case ST_WAIT_APOGEE: {
                insert(&baro_detector, bar1_val, bar2_val, phase, BARO_DTR);

                if (!fluctus_apogee_detected) {
                    if (get_fluctus_apogee()) {
                        fluctus_apogee_detected = 1;
                        boot_params.fluctus_apogee_detected = fluctus_apogee_detected;
                        fluctus_apogee_timestamp = time_since(ignition_timestamp);
                        boot_params.fluctus_apogee_timestamp = fluctus_apogee_timestamp;
                        ping("FLUCTUS APOGEE detected during WAIT_APOGEE");
                    }
                }

                if (!override_calc_fallback_timers) {
                    if (altitude_buf_size < ALTITUDE_BUFFER_SIZE) {
                        if (loop_ctr % ALT_BUF_ELT_SPACING == 0) {
                            altitude_readings[altitude_buf_size] = compute_height((bar1_val + bar2_val) / 2.0f);
                            time_readings[altitude_buf_size] = time_since(ignition_timestamp) / 1000.0f;
                            altitude_buf_size++;
                        }
                    }
                    else if (!heights_recorded) {
                        quadr_curve_fit(altitude_readings, time_readings,
                            &post_lockout_accel, &post_lockout_vel, &post_lockout_alt, ALTITUDE_BUFFER_SIZE);

                        if (post_lockout_accel < 0) {
                            approximated_accel = -sqrtf(post_lockout_accel * -G);
                        }
                        else {
                            approximated_accel = -0.1f;
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

                        ping("FALLBACK TIMERS COMPUTED");
                        ping_detail("  altitude (m)", post_lockout_alt);
                        ping_detail("  velocity (m/s)", post_lockout_vel);
                        ping_detail("  accel (m/s^2)", approximated_accel);
                        ping_detail("  fallback apogee (ms)", (float)fallback_apogee_time);
                        ping_detail("  fallback 5k (ms)", (float)fallback_5k_time);
                        ping_detail("  fallback 1k (ms)", (float)fallback_1k_time);
                    }

                    else if (baro_detector.avg_size >= AD_CAPACITY) {
                        uint32_t current_time = time_since(ignition_timestamp);
                        uint8_t apogee_by_detection = detect_event(&baro_detector, phase);
                        uint8_t apogee_by_fallback = current_time >= fallback_apogee_time;
                        uint8_t apogee_by_constant_timer = current_time >= APOGEE_CONSTANT_TIMER;

                        uint8_t script_apogee_detected = apogee_by_detection || apogee_by_fallback || apogee_by_constant_timer;
                        uint8_t fluctus_apogee_correct = fluctus_apogee_detected && !fluctus_disabled;
                        uint8_t fluctus_apogee_agreement = script_apogee_detected && fluctus_apogee_correct;

                        uint8_t fluctus_override = fluctus_apogee_correct &&
                            (time_since(ignition_timestamp) - fluctus_apogee_timestamp) > APOGEE_AGREEMENT_WINDOW;
                        uint8_t script_detection_only = script_apogee_detected && fluctus_disabled;

                        if (fluctus_apogee_agreement || fluctus_override || script_detection_only) {
                            apogee_flag = 1;
                            phase = ST_WAIT_DROGUE;
                            boot_params.phase = phase;
                            apogee_timestamp = current_time;
                            apogee_altitude = compute_height(mean(baro_detector.avg_size, baro_detector.average));

                            apogee_detection_worked = 0;
                            fallback_timers_worked = 0;
                            constant_timers_worked = 0;

                            if (apogee_by_detection) {
                                apogee_detection_worked = 1;
                                ping("APOGEE — barometer detection");
                            }
                            else if (apogee_by_fallback) {
                                fallback_timers_worked = 1;
                                ping("APOGEE — fallback timer");
                            }
                            else if (apogee_by_constant_timer) {
                                constant_timers_worked = 1;
                                ping("APOGEE — constant timer");
                            }

                            boot_params.apogee_detection_worked = apogee_detection_worked;
                            boot_params.fallback_timers_worked = fallback_timers_worked;
                            boot_params.constant_timers_worked = constant_timers_worked;

                            ping_detail("  apogee altitude (m)", apogee_altitude);
                            ping_detail("  apogee time (ms)", (float)apogee_timestamp);
                        }
                    }
                }

                else {
                    // override_calc_fallback_timers is set — Fluctus detected apogee
                    // during MECO/LOCKOUT, skip curve fitting, just wait for agreement window
                    uint32_t current_time = time_since(ignition_timestamp);
                    if ((current_time - fluctus_apogee_timestamp) > APOGEE_AGREEMENT_WINDOW) {
                        apogee_flag = 1;
                        phase = ST_WAIT_DROGUE;
                        boot_params.phase = phase;
                        apogee_timestamp = current_time;
                        apogee_altitude = compute_height(mean(baro_detector.avg_size, baro_detector.average));

                        apogee_detection_worked = 0;
                        fallback_timers_worked = 0;
                        constant_timers_worked = 0;

                        boot_params.apogee_detection_worked = apogee_detection_worked;
                        boot_params.fallback_timers_worked = fallback_timers_worked;
                        boot_params.constant_timers_worked = constant_timers_worked;

                        ping("APOGEE — Fluctus override (agreement window expired)");
                        ping_detail("  apogee altitude (m)", apogee_altitude);
                        ping_detail("  apogee time (ms)", (float)apogee_timestamp);
                    }
                }
                loop_ctr++;
                break;
            }

            case ST_WAIT_DROGUE: {
                deployPilot();
                insert(&baro_detector, bar1_val, bar2_val, phase, BARO_DTR);

                if (baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp);

                    uint8_t fiveK_by_detection = detect_event(&baro_detector, phase);
                    uint8_t fiveK_by_fallback = current_time >= fallback_5k_time;
                    uint8_t fiveK_by_constant_timer = current_time >= DROGUE_CONSTANT_TIMER;

                    // TIER 1: If apogee logic worked, strictly lock out ALL timers.
                    if (apogee_detection_worked) {
                        if (fiveK_by_detection) {
                            drogue_flag = 1;
                            ping("DROGUE — TIER 1 barometer detection");
                        }
                    }
                    // TIER 2: Apogee failed, so we fall back to computed timers.
                    else if (fallback_timers_worked) {
                        if (fiveK_by_fallback) {
                            drogue_flag = 1;
                            ping("DROGUE — TIER 2 fallback timer");
                        }
                    }
                    // TIER 3: Both primary and secondary failed.
                    else if (constant_timers_worked) {
                        if (fiveK_by_constant_timer) {
                            drogue_flag = 1;
                            ping("DROGUE — TIER 3 constant timer");
                        }
                    }
                    // TIER 4: Use barometer detection if nothing works
                    else {
                        if (fiveK_by_detection) {
                            drogue_flag = 1;
                            ping("DROGUE — TIER 4 barometer fallback");
                        }
                    }

                    if (get_fluctus_5k()) {
                        if (!drogue_flag) ping("DROGUE — Fluctus 5k override");
                        drogue_flag = 1;
                        fluctus_5k_detected = 1;
                    }

                    if (drogue_flag) {
                        phase = ST_WAIT_MAIN;
                        boot_params.phase = phase;
                        drogue_timestamp = current_time;
                        drogue_altitude = compute_height(mean(baro_detector.avg_size, baro_detector.average));
                        ping_detail("  drogue altitude (m)", drogue_altitude);
                    }
                }
                break;
            }

            case ST_WAIT_MAIN: {
                deployDrogue();
                insert(&baro_detector, bar1_val, bar2_val, phase, BARO_DTR);

                if (baro_detector.avg_size >= AD_CAPACITY) {
                    uint32_t current_time = time_since(ignition_timestamp);

                    uint8_t oneK_by_detection = detect_event(&baro_detector, phase);
                    uint8_t oneK_by_fallback = current_time >= fallback_1k_time;
                    uint8_t oneK_by_constant_timer = current_time >= MAIN_CONSTANT_TIMER;

                    // TIER 1
                    if (apogee_detection_worked) {
                        if (oneK_by_detection) {
                            main_flag = 1;
                            ping("MAIN — TIER 1 barometer detection");
                        }
                    }
                    // TIER 2
                    else if (fallback_timers_worked) {
                        if (oneK_by_fallback) {
                            main_flag = 1;
                            ping("MAIN — TIER 2 fallback timer");
                        }
                    }
                    // TIER 3
                    else if (constant_timers_worked) {
                        if (oneK_by_constant_timer) {
                            main_flag = 1;
                            ping("MAIN — TIER 3 constant timer");
                        }
                    }
                    // TIER 4
                    else {
                        if (oneK_by_detection) {
                            main_flag = 1;
                            ping("MAIN — TIER 4 barometer fallback");
                        }
                    }

                    if (get_fluctus_1k()) {
                        if (!main_flag) ping("MAIN — Fluctus 1k override");
                        main_flag = 1;
                        fluctus_1k_detected = 1;
                    }

                    if (main_flag) {
                        phase = ST_WAIT_GROUND;
                        boot_params.phase = phase;
                        main_timestamp = current_time;
                        main_altitude = compute_height(mean(baro_detector.avg_size, baro_detector.average));
                        ping_detail("  main altitude (m)", main_altitude);
                    }
                }
                break;
            }

            case ST_WAIT_GROUND: {
                deployMain();
                insert(&baro_detector, bar1_val, bar2_val, phase, BARO_DTR);

                if (baro_detector.avg_size >= AD_CAPACITY) {
                    if (detect_event(&baro_detector, phase)) {
                        phase = ST_DONE;
                        boot_params.phase = phase;
                        landed_flag = 1;
                        landed_timestamp = time_since(ignition_timestamp);
                        ping("LANDED");
                    }
                }
                break;
            }

            case ST_DONE: {
                ping("DONE — flight complete");
                goto end_loop;
            }

            default: {
                break;
            }
        } // end switch(phase)

        if (num_cycles % 20 == 0)
            boot_params.current_time_in_flight = time_since(ignition_timestamp);

        num_cycles++;
        vTaskDelayUntil(&last, period);
    }

end_loop:

    // ========================================================================
    // TEST RESULTS SUMMARY
    // ========================================================================
    printf("\n============================================================\n");
    printf("  TEST RESULTS SUMMARY\n");
    printf("============================================================\n\n");

    printf("--- FLAGS ---\n");
    printf("  MECO:           %d\n", MECO_flag);
    printf("  Apogee:         %d\n", apogee_flag);
    printf("    Detection:    %d\n", apogee_detection_worked);
    printf("    Fallback:     %d\n", fallback_timers_worked);
    printf("    Constant:     %d\n", constant_timers_worked);
    printf("  Drogue:         %d\n", drogue_flag);
    printf("  Main:           %d\n", main_flag);
    printf("  Landed:         %d\n\n", landed_flag);

    printf("--- FLUCTUS ---\n");
    printf("  Disabled:       %d\n", fluctus_disabled);
    printf("  Apogee:         %d (at %u ms)\n", fluctus_apogee_detected, fluctus_apogee_timestamp);
    printf("  5k:             %d\n", fluctus_5k_detected);
    printf("  1k:             %d\n\n", fluctus_1k_detected);

    printf("--- TIMESTAMPS (ms since ignition) ---\n");
    printf("  MECO:           %u ms\n", meco_timestamp);
    printf("  Apogee:         %u ms\n", apogee_timestamp);
    printf("  Drogue:         %u ms\n", drogue_timestamp);
    printf("  Main:           %u ms\n", main_timestamp);
    printf("  Landed:         %u ms\n\n", landed_timestamp);

    printf("--- ALTITUDES ---\n");
    printf("  Apogee:         %.2f m\n", apogee_altitude);
    printf("  Drogue deploy:  %.2f m\n", drogue_altitude);
    printf("  Main deploy:    %.2f m\n\n", main_altitude);

    printf("--- DEPLOY HARDWARE ---\n");
    printf("  Pilot chute:    %s\n", pilot_deployed  ? "DEPLOYED" : "NOT DEPLOYED");
    printf("  Drogue chute:   %s\n", drogue_deployed ? "DEPLOYED" : "NOT DEPLOYED");
    printf("  Main chute:     %s\n\n", main_deployed   ? "DEPLOYED" : "NOT DEPLOYED");

    if (heights_recorded) {
        printf("--- POST-LOCKOUT KINEMATICS ---\n");
        printf("  Altitude:       %.2f m\n", post_lockout_alt);
        printf("  Velocity:       %.2f m/s\n", post_lockout_vel);
        printf("  Acceleration:   %.2f m/s^2\n\n", approximated_accel);

        printf("--- FALLBACK TIMER PREDICTIONS ---\n");
        printf("  Apogee:         %u ms  (%.2f m)\n", fallback_apogee_time, fallback_apogee_altitude);
        printf("  Drogue:         %u ms  (%.2f m)\n", fallback_5k_time, fallback_5k_altitude);
        printf("  Main:           %u ms  (%.2f m)\n", fallback_1k_time, fallback_1k_altitude);
    }

    printf("--- GROUND CONDITIONS ---\n");
    printf("  T_GROUND:       %.2f K (%.2f C)\n", T_GROUND, T_GROUND - 273.15f);
    printf("  P_GROUND:       %.2f hPa\n", P_GROUND);
    printf("  Drogue pressure:%.2f hPa (target alt: %.0f m)\n", DROGUE_DEPLOY_PRESSURE, DROGUE_DEPLOY_ALTITUDE);
    printf("  Main pressure:  %.2f hPa (target alt: %.0f m)\n", MAIN_DEPLOY_PRESSURE, MAIN_DEPLOY_ALTITUDE);

    printf("\n============================================================\n");
    printf("  Final phase: %s\n", phase_names[phase]);
    printf("============================================================\n");

    return 0;
}

// ============================================================================
// MAIN — entry point for the test
// ============================================================================
int main(void) {
    printf("\n============================================================\n");
    printf("  AUTOSEQUENCE INTEGRATION TEST\n");
    printf("  %d samples @ %d Hz (%.1fs simulated flight)\n",
           DATA_SIZE, SAMPLE_RATE_HZ, DATA_SIZE * SAMPLE_PERIOD_MS / 1000.0f);
    printf("  Fluctus apogee at sample %d, 5k at %d, 1k at %d\n",
           FLUCTUS_APOGEE_SAMPLE, FLUCTUS_5K_SAMPLE, FLUCTUS_1K_SAMPLE);
    printf("============================================================\n\n");

    Autos_boot_t boot = {0};
    boot.phase = ST_DISARMED;

    int result = execute_flight_autosequence(boot);

    printf("\n  execute_flight_autosequence returned: %d\n", result);
    return result;
}
