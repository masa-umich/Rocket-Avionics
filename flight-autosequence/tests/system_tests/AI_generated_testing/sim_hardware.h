#ifndef SIM_HARDWARE_H
#define SIM_HARDWARE_H


typedef struct {
    float XL_x; // accel in x direction
    float XL_y; // y direction is axial to the rocket - positive points from engine to nose cone (upwards)
    float XL_z;
    float W_x;  // angular velocity around x axis
    float W_y;
    float W_z;
} IMU_values;
/*
 * sim_hardware.h
 *
 * Declares every hardware-abstraction function called by execute_flight_autosequence()
 * that is NOT already declared in ad-functions.h or autosequence_script.h.
 *
 * In production these are provided by the RTOS / peripheral drivers.
 * In simulation they are backed by sim_hardware.c, which streams rows from
 * sim_sensor_data.csv at the configured sample period.
 *
 * Add  #include "sim_hardware.h"  to test_autosequence_AI.h (or your test main)
 * before compiling.
 */

#include <stdint.h>
#include "ad-functions.h"   /* IMU_values, FlightPhase, … */

/* ── Utility macro expected by the autosequence ─────────────────────────── */
#ifndef UNUSED
#  define UNUSED(x) ((void)(x))
#endif

/* ── Path to the sensor-data CSV (override at compile time if needed) ───── */
#ifndef SIM_CSV_FILE
#  define SIM_CSV_FILE "tests/system_tests/AI_generated_testing/sim_sensor_data.csv"
#endif

/* ── Simulated time ──────────────────────────────────────────────────────── */

/** Returns the current simulated wall-clock time in milliseconds.
 *  Advances every time get_sensor_data() reads a new CSV row. */
uint32_t getTime(void);

/** Returns (getTime() - timestamp), i.e. elapsed ms since timestamp. */
uint32_t time_since(uint32_t timestamp);

/* ── Sensor data ─────────────────────────────────────────────────────────── */

/** Read one 20 ms tick of sensor data from the CSV.
 *  Advances the simulated clock.  Halts with an error if the file is exhausted. */
void get_sensor_data(float *bar1_hPa, float *bar2_hPa,
                     IMU_values *imu1, IMU_values *imu2,
                     float *bar1_temp_C, float *bar2_temp_C);

/* ── Abort / arm ─────────────────────────────────────────────────────────── */

/** Returns non-zero to request an immediate abort.
 *  Always returns 0 in simulation (no abort injected). */
int should_abort(void);

/** Returns non-zero once the simulated launch rail / fill valves are open.
 *  Transitions at VALVES_OPEN_SIM_MS (500 ms by default). */
int valves_open(void);

/* ── Pyro / deployment channels ─────────────────────────────────────────── */
void deployPilot(void);    /**< Fire pilot / drogue-deployment initiator.  */
void deployDrogue(void);   /**< Fire drogue shear-pin cutter / gas-gen.    */
void deployMain(void);     /**< Fire main chute bag release.                */

/* ── Fluctus (secondary FC) discrete outputs ─────────────────────────────── */

/** Returns 1 once the Fluctus FC has broadcast its apogee detection. */
uint8_t get_fluctus_apogee(void);

/** Returns 1 once the Fluctus FC has broadcast 5 000 ft detection. */
uint8_t get_fluctus_5k(void);

/** Returns 1 once the Fluctus FC has broadcast 1 000 ft detection. */
uint8_t get_fluctus_1k(void);

#endif /* SIM_HARDWARE_H */