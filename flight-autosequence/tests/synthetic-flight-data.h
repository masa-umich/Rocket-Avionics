/**
 * synthetic-flight-data.h
 *
 * Synthetic flight sensor data for integration testing of the autosequence.
 * This file provides realistic sensor readings that simulate a complete
 * rocket flight from pre-launch through landing.
 *
 * DATA CHARACTERISTICS:
 * - 200 samples at 20 Hz (10 seconds simulated, representing a compressed flight)
 * - Includes intentional edge cases to test detector robustness:
 *   - Acceleration spike at sample 30 (sensor glitch)
 *   - Mach transition dip at samples 40-44 (transonic effects)
 *   - Pressure spike at sample 84 (Mach lockout period)
 *   - Flat apogee region at samples 120-129 (minimum detectable change)
 *   - Barometer 1 dropout at samples 160-162 (sensor failure)
 *
 * SENSOR REDUNDANCY:
 * The flight computer uses redundant sensors (bar1/bar2, imu1/imu2) to
 * handle single-sensor failures. In this test data:
 * - bar1 has a pressure spike at sample 84 and dropout at 160-162
 * - bar2 does NOT have these anomalies
 * - imu1 has an acceleration spike at sample 30
 * - imu2 does NOT have this spike
 *
 * FLIGHT PROFILE:
 * The data simulates the following flight phases:
 * - Samples 0-19: Pre-launch, on pad, valves closed
 * - Sample 15: Valves open (ignition)
 * - Samples 20-59: Boost phase (rapid altitude gain, high acceleration)
 * - Samples 40-44: Mach transition (acceleration dip)
 * - Samples 50-59: MECO region (acceleration goes negative)
 * - Samples 60-89: Mach lockout (barometer unreliable)
 * - Samples 90-119: Coast to apogee (decreasing vertical velocity)
 * - Samples 120-129: Apogee (minimum pressure, zero vertical velocity)
 * - Samples 130-169: Descent (increasing pressure)
 * - Samples 170-199: Landing (ground pressure, stable)
 *
 * USAGE:
 * Include this file in test code after defining the test environment.
 * Access sensor data via arrays indexed by current_sample.
 */

#ifndef SYNTHETIC_FLIGHT_DATA_H
#define SYNTHETIC_FLIGHT_DATA_H

// ============================================================================
// DATA CONFIGURATION CONSTANTS
// ============================================================================

/**
 * DATA_SIZE - Total number of samples in each sensor array
 *
 * 200 samples provides enough data to test all flight phases while
 * keeping test execution time reasonable.
 */
#define DATA_SIZE 200

/**
 * SAMPLE_RATE_HZ - Simulated sampling frequency in Hertz
 *
 * 20 Hz matches typical flight computer sampling rates and provides
 * sufficient resolution for detecting flight events.
 */
#define SAMPLE_RATE_HZ 20

/**
 * SAMPLE_PERIOD_MS - Time between samples in milliseconds
 *
 * Computed from sample rate: 1000ms / 20Hz = 50ms per sample.
 * Used to convert sample indices to simulated time.
 */
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)  // 50ms

// ============================================================================
// BAROMETER 1 - Primary pressure sensor (hPa)
//
// Pressure decreases with altitude following the barometric formula.
// Ground level: ~1013 hPa (standard atmosphere)
// Apogee: ~71 hPa (corresponding to ~18km altitude)
//
// EDGE CASES IN THIS SENSOR:
// - Sample 84: Spurious pressure spike to 150 hPa during Mach lockout
// - Samples 160-162: Sensor dropout (reads 0 hPa) during descent
//
// The autosequence must handle these anomalies using redundant sensor
// data and data imputation.
// ============================================================================
static const float bar1[DATA_SIZE] = {
    // ========================================================================
    // PRE-LAUNCH / VALVES CLOSED (samples 0-19)
    // Rocket is stationary on launch pad at ground level
    // Pressure is stable at standard sea level value
    // ========================================================================
    1013, 1013, 1013, 1013, 1013,  // Samples 0-4: Ground level
    1013, 1013, 1013, 1013, 1013,  // Samples 5-9: Ground level
    1013, 1013, 1013, 1013, 1013,  // Samples 10-14: Ground level (valves open at 15)
    1013, 1013, 1013, 1013, 1013,  // Samples 15-19: Valves now open, still on pad

    // ========================================================================
    // BOOST PHASE (samples 20-59)
    // Rapid altitude gain causes pressure to decrease quickly
    // Pressure drops from 1013 hPa to ~110 hPa (sea level to ~16km)
    // ========================================================================
    1010, 990, 960, 920, 870,      // Samples 20-24: Initial climb
    810, 740, 670, 600, 530,       // Samples 25-29: Accelerating ascent
    470, 420, 375, 335, 300,       // Samples 30-34: Continuing climb
    270, 245, 225, 208, 193,       // Samples 35-39: Approaching Mach 1
    180, 170, 162, 155, 149,       // Samples 40-44: Transonic region
    144, 140, 136, 133, 130,       // Samples 45-49: Post-transonic
    128, 126, 124, 122, 120,       // Samples 50-54: MECO region
    118, 116, 114, 112, 110,       // Samples 55-59: End of boost

    // ========================================================================
    // MACH LOCKOUT (samples 60-89)
    // Barometer readings may be erratic above Mach 1 due to shock waves
    // Autosequence ignores barometer during this phase
    // ========================================================================
    108, 106, 104, 102, 100,       // Samples 60-64: Early lockout
    99, 98, 97, 96, 95,            // Samples 65-69: Mid lockout
    94, 93, 92, 91, 90,            // Samples 70-74: Continuing lockout
    89, 88, 87, 86, 85,            // Samples 75-79: Late lockout
    // EDGE CASE: Sample 84 has spurious pressure spike to 150 hPa
    // This tests that the detector ignores erratic readings during lockout
    84, 83, 82, 81, 150,           // Samples 80-84: SPIKE at 84 (150 hPa)
    79, 78, 77, 76, 75,            // Samples 85-89: End of lockout

    // ========================================================================
    // COAST TO APOGEE (samples 90-119)
    // Rocket is coasting upward, decelerating due to gravity
    // Pressure continues to decrease but rate slows as velocity decreases
    // ========================================================================
    74.5, 74, 73.5, 73, 72.5,      // Samples 90-94: Slow pressure decrease
    72.2, 72, 71.8, 71.6, 71.5,    // Samples 95-99: Approaching apogee
    71.4, 71.3, 71.2, 71.15, 71.1, // Samples 100-104: Very slow change
    71.08, 71.06, 71.05, 71.04, 71.03, // Samples 105-109: Near zero velocity

    // ========================================================================
    // APOGEE REGION (samples 120-129)
    // EDGE CASE: Flat region with minimal pressure change
    // Tests apogee detection when signal is near noise floor
    // Pressure reaches minimum ~71.02 hPa, then starts increasing
    // ========================================================================
    71.02, 71.02, 71.02, 71.02, 71.03, // Samples 120-124: Flat minimum
    71.04, 71.06, 71.1, 71.15, 71.25,  // Samples 125-129: Beginning descent

    // ========================================================================
    // DESCENT (samples 130-169)
    // Rocket falling under gravity, then parachutes slow descent
    // Pressure increases as altitude decreases
    // ========================================================================
    71.5, 72, 73, 75, 78,          // Samples 130-134: Initial descent
    82, 88, 96, 106, 120,          // Samples 135-139: Accelerating descent
    138, 160, 188, 222, 264,       // Samples 140-144: Continued descent
    314, 374, 444, 524, 614,       // Samples 145-149: Rapid altitude loss
    700, 770, 830, 880, 920,       // Samples 150-154: Parachute slowing descent
    952, 978, 998, 1010, 1018,     // Samples 155-159: Approaching ground

    // EDGE CASE: Samples 160-162 sensor dropout (reads 0)
    // Tests data imputation using redundant barometer
    0, 0, 0,                        // Samples 160-162: SENSOR DROPOUT
    1012, 1013, 1013, 1013, 1013,   // Samples 163-167: Recovery/landing
    1013, 1013,                     // Samples 168-169: On ground

    // ========================================================================
    // LANDING (samples 170-199)
    // Rocket has landed, pressure stable at ground level
    // Tests landing detection via pressure slope = 0
    // ========================================================================
    1013, 1013, 1013, 1013, 1013,  // Samples 170-174: On ground
    1013, 1013, 1013, 1013, 1013,  // Samples 175-179: On ground
    1013, 1013, 1013, 1013, 1013,  // Samples 180-184: On ground
    1013, 1013, 1013, 1013, 1013,  // Samples 185-189: On ground
    1013, 1013, 1013, 1013, 1013,  // Samples 190-194: On ground
    1013, 1013, 1013, 1013, 1013   // Samples 195-199: On ground
};

// ============================================================================
// BAROMETER 2 - Redundant pressure sensor (hPa)
//
// Provides backup pressure readings for sensor fusion.
// This sensor does NOT have the anomalies present in bar1:
// - No pressure spike at sample 84
// - No dropout at samples 160-162 (provides valid data for imputation)
//
// The detector averages both barometers and uses imputation when one fails.
// ============================================================================
static const float bar2[DATA_SIZE] = {
    // Pre-launch: Identical to bar1
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,

    // Boost phase: Identical to bar1
    1010, 990, 960, 920, 870, 810, 740, 670, 600, 530,
    470, 420, 375, 335, 300, 270, 245, 225, 208, 193,
    180, 170, 162, 155, 149, 144, 140, 136, 133, 130,
    128, 126, 124, 122, 120, 118, 116, 114, 112, 110,

    // Mach lockout: Note sample 84 is 80 (NOT 150 like bar1)
    108, 106, 104, 102, 100, 99, 98, 97, 96, 95,
    94, 93, 92, 91, 90, 89, 88, 87, 86, 85,
    84, 83, 82, 81, 80,                        // Sample 84: 80 hPa (no spike)
    79, 78, 77, 76, 75,

    // Coast to apogee: Identical to bar1
    74.5, 74, 73.5, 73, 72.5, 72.2, 72, 71.8, 71.6, 71.5,
    71.4, 71.3, 71.2, 71.15, 71.1, 71.08, 71.06, 71.05, 71.04, 71.03,

    // Apogee: Identical to bar1
    71.02, 71.02, 71.02, 71.02, 71.03, 71.04, 71.06, 71.1, 71.15, 71.25,

    // Descent: Identical to bar1
    71.5, 72, 73, 75, 78, 82, 88, 96, 106, 120,
    138, 160, 188, 222, 264, 314, 374, 444, 524, 614,
    700, 770, 830, 880, 920, 952, 978, 998, 1010, 1018,

    // Landing region: bar2 provides valid data during bar1 dropout
    1020, 1018, 1015, 1012, 1013,              // Samples 160-164: Valid readings
    1013, 1013, 1013, 1013, 1013,              // Samples 165-169: Ground level

    // On ground: Identical to bar1
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013
};

// ============================================================================
// IMU 1 - Primary Z-axis accelerometer (m/s^2)
//
// Z-axis acceleration, positive = upward (rocket pointing up).
// At rest: 0 m/s^2 (note: this is unusual - typically would read ~9.8)
// During boost: High positive values (thrust > gravity)
// After MECO: Negative values (gravity decelerating upward motion)
// During descent: ~9.8 m/s^2 (freefall relative to rocket frame)
//
// EDGE CASES IN THIS SENSOR:
// - Sample 30: Acceleration spike to 120 m/s^2 (sensor glitch or vibration)
// - Samples 40-44: Mach transition dip (transonic aerodynamic effects)
//
// The detector uses sensor fusion with imu2 to filter these anomalies.
// ============================================================================
static const float imu1_z[DATA_SIZE] = {
    // ========================================================================
    // PRE-LAUNCH (samples 0-19)
    // Rocket at rest on pad, no acceleration
    // Note: In real flight, would read ~9.8 due to gravity, but we're using
    // a body-frame reference where 0 = stationary
    // ========================================================================
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Samples 0-9: At rest
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Samples 10-19: At rest

    // ========================================================================
    // BOOST PHASE (samples 20-59)
    // High acceleration during motor burn
    // Acceleration builds to peak, then decreases as propellant depletes
    // ========================================================================
    25, 40, 52, 60, 65,            // Samples 20-24: Acceleration building
    68, 70, 71, 72, 72,            // Samples 25-29: Approaching peak

    // EDGE CASE: Sample 30 has acceleration spike to 120 m/s^2
    // This tests that the detector filters single-sample anomalies
    120,                            // Sample 30: SPIKE (120 m/s^2)
    72, 71, 70, 68,                // Samples 31-34: Normal readings
    65, 60, 55, 48, 40,            // Samples 35-39: Decreasing (thrust tailoff)

    // EDGE CASE: Samples 40-44 Mach transition dip
    // Transonic flight causes pressure waves affecting measured acceleration
    30, 25, 35, 28, 22,            // Samples 40-44: MACH DIP (erratic)

    // Deceleration to MECO (acceleration becomes negative = deceleration)
    15, 8, 2,                       // Samples 45-47: Low positive accel
    -2, -5,                         // Samples 48-49: MECO region (negative)
    -7, -8, -9, -9.3, -9.5,        // Samples 50-54: Decelerating upward
    -9.6, -9.7, -9.8, -9.8, -9.8,  // Samples 55-59: Approaching ~1g decel

    // ========================================================================
    // POST-MECO / LOCKOUT (samples 60-89)
    // Coasting upward, decelerating due to drag and gravity
    // Acceleration trends toward +9.8 m/s^2 as drag decreases
    // ========================================================================
    -9.8, -9.6, -9.3, -9.0, -8.5,  // Samples 60-64: Drag dominates
    -8.0, -7.3, -6.5, -5.5, -4.5,  // Samples 65-69: Drag decreasing
    -3.5, -2.5, -1.5, -0.5, 0.5,   // Samples 70-74: Crossing zero
    1.5, 2.5, 3.5, 4.5, 5.5,       // Samples 75-79: Positive (slowing down)
    6.3, 7.0, 7.6, 8.1, 8.5,       // Samples 80-84: Approaching 1g
    8.8, 9.1, 9.3, 9.5, 9.6,       // Samples 85-89: Near freefall

    // ========================================================================
    // COAST TO APOGEE (samples 90-119)
    // Very low velocity, acceleration ~9.8 m/s^2 (freefall)
    // ========================================================================
    9.7, 9.75, 9.78, 9.8, 9.8,     // Samples 90-94: Approaching apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 95-99: Near apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 100-104: At/near apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 105-109: At/near apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 110-114: At/near apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 115-119: At/near apogee

    // ========================================================================
    // APOGEE (samples 120-129)
    // Vertical velocity = 0, freefall acceleration
    // ========================================================================
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 120-124: At apogee
    9.8, 9.8, 9.8, 9.8, 9.8,       // Samples 125-129: Beginning descent

    // ========================================================================
    // DESCENT (samples 130-169)
    // Falling, then parachutes deploy slowing descent
    // Acceleration remains ~9.8 (freefall relative to body frame)
    // ========================================================================
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,  // 130-139
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,  // 140-149
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,  // 150-159
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,  // 160-169

    // ========================================================================
    // LANDING (samples 170-199)
    // Impact spike, then at rest
    // ========================================================================
    15, 5,                          // Samples 170-171: Landing impact
    0, 0, 0, 0, 0, 0, 0, 0,        // Samples 172-179: At rest
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Samples 180-189: At rest
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0   // Samples 190-199: At rest
};

// ============================================================================
// IMU 2 - Redundant Z-axis accelerometer (m/s^2)
//
// Backup accelerometer for sensor fusion.
// This sensor does NOT have the spike at sample 30 that imu1 has.
// Otherwise follows the same profile as imu1.
//
// The detector averages both IMUs to filter single-sensor anomalies.
// ============================================================================
static const float imu2_z[DATA_SIZE] = {
    // Pre-launch: Identical to imu1
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // Boost phase: Note sample 30 is 72 (NOT 120 like imu1)
    25, 40, 52, 60, 65, 68, 70, 71, 72, 72,
    72,                                        // Sample 30: 72 m/s^2 (no spike)
    72, 71, 70, 68, 65, 60, 55, 48, 40,

    // Mach dip and MECO: Identical to imu1
    30, 25, 35, 28, 22,
    15, 8, 2, -2, -5,
    -7, -8, -9, -9.3, -9.5, -9.6, -9.7, -9.8, -9.8, -9.8,

    // Post-MECO: Identical to imu1
    -9.8, -9.6, -9.3, -9.0, -8.5, -8.0, -7.3, -6.5, -5.5, -4.5,
    -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5,
    6.3, 7.0, 7.6, 8.1, 8.5, 8.8, 9.1, 9.3, 9.5, 9.6,

    // Coast to apogee: Identical to imu1
    9.7, 9.75, 9.78, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,

    // Descent: Identical to imu1
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,
    9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8, 9.8,

    // Landing: Identical to imu1
    15, 5, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// ============================================================================
// TEMPERATURE SENSORS - Kelvin
//
// Temperature readings from barometer-integrated temperature sensors.
// Temperature decreases with altitude in the troposphere following
// the standard lapse rate (~6.5 K/km).
//
// Ground temperature: ~293 K (20C)
// Tropopause temperature: ~217 K (-56C) - minimum, stays constant above
//
// Temperature readings are used to:
// 1. Establish ground conditions before launch
// 2. Calculate accurate altitude using non-standard atmosphere model
// 3. Validate sensor health (out-of-range values indicate failure)
// ============================================================================
static const float temp1_K[DATA_SIZE] = {
    // ========================================================================
    // PRE-LAUNCH / GROUND (samples 0-19)
    // Ground temperature ~293K (approximately 20 degrees Celsius)
    // ========================================================================
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // Samples 0-9
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // Samples 10-19

    // ========================================================================
    // BOOST / ASCENT (samples 20-39)
    // Temperature decreases with altitude at ~6.5 K/km
    // From 293K at ground to 217K at tropopause (~11km)
    // ========================================================================
    291, 287, 281, 273, 263,       // Samples 20-24: Cooling during climb
    252, 240, 230, 222, 218,       // Samples 25-29: Approaching tropopause
    217, 217, 217, 217, 217,       // Samples 30-34: At tropopause
    217, 217, 217, 217, 217,       // Samples 35-39: Above tropopause

    // ========================================================================
    // HIGH ALTITUDE (samples 40-119)
    // Temperature constant at tropopause value (~217K)
    // Isothermal region of atmosphere
    // ========================================================================
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 40-49
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 50-59
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 60-69
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 70-79
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 80-89
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 90-99
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 100-109
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,  // 110-119

    // ========================================================================
    // DESCENT (samples 120-149)
    // Temperature increases as altitude decreases
    // Back through tropopause to warmer air
    // ========================================================================
    217, 218, 220, 224, 230,       // Samples 120-124: Beginning descent
    238, 248, 258, 268, 278,       // Samples 125-129: Below tropopause
    283, 286, 288, 290, 291,       // Samples 130-134: Approaching ground
    292, 292, 293, 293, 293,       // Samples 135-139: Near ground temp
    293, 293, 293, 293, 293,       // Samples 140-144: Ground temperature
    293, 293, 293, 293, 293,       // Samples 145-149: Ground temperature

    // ========================================================================
    // LANDING / GROUND (samples 150-199)
    // Temperature stable at ground value
    // ========================================================================
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // 150-159
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // 160-169
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // 170-179
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,  // 180-189
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293   // 190-199
};

// ============================================================================
// TEMPERATURE SENSOR 2 - Redundant (Kelvin)
//
// Backup temperature sensor providing identical readings to temp1_K.
// Both sensors are functional throughout the test flight.
// ============================================================================
static const float temp2_K[DATA_SIZE] = {
    // Identical to temp1_K - both sensors working correctly
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    291, 287, 281, 273, 263, 252, 240, 230, 222, 218,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 218, 220, 224, 230, 238, 248, 258, 268, 278,
    283, 286, 288, 290, 291, 292, 292, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293
};

// ============================================================================
// VALVE STATE - Propellant valve status
//
// Indicates whether the main propellant valves are open (1) or closed (0).
// Valves opening signals ignition and the start of the boost phase.
//
// The autosequence waits for valves to open before transitioning from
// ST_DETECT_VALVES_OPEN to ST_WAIT_MECO. If valves don't open within
// the timeout (MAX_HANDOFF_TO_VALVE_OPEN_MS), the autosequence aborts.
//
// In this test data:
// - Samples 0-14: Valves closed (pre-ignition)
// - Sample 15: Valves open (ignition)
// - Samples 16-199: Valves remain open
// ============================================================================
static const int valves_open_data[DATA_SIZE] = {
    // ========================================================================
    // PRE-IGNITION (samples 0-14)
    // Valves closed, waiting for ignition command
    // ========================================================================
    0, 0, 0, 0, 0,                 // Samples 0-4: Closed
    0, 0, 0, 0, 0,                 // Samples 5-9: Closed
    0, 0, 0, 0, 0,                 // Samples 10-14: Closed

    // ========================================================================
    // IGNITION AND FLIGHT (samples 15-199)
    // Valves open at sample 15 and remain open
    // ========================================================================
    1, 1, 1, 1, 1,                 // Samples 15-19: Open (ignition at 15)
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 20-29: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 30-39: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 40-49: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 50-59: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 60-69: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 70-79: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 80-89: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 90-99: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 100-109: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 110-119: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 120-129: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 130-139: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 140-149: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 150-159: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 160-169: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 170-179: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // Samples 180-189: Open
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1   // Samples 190-199: Open
};

#endif // SYNTHETIC_FLIGHT_DATA_H
