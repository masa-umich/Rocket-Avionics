/**
 * synthetic-flight-data-3.h
 *
 * SCENARIO: "Extended Flat Apogee + Sensor Brownout"
 *
 * High-altitude flight with an unusually flat apogee region and multiple
 * sensor failures that overlap in time, stress-testing the system's
 * ability to detect apogee when the signal is barely above the noise floor.
 *
 * EDGE CASES:
 * 1. Temperature sensor garbage during pre-launch (samples 5-14):
 *    Both temp sensors read 423K (150C), well above TEMP_C_MAX (50C).
 *    fix_reading should impute these. Since this happens BEFORE valves open,
 *    if T_GROUND is set from the last reading (sample 14 = garbage), it will
 *    use the imputed value (buffer mean). Tests ground calibration robustness.
 *
 * 2. Bar1 intermittent during lockout (samples 60-79):
 *    Alternates between valid readings and 0 every 2-3 samples.
 *    fix_reading should impute the zeros. Bar2 is clean.
 *    Tests imputation under rapid intermittent failure pattern.
 *
 * 3. Both IMUs fail at sample 82 — read 0.0 m/s^2 for the rest of
 *    the flight. Since 0 is within ACCEL_MIN/MAX range, this is NOT
 *    filtered by fix_reading. The IMU buffer fills with zeros.
 *    Tests: system behavior when IMU gives plausible but wrong data.
 *    (IMU is only used during WAIT_MECO, so if MECO was already
 *    detected this won't affect later phases.)
 *
 * 4. Extended flat apogee (samples 100-139): 40 samples where pressure
 *    varies by only 0.2 hPa total (71.0 to 71.2). The pressure slope
 *    is extremely close to zero, making apogee detection via linreg_slope
 *    very difficult. Tests the sensitivity of the slope threshold.
 *
 * 5. Rapid pressure spike at sample 142 — bar1 jumps to 300 hPa then
 *    immediately returns to normal. Single-sample anomaly during descent.
 *
 * EXPECTED BEHAVIOR:
 * - T_GROUND may be slightly corrupted depending on imputation quality
 * - Bar1 intermittent failures imputed from buffer mean during lockout
 * - Apogee detection delayed due to extremely flat pressure minimum
 * - May require fallback timers if slope never clearly crosses zero
 */

#ifndef SYNTHETIC_FLIGHT_DATA_3_H
#define SYNTHETIC_FLIGHT_DATA_3_H

#define DATA_SIZE 200
#define SAMPLE_RATE_HZ 20
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)

// Barometer 1: intermittent during lockout (60-79), spike at 142
static const float bar1[DATA_SIZE] = {
    // Pre-launch (0-19)
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    // Boost (20-59)
    1010, 990, 960, 920, 870, 810, 740, 670, 600, 530,
    470, 420, 375, 335, 300, 270, 245, 225, 208, 193,
    180, 170, 162, 155, 149, 144, 140, 136, 133, 130,
    128, 126, 124, 122, 120, 118, 116, 114, 112, 110,
    // INTERMITTENT LOCKOUT (60-79) — alternates valid/zero
    108, 106, 0, 0, 100, 99, 0, 0, 96, 95,
    0, 0, 92, 91, 0, 0, 88, 87, 0, 0,
    // Lockout continues clean (80-89)
    84, 83, 82, 81, 80, 79, 78, 77, 76, 75,
    // Coast to apogee (90-99) — normal
    74.5, 74, 73.5, 73, 72.5, 72.2, 72, 71.8, 71.5, 71.3,
    // EXTENDED FLAT APOGEE (100-139) — 40 samples, only 0.2 hPa variation
    71.2, 71.15, 71.1, 71.08, 71.05, 71.02, 71.0, 71.0, 71.0, 71.0,
    71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0,
    71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.01,
    71.02, 71.03, 71.05, 71.08, 71.1, 71.12, 71.15, 71.18, 71.2, 71.25,
    // Descent (140-169) — pressure spike at 142
    71.5, 72, 300, 75, 78, 82, 88, 96, 106, 120,
    138, 160, 188, 222, 264, 314, 374, 444, 524, 614,
    700, 770, 830, 880, 920, 952, 978, 998, 1010, 1013,
    // Landing (170-199)
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013
};

// Barometer 2: clean throughout — no anomalies
static const float bar2[DATA_SIZE] = {
    // Pre-launch (0-19)
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    // Boost (20-59) — identical to original
    1010, 990, 960, 920, 870, 810, 740, 670, 600, 530,
    470, 420, 375, 335, 300, 270, 245, 225, 208, 193,
    180, 170, 162, 155, 149, 144, 140, 136, 133, 130,
    128, 126, 124, 122, 120, 118, 116, 114, 112, 110,
    // Lockout (60-89) — clean
    108, 106, 104, 102, 100, 99, 98, 97, 96, 95,
    94, 93, 92, 91, 90, 89, 88, 87, 86, 85,
    84, 83, 82, 81, 80, 79, 78, 77, 76, 75,
    // Coast to apogee (90-99)
    74.5, 74, 73.5, 73, 72.5, 72.2, 72, 71.8, 71.5, 71.3,
    // EXTENDED FLAT APOGEE (100-139) — same as bar1
    71.2, 71.15, 71.1, 71.08, 71.05, 71.02, 71.0, 71.0, 71.0, 71.0,
    71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0,
    71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.0, 71.01,
    71.02, 71.03, 71.05, 71.08, 71.1, 71.12, 71.15, 71.18, 71.2, 71.25,
    // Descent (140-169) — clean, no spike
    71.5, 72, 73, 75, 78, 82, 88, 96, 106, 120,
    138, 160, 188, 222, 264, 314, 374, 444, 524, 614,
    700, 770, 830, 880, 920, 952, 978, 998, 1010, 1013,
    // Landing (170-199)
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013,
    1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013, 1013
};

// IMU1: normal until sample 82, then reads 0 (plausible but wrong)
static const float imu1_z[DATA_SIZE] = {
    // Pre-launch (0-19)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // Boost (20-59) — same as original
    25, 40, 52, 60, 65, 68, 70, 71, 72, 72,
    72, 72, 71, 70, 68, 65, 60, 55, 48, 40,
    30, 25, 35, 28, 22, 15, 8, 2, -2, -5,
    -7, -8, -9, -9.3, -9.5, -9.6, -9.7, -9.8, -9.8, -9.8,
    // Post-MECO (60-81) — normal
    -9.8, -9.6, -9.3, -9.0, -8.5, -8.0, -7.3, -6.5, -5.5, -4.5,
    -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5,
    6.3, 7.0,
    // BOTH IMUS FAIL AT SAMPLE 82 — read 0 (within valid range, NOT filtered)
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // Continues reading 0 for rest of flight (100-199)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// IMU2: same failure pattern as IMU1 — both die at sample 82
static const float imu2_z[DATA_SIZE] = {
    // Pre-launch (0-19)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // Boost (20-59) — same as original
    25, 40, 52, 60, 65, 68, 70, 71, 72, 72,
    72, 72, 71, 70, 68, 65, 60, 55, 48, 40,
    30, 25, 35, 28, 22, 15, 8, 2, -2, -5,
    -7, -8, -9, -9.3, -9.5, -9.6, -9.7, -9.8, -9.8, -9.8,
    // Post-MECO (60-81) — normal
    -9.8, -9.6, -9.3, -9.0, -8.5, -8.0, -7.3, -6.5, -5.5, -4.5,
    -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5,
    6.3, 7.0,
    // BOTH IMUS FAIL AT SAMPLE 82
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // Continues reading 0 (100-199)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Temp1: garbage readings (423K = 150C) during pre-launch samples 5-14
// fix_reading should impute these since 150C > TEMP_C_MAX (50C)
static const float temp1_K[DATA_SIZE] = {
    // Samples 0-4: valid pre-launch
    293, 293, 293, 293, 293,
    // Samples 5-14: GARBAGE — 423K (150C), outside valid Celsius range
    423, 423, 423, 423, 423, 423, 423, 423, 423, 423,
    // Samples 15-19: valid again
    293, 293, 293, 293, 293,
    // Boost/ascent (20-59) — normal
    291, 287, 281, 273, 263, 252, 240, 230, 222, 218,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    // High altitude (60-139) — constant at tropopause
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    // Descent (140-169) — warming up
    217, 218, 220, 224, 230, 238, 248, 258, 268, 278,
    283, 286, 288, 290, 291, 292, 292, 293, 293, 293,
    // Landing (170-199)
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293
};

// Temp2: same garbage pattern as temp1 (both sensors affected)
static const float temp2_K[DATA_SIZE] = {
    293, 293, 293, 293, 293,
    423, 423, 423, 423, 423, 423, 423, 423, 423, 423,
    293, 293, 293, 293, 293,
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
    217, 217, 217, 217, 217, 217, 217, 217, 217, 217,
    217, 218, 220, 224, 230, 238, 248, 258, 268, 278,
    283, 286, 288, 290, 291, 292, 292, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293,
    293, 293, 293, 293, 293, 293, 293, 293, 293, 293
};

// Valves open at sample 15 (standard timing)
static const int valves_open_data[DATA_SIZE] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

#endif
