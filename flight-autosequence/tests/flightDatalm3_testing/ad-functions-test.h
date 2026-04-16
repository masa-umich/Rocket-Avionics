#ifndef AD_FUNCTIONS_H
#define AD_FUNCTIONS_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define ALTITUDE_BUFFER_SIZE 11
// size of the array of readings we maintain during the flight
// used with the barometer and the IMU
#define AD_CAPACITY 5          

// --- PHYSICS CONSTANTS ---
extern const float CROSS_SECT_AREA;           // m^2
extern const float MASS_AT_MECO;              // kg
extern const float CD;                        // drag coefficient
extern const float G;                         // gravity
extern const float MOLAR_MASS_AIR;            // kg/mol
extern const float R_GAS_CONST;               // gas constant
extern const float GAMMA;

// --- DEPLOYMENT ALTITUDES ---
extern const float DROGUE_DEPLOY_ALTITUDE;     // m
extern const float MAIN_DEPLOY_ALTITUDE;       // m
extern float GROUND_ALTITUDE;                  // m, to be set at launch

// --- THRESHOLD VALUES FOR BAD DATA ---
extern const float ACCEL_MIN; // m/s^2
// no max accel threshold because accel readings during burn will be very high

// Changed from BARO to ALT thresholds
extern const float ALT_MIN; // m
extern const float ALT_MAX; // m

extern const float TEMP_C_MIN; // Celsius
extern const float TEMP_C_MAX; // Celsius
extern const float TEMP_K_MIN; // Kelvin
extern const float TEMP_K_MAX; // Kelvin

extern const float MACH_DIP_THRESHOLD; // drop in acceleration in m/s^2 that indicates transition
extern const float MIN_START_ACCEL;    // minimum accel we need to see to consider detecting a spike

// Updated enum to reflect computed values
typedef enum {
    ACCEL_DTR = 0,
    ALT_DTR,
    TEMP_DTR
} DetectorType;

typedef struct {
    float readings_1[AD_CAPACITY]; 
    int index_1;
    int size_1;

    float readings_2[AD_CAPACITY];
    int index_2;
    int size_2;

    float average[AD_CAPACITY];
    int avg_index;
    int avg_size;

    float slope[AD_CAPACITY];
    int slope_i;
    int slope_size;
} Detector;

// What 'state' is our apogee-detection system in? 
typedef enum {
    ST_DETECT_VALVES_OPEN = 0,
    ST_MACH_LOCKOUT,
    ST_WAIT_MECO,
    ST_WAIT_APOGEE,
    ST_WAIT_DROGUE,
    ST_WAIT_MAIN,
    ST_WAIT_GROUND,
    ST_DONE
} FlightPhase;


float vector_magnitude(float x, float y, float z);

float mean(int size, float *arr);

int min(int x, int y);

int max(int x, int y);

int buffer_lt(float* buf, int size, float search_value);

int buffer_gt(float* buf, int size, float search_value);

int buffer_eq(float* buf, int size, float search_value);


float fix_reading(float reading, float* buf, int size, DetectorType type);

void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt, int size);

// Safely inserts data using a circular buffer. 
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type);

int detect_event(Detector * detector, FlightPhase phase);

void compute_fallback_times(float altitude, float velocity, float accel,
                                uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time);

int detect_acceleration_spike(Detector * detector, float* max_accel_seen);

#endif