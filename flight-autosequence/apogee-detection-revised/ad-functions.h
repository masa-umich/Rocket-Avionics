#ifndef AD_FUNCTIONS_H
#define AD_FUNCTIONS_H

#include "stdint.h"
#include "ad-helpers.h"
#include <math.h>

//used with the barometer and the IMU
#define AD_CAPACITY 21        

typedef enum {
    IMU_DTR = 0,
    BARO_DTR,
    TEMP_DTR
} DetectorType;


// DETECTOR STRUCTURE
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

    float slope;
} Detector;


// FLIGHT PHASE DEFINITIONS
typedef enum {
	ST_DISARMED = 0x00,
    ST_DETECT_VALVES_OPEN = 0x01,
    ST_WAIT_MECO = 0x02,
    ST_MACH_LOCKOUT = 0x03,
    ST_WAIT_APOGEE = 0x04,
    ST_WAIT_DROGUE = 0x05,
    ST_WAIT_MAIN = 0x06,
    ST_WAIT_GROUND = 0x07,
    ST_DONE = 0x08
} FlightPhase;


// BOUNDS FOR GOOD READINGS
extern const float ACCEL_MIN;                           // m/s^2
extern const float ACCEL_MAX;                           // m/s^2

extern const float BARO_MIN;                            // hPa
extern const float BARO_MAX;                            // hPa TODO: adjust max based on expected max altitude

extern const float TEMP_C_MIN;                          // Celsius
extern const float TEMP_C_MAX;                          // Celsius


// TERMINAL VELOCITY ESTIMATES FOR FALLBACK TIMER CALCULATIONS
extern const float PILOT_TERM_VEL;                      // m/s, MODIFY
extern const float DROGUE_TERM_VEL;                     // m/s, MODIFY

      
// PHYSICAL CONSTANTS
extern const float G;                                   // gravity
extern const float MOLAR_MASS_AIR;                      // kg/mol
extern const float R_GAS_CONST;                         // gas constant
extern const float GAMMA;
extern const float LAPSE_RATE;                          // Celsius per meter


// DEPLOYMENT ALTITUDE DEFINITIONS
extern const float DROGUE_DEPLOY_ALTITUDE;              // m, for reference
extern const float MAIN_DEPLOY_ALTITUDE;                // m, for reference

// DEPLOYMENT PRESSURES AND GROUND PRESSURE AND TEMP VARIABLES - SET IN STATE 1
extern float DROGUE_DEPLOY_PRESSURE;                    // hPa, to be set at launch
extern float MAIN_DEPLOY_PRESSURE;                      // hPa, to be set at launch
extern float P_GROUND;                                  // hPa
extern float T_GROUND;                                  // Kelvin (20 deg C)

//Accepts pressure in hPa
//Returns height in meters using troposphere model
float compute_height(float avg_pressure);


// takes altitude in meters
// returns pressure in hPa using troposphere model
float compute_pressure(float altitude);


// inserts data into detector buffer
// resistant to bad readings through imputation
// updates slope with linear regression on moving average
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type);


// if we detect an event, return 1, else return 0
// events are specified by the current phase of the flight
// MECO     -> acceleration drops below 0
// Apogee   -> slope of pressure buffer exceeds 0 (pressure inversely proportional to altitude)
// 5k feet  -> pressure rises above drogue deploy pressure
// 1k feet  -> pressure rises above main deploy pressure
// Ground   -> pressure rises above ground pressure
int detect_event(Detector * detector, FlightPhase phase);


// kinematics-based estimation of fallback times to apogee, 5km, and 1km
// Apogee time: t = -v/a
// 5k time: t = t_apogee + (h_apogee - h_drogue) / pilot_term_vel 
// 1k time: t = t_5k + (h_5k - h_main) / drogue_term_vel
void compute_fallback_times(float altitude, float velocity, float accel,
                            uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time,
                            float* h_apogee, float* h_5k, float* h_1k);


// resets all values in the detector to 0 and resets indices and sizes
void clear(Detector * detector);


// if we get a reading that is out of bounds, 
// replace it with the mean of the buffer to avoid throwing off our detector with bad data
float fix_reading(float reading, float* buf, int size, DetectorType type);

#endif
