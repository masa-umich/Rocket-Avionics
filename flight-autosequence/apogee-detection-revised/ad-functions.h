#ifndef AD_FUNCTIONS_H
#define AD_FUNCTIONS_H

#include "stdint.h"
#include "ad-helpers.h"
#include <math.h>

//used with the barometer and the IMU
#define AD_CAPACITY 21        

extern const float G;                           // gravity
extern const float MOLAR_MASS_AIR;          // kg/mol
extern const float R_GAS_CONST;               // gas constant
extern const float GAMMA;
extern const float LAPSE_RATE;               // Celsius per meter


extern const float DROGUE_DEPLOY_ALTITUDE;     // m, for reference
extern const float MAIN_DEPLOY_ALTITUDE;       // m, for reference


// TO BE SET AT LAUNCH
extern float DROGUE_DEPLOY_PRESSURE;   // hPa, to be set at launch
extern float MAIN_DEPLOY_PRESSURE;      // hPa, to be set at launch
extern float P_GROUND;             // hPa
extern float T_GROUND;               // Kelvin (20 deg C)

//extern float GROUND_ALTITUDE;     // meters above sea level at FAR
//extern float P_TROPOPAUSE;             // hPa  // TO-DO DOUBLE CHECK THIS
//extern float T_TROPOPAUSE;             // Kelvin (56.5 deg C)
//extern float ALT_TROPOPAUSE;              // meters



// THRESHOLD VALUES FOR BAD DATA
extern const float ACCEL_MIN; // m/s^2
extern const float ACCEL_MAX;  // m/s^2
// no max accel threshold because accel readings during burn will be very high

extern const float BARO_MIN; // hPa
extern const float BARO_MAX; // hPa TODO: adjust max based on expected max altitude

extern const float TEMP_C_MIN; // Celsius
extern const float TEMP_C_MAX; // Celsius

extern const float PILOT_TERM_VEL; // m/s, terminal velocity with drogue chute deployed (for fallback timer calculations)
extern const float DROGUE_TERM_VEL;   // m/s, terminal velocity with main chute deployed (for fallback timer calculations)

typedef enum {
    IMU_DTR = 0,
    BARO_DTR,
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

    float slope;
} Detector;

//What 'state' is our apogee-detection system in?
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


float compute_height(float avg_pressure);

float compute_pressure(float altitude, float ground_temp, float ground_pressure);

//Safely inserts data from barometers & imus using a circular buffer. 
//If we 'miss' a reading from a sensor, don't pass in bad data! Pass 0.0f
//Reason: this insert module will be used for barometer temp, barometer pressure, 
//and IMU x acceleration (up), and relies on imputation to handle missed readings.
//If the IMU or barometer driver misses data from the sensor it should pass 0.0f. 
//to the Flight_Computer_State_t in main.c so the data is good by the time it reaches here
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type);

int detect_event(Detector * detector, FlightPhase phase);

void compute_fallback_times(float altitude, float velocity, float accel,
                            uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time,
                            float* h_apogee, float* h_5k, float* h_1k);

void clear(Detector * detector);

float fix_reading(float reading, float* buf, int size, DetectorType type);

#endif
