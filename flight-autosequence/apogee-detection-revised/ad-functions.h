#ifndef AD_FUNCTIONS_H
#define AD_FUNCTIONS_H

//size of the array of readings we maintain during the flight
//used with the barometer and the IMU
#define AD_CAPACITY 5          

const float CROSS_SECT_AREA = 0.08f;            // m^2
const float MASS_AT_MECO = 255.0f;              // kg
const float CD = 0.50f;                         //drag coefficient
const float G = 9.8f;                           // gravity
const float MOLAR_MASS_AIR = 0.02896f;          // kg/mol
const float R_GAS_CONST = 8.315f;               // gas constant
const float GAMMA = 1.4f;

const float DROGUE_DEPLOY_ALTITUDE = 1500.0f;     // m, for reference
const float MAIN_DEPLOY_ALTITUDE = 300.0f;       // m, for reference


// TO BE SET AT LAUNCH
float GROUND_ALTITUDE = 700.0f;     // meters above sea level at FAR
float DROGUE_DEPLOY_PRESSURE = 825.0f;    // hPa, to be set at launch
float MAIN_DEPLOY_PRESSURE = 975.0f;      // hPa, to be set at launch
float LAPSE_RATE = 0.0065f;               // Celsius per meter
float P_GROUND = 1013.25f;             // hPa 
float T_GROUND = 293.0f;               // Kelvin (20 deg C)
float P_TROPOPAUSE = 226.32f;             // hPa  // TO-DO DOUBLE CHECK THIS
float T_TROPOPAUSE = 216.65f;             // Kelvin (56.5 deg C)
float ALT_TROPOPAUSE = 11000.0f;              // meters



// THRESHOLD VALUES FOR BAD DATA
const float ACCEL_MIN = -20.0f; // m/s^2
// no max accel threshold because accel readings during burn will be very high

const float BARO_MIN = 0.0f; // hPa
const float BARO_MAX = 1100.0f; // hPa TODO: adjust max based on expected max altitude

const float TEMP_C_MIN = -50.0f; // Celsius
const float TEMP_C_MAX = 50.0f; // Celsius
const float TEMP_K_MIN = TEMP_C_MIN + 273.15f; // Kelvin
const float TEMP_K_MAX = TEMP_C_MAX + 273.15f; // Kelvin


const float MACH_DIP_THRESHOLD = 15.0f; // drop in acceleration in m/s^2 that indicates transition from subsonic to supersonic
const float MIN_START_ACCEL = 20.0f; // minimum accel we need to see to consider detecting a spike for mach 1 transition



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

    float slope[AD_CAPACITY];
    int slope_i;
    int slope_size;
} Detector;

//What 'state' is our apogee-detection system in? 
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

float compute_fallback_times(float altitude, float velocity, float accel,
                                float *apogee_time, float *five_k_time, float *one_k_time);

int detect_acceleration_spike(Detector * detector, float max_accel_seen);

#endif