#ifndef AD_FUNCTIONS_H
#define AD_FUNCTIONS_H

//size of the array of readings we maintain during the flight
//used with the barometer and the IMU
#define AD_CAPACITY 5          

const float CROSS_SECT_AREA = 0.08f;            // m^2
const float MASS_AT_MECO = 255.0f;              // kg
const float CD = 0.50f;                         //drag coefficient

const float P_SEA_LEVEL = 1013.25f;             // hPa 
const float P_TROPOPAUSE = 226.32f;             // hPa  // TO-DO DOUBLE CHECK THIS

const float T_SEA_LEVEL = 293.0f;               // Kelvin (20 deg C)
const float T_TROPOPAUSE = 216.65f;             // Kelvin (56.5 deg C)

const float ALT_TROPOPAUSE = 11000.0f;              // meters

const float LAPSE_RATE = 0.0065f;               // Celsius per meter
const float G = 9.8f;                           // gravity
const float MOLAR_MASS_AIR = 0.02896f;          // kg/mol
const float R_GAS_CONST = 8.315f;               // gas constant
const float GAMMA = 1.4f;
const float R_AIR = R_GAS_CONST / MOLAR_MASS_AIR; // specific gas constant for air

const float DROGUE_DEPLOY_PRESSURE = 825.0f;    // hPa
const float MAIN_DEPLOY_PRESSURE = 975.0f;      // hPa



// THRESHOLD VALUES FOR BAD DATA
const float ACCEL_MIN = -20.0f; // m/s^2
// no max accel threshold because accel readings during burn will be very high

const float BARO_MIN = 0.0f; // hPa
const float BARO_MAX = 1100.0f; // hPa TODO: adjust max based on expected max altitude

const float TEMP_C_MIN = -50.0f; // Celsius
const float TEMP_C_MAX = 50.0f; // Celsius
const float TEMP_K_MIN = TEMP_C_MIN + 273.15f; // Kelvin
const float TEMP_K_MAX = TEMP_C_MAX + 273.15f; // Kelvin

typedef enum {
    IMU_DTR = 0,
    BARO_DTR,
    TEMP_DTR
} DetectorType;

const float speed_LUT[120] = {0, 5.92027, 11.9846, 17.8747, 23.6646, 29.3443, 34.7899, 
                              40.4716, 46.1715, 51.8967, 57.7432, 63.66, 69.6572, 75.746, 
                              81.9011, 88.1425, 94.4895, 100.859, 107.304, 113.838, 120.417, 
                              127.096, 133.833, 140.689, 147.592, 154.543, 161.601, 168.693, 
                              175.833, 182.962, 190.163, 197.416, 204.685, 211.947, 219.258, 
                              226.584, 233.874, 241.168, 248.482, 255.813, 263.108, 270.407, 
                              277.714, 284.978, 292.236, 299.478, 306.709, 313.883, 320.889, 
                              327.615, 334.117, 340.352, 346.334, 352.037, 357.52, 362.999, 
                              368.434, 373.858, 379.223, 384.564, 389.905, 395.225, 400.549, 
                              405.868, 411.182, 416.518, 421.788, 427.037, 432.333, 437.661, 
                              442.938, 448.239, 453.532, 458.846, 464.152, 469.426, 474.702, 
                              480.014, 485.305, 490.624, 495.996, 501.314, 506.669, 512.026, 
                              517.379, 522.785, 528.224, 533.692, 538.807, 536.463, 536.178, 
                              538.42, 539.204, 534.973, 529.894, 524.621, 519.236, 513.794, 
                              508.41, 503.133, 497.979, 492.911, 487.96, 483.098, 478.327, 
                              473.642, 469.021, 464.465, 459.968, 455.493, 451.061, 446.695, 
                              442.373, 438.06, 433.783, 429.515, 425.269, 421.087, 416.931, 412.859};


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

//Accepts temperatures in Kelvin
//Accepts pressures in hPa (converts to Pa)
float compute_rho(float pressure, float temp);

//Accepts temperature in Kelvin
//Returns sp of sound in m/s
float speed_of_sound(float temp);

//Estimate our height given pressure and temp
//Expects hPa and Kelvin
float compute_height(float avg_pressure /*,float avg_temp*/);

//Safely inserts data from barometers & imus using a circular buffer. 
//If we 'miss' a reading from a sensor, don't pass in bad data! Pass 0.0f
//Reason: this insert module will be used for barometer temp, barometer pressure, 
//and IMU x acceleration (up), and relies on imputation to handle missed readings.
//If the IMU or barometer driver misses data from the sensor it should pass 0.0f. 
//to the Flight_Computer_State_t in main.c so the data is good by the time it reaches here
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type);

//wait-time helper 
float advance_chunk(float *h, float *v, float chunk_dt, float press, float temp);

//wait-time helper 
float wait_time_piecewise(float h0, float v0, float pressure, float temp, float chunk_dt, float max_time);

//computes the time the rocket will take to drop back below mach 1
//needs meco_time -> to index into speed_LUT for estimated velocity
float compute_wait_time(int meco_time_ms, float avg_pressure, float avg_temp);

//phase 3
int detect_event(Detector * detector, FlightPhase phase);

float compute_fallback_times(float altitude, float velocity, float accel,
                                float *apogee_time, float *five_k_time, float *one_k_time);

#endif