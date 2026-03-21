#include "ad-functions.h"

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

//If we 'miss' a reading from a sensor, don't pass in bad data! Pass 0.0f
//Reason: this insert module will be used for barometer temp, barometer pressure, 
//and IMU x acceleration (up)
//If either of those drivers misses data from the sensor it should send over 0.0f.  
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type) {

    // Data imputation: if we miss a reading, we can still insert something reasonable into our buffers
    float corrected_reading1 = fix_reading(reading1, detector->readings_1, detector->size_1, type);
    detector->readings_1[detector->index_1] = corrected_reading1;
    detector->index_1++;
    detector->index_1 = detector->index_1 % AD_CAPACITY;
    detector->size_1 = min(detector->size_1 + 1, AD_CAPACITY);

    // Same imputation for reading 2
    float corrected_reading2 = fix_reading(reading2, detector->readings_2, detector->size_2, type);  
    detector->readings_2[detector->index_2] = corrected_reading2;
    detector->index_2++;
    detector->index_2 = detector->index_2 % AD_CAPACITY;
    detector->size_2 = min(detector->size_2 + 1, AD_CAPACITY);

    // Update average buffer    
    detector->average[detector->avg_index] = 
                (mean(detector->size_1, detector->readings_1) + 
                 mean(detector->size_2, detector->readings_2)) / 2;        
    detector->avg_index++;
    detector->avg_index = detector->avg_index % AD_CAPACITY;
    detector->avg_size = min(detector->avg_size + 1, AD_CAPACITY);

    if (phase == ST_MACH_LOCKOUT || phase == ST_WAIT_APOGEE || phase == ST_WAIT_GROUND)
    {
        if (detector->avg_size >= AD_CAPACITY)
        {
            detector->slope[detector->slope_i] = // TODO check slope calculation  
                (detector->average[(detector->avg_index + AD_CAPACITY - 1) % AD_CAPACITY] -
                 detector->average[(detector->avg_index + AD_CAPACITY - AD_CAPACITY) % AD_CAPACITY]) /
                AD_CAPACITY;
            
            detector->slope_size = min(detector->slope_size + 1, AD_CAPACITY);
            detector->slope_i = (detector->slope_i + 1) % AD_CAPACITY;
        }
    }
}


// if we detect an event, return 1, else return 0
// events are specified by the current phase of the flight
// MECO     ->
// Apogee   ->
// 5k feet  ->
// 1k feet  ->
// Ground   ->
int detect_event(Detector * detector, FlightPhase phase) {
    if (phase == ST_WAIT_MECO)
        return buffer_lt(detector->average, detector->avg_size, 0);

    if (phase == ST_WAIT_APOGEE)
        return buffer_lt(detector->slope, detector->slope_size, 0);

    if (phase == ST_WAIT_DROGUE)                  
        return buffer_gt(detector->average, detector->avg_size, DROGUE_DEPLOY_PRESSURE); 

    if (phase == ST_WAIT_MAIN)                     
        return buffer_gt(detector->average, detector->avg_size, MAIN_DEPLOY_PRESSURE);  

    if (phase == ST_WAIT_GROUND)
        return buffer_eq(detector->slope, detector->slope_size, 0);

    return 0;
}


//Accepts pressure in hPa
//Accepts temperature in Kelvin
//Returns height in meters using 
// 1) tropospheric formula below tropopause, and
// 2) isothermal formula above tropopause
float compute_height(float avg_pressure) {
    float estimated_height;
    float scale_height_trop = R_GAS_CONST * LAPSE_RATE / (G * MOLAR_MASS_AIR);
    float scale_height_iso = R_GAS_CONST * T_TROPOPAUSE / (G * MOLAR_MASS_AIR);

    if (avg_pressure < P_TROPOPAUSE){
        estimated_height = (T_GROUND / LAPSE_RATE) * (1.0f - powf(avg_pressure / P_GROUND, scale_height_trop));
    }

    else {
        estimated_height = ALT_TROPOPAUSE + scale_height_iso * logf(P_TROPOPAUSE / avg_pressure);
    }
    
    return estimated_height;
}


// kinematics-based estimation of fallback times to apogee, 5km, and 1km
// void because isn't supposed to return anything (pointers and whatnot)
void compute_fallback_times(float altitude, float velocity, float accel,
				uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time) {
    // time to apogee
    float t_apogee = (-velocity)/accel;
    *apogee_time += t_apogee * 1000;

    // time to 5km altitude
    float t_5k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 5000.0f))) / accel;
    *five_k_time += t_5k * 1000;

    // time to 1km altitude
    float t_1k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 1000.0f))) / accel;
    *one_k_time += t_1k * 1000;
}


float compute_pressure(float altitude, float ground_temp, float ground_pressure) {
    float exp = G * MOLAR_MASS_AIR / (R_GAS_CONST * LAPSE_RATE);
    return ground_pressure * powf(1 - (LAPSE_RATE * altitude) / ground_temp, exp);
}


int detect_acceleration_spike(Detector* detector, float* max_accel_seen) {
    if (detector->avg_size < AD_CAPACITY)
        return 0;

    float filtered_accel = detector->average[(detector->avg_index - 1 + AD_CAPACITY) % AD_CAPACITY];
    if (filtered_accel > *max_accel_seen && filtered_accel > MIN_START_ACCEL)
        *max_accel_seen = filtered_accel;

    if (*max_accel_seen > MIN_START_ACCEL) {
        float accel_drop = *max_accel_seen - filtered_accel;
        if (accel_drop > MACH_DIP_THRESHOLD){
            *max_accel_seen = MIN_START_ACCEL;
            return 1;
        }
    }
    
    return 0;

}


