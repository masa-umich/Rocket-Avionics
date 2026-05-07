#include "ad-functions.h"


// BOUNDS FOR GOOD READINGS
const float ACCEL_MIN = -200.0f;                            // m/s^2
const float ACCEL_MAX = 200.0f;                             // m/s^2

const float BARO_MIN = 50.0f;                              // hPa
const float BARO_MAX = 1200.0f;                             // hPa TODO: adjust max based on expected max altitude

const float TEMP_C_MIN = -40.0f;                            // Celsius
const float TEMP_C_MAX = 60.0f;                             // Celsius


// TERMINAL VELOCITY ESTIMATES FOR FALLBACK TIMER CALCULATIONS
const float PILOT_TERM_VEL = 68.0f;                          // m/s, DOUBLE CHECK
const float DROGUE_TERM_VEL = 22.0f;                         // m/s, DOUBLE CHECK


// PHYSICAL CONSTANTS
const float G = 9.80665f;                                   // m/s^2, gravity
const float MOLAR_MASS_AIR = 0.02896f;                      // kg/mol
const float R_GAS_CONST = 8.314463f;                        // gas constant
const float GAMMA = 1.4f;                                   // specific heat ratio for air
const float LAPSE_RATE = 0.0065f;                           // Celsius per meter


// DEPLOYMENT ALTITUDE DEFINITIONS
const float DROGUE_DEPLOY_ALTITUDE = 1524.0f + 75.0f;               // m, for reference
const float MAIN_DEPLOY_ALTITUDE = 304.8f + 75.0f;                  // m, for reference


// DEPLOYMENT PRESSURES AND GROUND PRESSURE AND TEMP VARIABLES - SET IN STATE 1
float DROGUE_DEPLOY_PRESSURE = 825.0f;                      // hPa, to be set at ignition
float MAIN_DEPLOY_PRESSURE = 975.0f;                        // hPa, to be set at ignition
float P_GROUND = 934.57f;                                  // hPa, to be set at ignition, default ground pressure at 25C and 700 m (FAR)
float T_GROUND = 25.0f;                                     // Celsius, to be set at ignition


// inserts data into detector buffer
// resistant to bad readings through imputation
// updates slope with linear regression on moving average
void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type) {

    // Data imputation: if we miss a reading, we can still insert something reasonable into our buffers
    float corrected_reading1 = fix_reading(reading1, detector->readings_1, detector->size_1, type);

    // insert into buffer and update size and index
    detector->readings_1[detector->index_1] = corrected_reading1;
    detector->index_1 = (detector->index_1 + 1) % AD_CAPACITY;
    detector->size_1 = min(detector->size_1 + 1, AD_CAPACITY);

    // Same imputation for reading 2
    float corrected_reading2 = fix_reading(reading2, detector->readings_2, detector->size_2, type);  

    // insert into buffer and update size and index
    detector->readings_2[detector->index_2] = corrected_reading2;
    detector->index_2 = (detector->index_2 + 1) % AD_CAPACITY;
    detector->size_2 = min(detector->size_2 + 1, AD_CAPACITY);

    // Update average buffer based on mean of the two reading buffers
    detector->average[detector->avg_index] = 
                (mean(detector->size_1, detector->readings_1) + 
                 mean(detector->size_2, detector->readings_2)) / 2;        
    detector->avg_index = (detector->avg_index + 1) % AD_CAPACITY;
    detector->avg_size = min(detector->avg_size + 1, AD_CAPACITY);

    // update slope if we have enough readings - linear regression on mean buffer
    if (detector->avg_size >= AD_CAPACITY) {
        detector->slope = linreg_slope(detector->average, detector->avg_index, AD_CAPACITY);
    }
}


// if we detect an event, return 1, else return 0
// events are specified by the current phase of the flight
// MECO     -> acceleration drops below 0
// Apogee   -> slope of pressure buffer exceeds 0 (pressure inversely proportional to altitude)
// 5k feet  -> pressure rises above drogue deploy pressure
// 1k feet  -> pressure rises above main deploy pressure
// Ground   -> pressure rises above ground pressure
int detect_event(Detector * detector, FlightPhase phase) {
    if (phase == ST_WAIT_MECO)
        return buffer_lt(detector->average, detector->avg_size, AD_CAPACITY, 0.0f);

    if (phase == ST_WAIT_APOGEE)
        return detector->slope > 0.0f; // pressure slope should be positive at apogee since pressure increases as we descend

    if (phase == ST_WAIT_DROGUE)                  
        return buffer_gt(detector->average, detector->avg_size, AD_CAPACITY, DROGUE_DEPLOY_PRESSURE); 

    if (phase == ST_WAIT_MAIN)                     
        return buffer_gt(detector->average, detector->avg_size, AD_CAPACITY, MAIN_DEPLOY_PRESSURE);  

    if (phase == ST_WAIT_GROUND)
        return buffer_gt(detector->average, detector->avg_size, AD_CAPACITY, P_GROUND-50.0f); // margin for error

    return 0;
}


//Accepts pressure in hPa
//Returns height in meters using troposphere model
float compute_height(float avg_pressure) {
    float ground_temp_K = T_GROUND + 273.15f; // convert to Kelvin
    float scale_height_trop = R_GAS_CONST * LAPSE_RATE / (G * MOLAR_MASS_AIR);
    float estimated_height = (ground_temp_K / LAPSE_RATE) * (1.0f - powf(avg_pressure / P_GROUND, scale_height_trop));
    return estimated_height;
}


// kinematics-based estimation of fallback times to apogee, 5km, and 1km
// Apogee time: t = -v/a
// 5k time: t = t_apogee + (h_apogee - h_drogue) / pilot_term_vel 
// 1k time: t = t_5k + (h_5k - h_main) / drogue_term_vel
void compute_fallback_times(float altitude, float velocity, float accel,
                uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time,
                float* h_apogee, float* h_5k, float* h_1k) {
    // time to apogee
    *h_apogee = altitude + (velocity * velocity) / (2.0f * -accel); // h = h0 + v^2/2a
    float t_apogee = (-velocity) / accel;
    *apogee_time += (uint32_t)(t_apogee * 1000);

    // time to 1.5km altitude (descending pass: use minus root)
    float t_5k = t_apogee + (*h_apogee - DROGUE_DEPLOY_ALTITUDE) / PILOT_TERM_VEL; // after apogee, we assume we immediately hit terminal velocity with drogue chute
    *h_5k = DROGUE_DEPLOY_ALTITUDE; 
    if (*h_apogee < DROGUE_DEPLOY_ALTITUDE) {
        t_5k = t_apogee;
        *h_5k = *h_apogee;
    }
    *five_k_time += (uint32_t)(t_5k * 1000);

    // time to 300m altitude (descending pass: use minus root)
    float t_1k = t_5k + (*h_5k - MAIN_DEPLOY_ALTITUDE) / DROGUE_TERM_VEL; // after 5k, we assume we immediately hit terminal velocity with main chute
    *h_1k = MAIN_DEPLOY_ALTITUDE;
    if (*h_5k < MAIN_DEPLOY_ALTITUDE) {
        t_1k = t_5k;
        *h_1k = *h_5k;
    }
    *one_k_time += (uint32_t)(t_1k * 1000);
}

// takes altitude in meters
// returns pressure in hPa using troposphere model
float compute_pressure(float altitude) {
    float ground_temp_K = T_GROUND + 273.15f; // convert to Kelvin
    float exponent = G * MOLAR_MASS_AIR / (R_GAS_CONST * LAPSE_RATE);
    return P_GROUND * powf(1.0f - (LAPSE_RATE * altitude) / ground_temp_K, exponent);; // convert Pa to hPa
}


// resets all values in the detector to 0 and resets indices and sizes
void clear(Detector * detector) {
    for (int i = 0; i < AD_CAPACITY; ++i) {
        detector->readings_1[i] = 0.0f;
        detector->readings_2[i] = 0.0f;
        detector->average[i] = 0.0f;
    }
    detector->index_1 = 0;
    detector->size_1 = 0;
    detector->index_2 = 0;
    detector->size_2 = 0;
    detector->avg_index = 0;
    detector->avg_size = 0;
    detector->slope = 0;
}


// if we get a reading that is out of bounds, 
// replace it with the mean of the buffer to avoid throwing off our detector with bad data
float fix_reading(float reading, float* buf, int size, DetectorType type) {
    switch (type) {
        case IMU_DTR:
            if (reading <= ACCEL_MIN || reading >= ACCEL_MAX)
                return mean(size, buf);
            return reading;

        case BARO_DTR:
            if (reading <= BARO_MIN || reading >= BARO_MAX)
                return mean(size, buf);
            return reading;

        case TEMP_DTR:
            if (reading <= TEMP_C_MIN || reading >= TEMP_C_MAX)
                return mean(size, buf);
            return reading;

        default:
            return reading;
    }
}


