#include "ad-functions.h"
#include <math.h>

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
            detector->slope[detector->slope_i] =
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
    float scale_height = R_GAS_CONST * LAPSE_RATE / (G * MOLAR_MASS_AIR);

    if (avg_pressure < P_TROPOPAUSE){
        estimated_height = (T_GROUND / LAPSE_RATE) * (1.0f - powf(avg_pressure / P_GROUND, scale_height));
    }

    else {
        estimated_height = ALT_TROPOPAUSE + scale_height * logf(P_TROPOPAUSE / avg_pressure);
    }
    
    return estimated_height;
}


// kinematics-based estimation of fallback times to apogee, 5km, and 1km
// void because isn't supposed to return anything (pointers and whatnot)
void compute_fallback_times(float altitude, float velocity, float accel,
                                float *apogee_time, float *five_k_time, float *one_k_time) {
    // time to apogee
    float t_apogee = (-velocity)/accel;
    *apogee_time = t_apogee;

    // time to 5km altitude
    float t_5k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 5000.0f))) / accel;
    *five_k_time = t_5k;

    // time to 1km altitude
    float t_1k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 1000.0f))) / accel;
    *one_k_time = t_1k;
}


float compute_pressure(float altitude, float ground_temp, float ground_pressure) {
    float exp = G * MOLAR_MASS_AIR / (R_GAS_CONST * LAPSE_RATE);
    return ground_pressure * powf(1 - (LAPSE_RATE * altitude) / ground_temp, exp);
}


int detect_acceleration_spike(Detector* detector, float max_accel_seen) {
    if (detector->avg_size < AD_CAPACITY)
        return 0;

    float filtered_accel = detector->average[(detector->avg_index - 1 + AD_CAPACITY) % AD_CAPACITY];
    if (filtered_accel > max_accel_seen && filtered_accel > MIN_START_ACCEL)
        max_accel_seen = filtered_accel;

    if (max_accel_seen > MIN_START_ACCEL) {
        float accel_drop = max_accel_seen - filtered_accel;
        if (accel_drop > MACH_DIP_THRESHOLD)
            return 1;
    }
    
    return 0;

}


