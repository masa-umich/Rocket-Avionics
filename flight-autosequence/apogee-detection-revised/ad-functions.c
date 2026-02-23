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

int detect_event(Detector * detector, FlightPhase phase) {
    if (phase == ST_WAIT_MECO)
        return buffer_lt(detector->average, detector->avg_size, 0);

    if (phase == ST_WAIT_APOGEE)
        return buffer_lt(detector->slope, detector->slope_size, 0);

    if (phase == ST_WAIT_DROGUE)                  //approximate pressure in hPa at 1500 m
        return buffer_gt(detector->average, detector->avg_size, DROGUE_DEPLOY_PRESSURE); 

    if (phase == ST_WAIT_MAIN)               //aproximate pressure in hPa at 300 m       
        return buffer_gt(detector->average, detector->avg_size, MAIN_DEPLOY_PRESSURE);  

    if (phase == ST_WAIT_GROUND)
        return buffer_eq(detector->slope, detector->slope_size, 0);

    return 0;
}

//Accepts temperature in Kelvin
//Returns sp of sound in m/s
float speed_of_sound(float temp) {
    return sqrtf(GAMMA * R_AIR * temp);
}

//Accepts temperatures in Kelvin
//Accepts pressures in hPa (converts to Pa)
//Returns density of air in kg/m^3 using ideal gas law
float compute_rho(float pressure, float temp) {             
    return ((pressure * 100.0f) * MOLAR_MASS_AIR) / (R_GAS_CONST * temp);
}

//Accepts pressure in hPa
//Accepts temperature in Kelvin
//Returns height in meters using 1) tropospheric formula and 2) isothermal formula above tropopause
//Currently only relying on pressure, but can also be found from temp if desired. 

// TO-DO: adjust temperature and pressure at sea level with measured values
float compute_height(float avg_pressure /*, float avg_temp*/) {
    float estimated_height;

    // CT: is there any way to include a phase where the static pressure and temperature are used
    // to calculate the standard atmosphere? It is all based on an offset from some altitude.
    /*
        The standard atmosphere here is calculate from sea-level but I think it is more accurate
        to calculate it from the launch site. This means implementing something that does
        something like the following:

        - At the beginning of flight-autosequence, pressure and temp are read and stored into
        the constants seen here to use the standard atmosphere to estimate altitude.
        
        - This would require that this function is changed to compute standard atmosphere based
        off those readings. This might mean that you would have to calculate the offsets at sea
        level with those measurements.

        - this would overall make it more accurate because there can be high and low pressure systems
        that can make altitude estimates up to 200 meters off or something like that
    */

    if (avg_pressure < P_TROPOPAUSE){
        estimated_height = (T_SEA_LEVEL / LAPSE_RATE) * (1.0f - powf(avg_pressure / P_SEA_LEVEL, (R_AIR * LAPSE_RATE) / G));
    }

    else {
        estimated_height = ALT_TROPOPAUSE + (R_AIR * T_TROPOPAUSE / G) * logf(P_TROPOPAUSE / avg_pressure);
    }
    
    return estimated_height;
}



//wait-time helper 
float advance_chunk(float *h, float *v, float chunk_dt, float press, float temp)
{
    const float local_s_of_s  = speed_of_sound(temp);
    const float rho = compute_rho(press, temp); // air density           
    const float drag_accel_coeff = max(1e-9f, 0.5f * rho * CD * CROSS_SECT_AREA / MASS_AT_MECO); // guard

    // Helper lambdas for closed-form v(t)
    const float s      = sqrtf(drag_accel_coeff/G);
    const float omega  = sqrtf(G*drag_accel_coeff);

    //CLOSED FORM SOLUTION to dv/dt = -g -(drag_coeff_accel)v^2
    //Since we are precomputing drag_coeff_accel once per chunk, we treat it as constant.
    //This simplifies the above differential equation allowing us to come to this clean
    //solution that we can simply evaluate once per chunk! 
    //*******************************************************/
    #define vel_at(t) (sqrtf(G/drag_accel_coeff) * tanf(atanf(s*(*v)) - omega*(t)))
    //*******************************************************/

    // Time (with this drag_accel_coeff) to drop to local Mach 1 speed
    //This is just solving the above closed form solution for t 
    const float t_to_mach = max(0.0f, (atanf(s*(*v)) - atanf(s*local_s_of_s)) / omega);

    // Use at most chunk_dt this pass
    const float t_used = min(chunk_dt, t_to_mach);
    const float v_new  = vel_at(t_used);

    // Altitude update: trapezoid (good within a short chunk)
    const float dh = 0.5f * ((*v) + v_new) * t_used;

    //Update 
    *h += dh;
    *v  = v_new;
    return t_used; // caller decides if we reached Mach 1
}

//wait-time helper 
float wait_time_piecewise(float h0, float v0, float pressure, float temp, float chunk_dt, float max_time) {
    float t_tot = 0.0, h = h0, v = v0;
    if (fabsf(v) <= speed_of_sound(temp)) // TO-DO CHECK SKIN TC ON NOSECONE
        return 0.0f;

    for (int i = 0 ; i < (int)ceilf(max_time/chunk_dt); ++i)
    {
        const float local_s_of_s = speed_of_sound(temp);
        if(fabsf(v) <= local_s_of_s)
            break;
        
        float used = advance_chunk(&h, &v, chunk_dt, pressure, temp);
        t_tot += used;

        // If we hit Mach within this chunk (used < chunk_dt), stop.
        if (used + 1e-9f < chunk_dt) 
            break;
    }
    return t_tot;
}


float compute_wait_time(int meco_time_ms, float avg_pressure, float avg_temp)
{
    //Explanation: 
    //Goal here is to use integer division to index into a 
    //LUT to get estimate of current speed. 
    //Current LUT has 4 estimates per second for 30 seconds.
    //We're expecting our meco_time in ms, so meco_time_ms / 250 will corresponds to 
    //and index in the table
    //Example meco_time_ms = 10000 (or 10 seconds), 100000 /250 = index 40 in the table, i.e.  
    float current_speed = speed_LUT[(int)(meco_time_ms / 250)];

    float current_height = compute_height(avg_pressure /*,avg_temp*/);

    float wait_s = wait_time_piecewise(current_height, current_speed, avg_pressure, avg_temp, 0.5f, 20.0f);

    return wait_s * 1000;
}


// kinematics-based estimation of fallback times to apogee, 5km, and 1km
float compute_fallback_times(float altitude, float velocity, float accel,
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



