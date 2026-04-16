#include "ad-functions-test.h"

// Note: Ensure your implementation of fix_reading, min, mean, buffer_lt, buffer_gt, 
// and buffer_eq are defined in your project or included here!

const float CROSS_SECT_AREA = 0.08f;            // m^2
const float MASS_AT_MECO = 255.0f;              // kg
const float CD = 0.50f;                         // drag coefficient
const float G = 9.8f;                           // gravity
const float MOLAR_MASS_AIR = 0.02896f;          // kg/mol
const float R_GAS_CONST = 8.315f;               // gas constant
const float GAMMA = 1.4f;

const float DROGUE_DEPLOY_ALTITUDE = 1500.0f;     // m
const float MAIN_DEPLOY_ALTITUDE = 300.0f;        // m
float GROUND_ALTITUDE = 700.0f;                   // m


// THRESHOLD VALUES FOR BAD DATA
const float ACCEL_MIN = -20.0f; // m/s^2

const float ALT_MIN = -500.0f;  // meters (allow for some negative drift if starting at 0)
const float ALT_MAX = 50000.0f; // meters 

const float TEMP_C_MIN = -50.0f; // Celsius
const float TEMP_C_MAX = 50.0f;  // Celsius
const float TEMP_K_MIN = TEMP_C_MIN + 273.15f; // Kelvin
const float TEMP_K_MAX = TEMP_C_MAX + 273.15f; // Kelvin

const float MACH_DIP_THRESHOLD = 15.0f; 
const float MIN_START_ACCEL = 20.0f; 
const float MECO_MAX_DROP_RATE = 15.0f;


void insert(Detector * detector, float reading1, float reading2, FlightPhase phase, DetectorType type) {
    // Data imputation
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
                 mean(detector->size_2, detector->readings_2)) / 2.0f;        
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

    if (phase == ST_WAIT_APOGEE) {
        // With altitude, a negative slope (< 0) correctly indicates the rocket is falling!
        fprintf(stderr, "Checking for apogee: Current slope buffer: ");
        for (int i = 0; i < detector->slope_size; ++i) {
            fprintf(stderr, "%f ", detector->slope[i]);
        }
        return buffer_lt(detector->slope, detector->slope_size, 0.0f);
    }

    if (phase == ST_WAIT_DROGUE)                  
        // We look for altitude dropping BELOW the drogue deployment height
        return buffer_lt(detector->average, detector->avg_size, DROGUE_DEPLOY_ALTITUDE); 

    if (phase == ST_WAIT_MAIN)                     
        // We look for altitude dropping BELOW the main deployment height
        return buffer_lt(detector->average, detector->avg_size, MAIN_DEPLOY_ALTITUDE);  

    if (phase == ST_WAIT_GROUND)
        // Altitude slope hits 0 (no longer changing)
        return buffer_eq(detector->slope, detector->slope_size, 0);

    return 0;
}


void compute_fallback_times(float altitude, float velocity, float accel,
                uint32_t *apogee_time, uint32_t *five_k_time, uint32_t *one_k_time) {
    // time to apogee
    float t_apogee = (-velocity)/accel;
    *apogee_time += (uint32_t)(t_apogee * 1000);

    // time to 1.5km altitude
    float t_5k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 1500.0f))) / accel;
    *five_k_time += (uint32_t)(t_5k * 1000);

    // time to 300m altitude
    float t_1k = (-velocity + sqrtf(velocity*velocity + 2*accel*(altitude - 300.0f))) / accel;
    *one_k_time += (uint32_t)(t_1k * 1000);
}


int detect_acceleration_spike(Detector* detector, float* max_accel_seen) {
    if (detector->avg_size < AD_CAPACITY)
        return 0;

    float current_accel = detector->average[(detector->avg_index - 1 + AD_CAPACITY) % AD_CAPACITY];

    // Update max acceleration seen
    if (current_accel > *max_accel_seen && current_accel > MIN_START_ACCEL) {
        *max_accel_seen = current_accel;
    }

    if (*max_accel_seen > MIN_START_ACCEL) {
        if (current_accel < 0.0f) {
            *max_accel_seen = MIN_START_ACCEL;
            return 0;
        }

        float accel_drop = *max_accel_seen - current_accel;
        
        if (accel_drop > MACH_DIP_THRESHOLD) {
            
            // --- THE BUFFER CHECK ---
            // Look back 'N' samples (e.g., 5 ticks ago) to check the slope
            int LOOKBACK_TICKS = 5; 
            int past_index = (detector->avg_index - 1 - LOOKBACK_TICKS + AD_CAPACITY) % AD_CAPACITY;
            float past_accel = detector->average[past_index];
            
            // Calculate the rate of change (jerk)
            float drop_rate = past_accel - current_accel; 
            
            // If the drop rate is too massive, it's a MECO cliff, not a Mach wave!
            if (drop_rate < MECO_MAX_DROP_RATE) {
                fprintf(stderr, "Acceleration spike detected! Max accel: %f, Current accel: %f, Drop: %f, Drop rate: %f\n", 
                        *max_accel_seen, current_accel, accel_drop, drop_rate);
                *max_accel_seen = MIN_START_ACCEL;
                return 1; // Valid Mach dip detected!
            } else {
                // It's dropping too fast (likely MECO), ignore it
                return 0; 
            }
        }
    }
    
    return 0;
}

void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt, int size){
    int N = size;

    float sum_t = 0.0f;
    float sum_t2 = 0.0f;

    float sum_h = 0.0f;
    float sum_ht = 0.0f;
    float sum_ht2 = 0.0f;
    float sum_t4 = 0.0f;

    float t0 = time_arr[N/2];
    float h0 = altitude_arr[N/2];

    for (int i = 0; i < N; ++i) {
        float t = time_arr[i] - t0; 
        float h = altitude_arr[i] - h0;
        float t2 = t * t;

        sum_t += t;
        sum_t2 += t * t;
        sum_h += h;
        sum_ht += h * t;
        sum_ht2 += h * t2;
        sum_t4 += t2*t2;
    }

    float denominator = (N * sum_t4) - (sum_t2 * sum_t2);

    if (fabsf(denominator) < 1e-2f || fabsf(sum_t2) < 1e-2f) {
        *inst_accel = 0.0f; 
        *vel = 0.0f; 
        *alt = altitude_arr[N/2];
        return; 
    }

    float a = (N * sum_ht2 - sum_h * sum_t2) / denominator;
    float b = sum_ht / sum_t2;
    float c = (sum_h - a * sum_t2) / N;

    *inst_accel = 2.0f * a;
    *vel = b;
    *alt = c + h0;
}

float vector_magnitude(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

float mean(int size, float *arr) {
    if (size == 0)
        return 0.0f;
    float sum = 0.0f;
    for(int i = 0; i < size; ++i)
        sum += arr[i];

    return (sum / (float)size);
}

int min(int x, int y) {
    if(x < y)
        return x;
    else return y;
}

int max(int x, int y) {
    if(x > y)
        return x;
    else return y;
}

int buffer_lt(float* buf, int size, float search_value) {
    if (size < AD_CAPACITY)
        return 0;

    int count = 0;
    for (int i = 0; i < size; ++i)
    {
        if (buf[i] < search_value)
            ++count;
    }
    if (count >= (size * 0.8))
        return 1;
    return 0;
}

int buffer_gt(float* buf, int size, float search_value) {
    if (size < AD_CAPACITY)
        return 0;

    int count = 0;
    for (int i = 0; i < size; ++i)
    {
        if (buf[i] > search_value)
            ++count;
    }
    if (count >= (size * 0.8))
        return 1;
    return 0;
}

int buffer_eq(float* buf, int size, float search_value) {
    if (size < AD_CAPACITY)
        return 0;

    int count = 0;
    for (int i = 0; i < size; ++i)
    {
        if (fabsf(buf[i] - search_value) < 1e-1f) // consider equal if within 0.1
            ++count;
    }
    if (count >= (size * 0.8))
        return 1;
    return 0;
}


float fix_reading(float reading, float* buf, int size, DetectorType type) {
    switch (type) {
        case ACCEL_DTR:
            if (reading <= ACCEL_MIN)
                return mean(size, buf);
            return reading;

        case ALT_DTR:
            if (reading <= ALT_MIN || reading >= ALT_MAX)
                return mean(size, buf);
            return reading;

        case TEMP_DTR:
            if (reading <= TEMP_K_MIN || reading >= TEMP_K_MAX)
                return mean(size, buf);
            return reading;

        default:
            return reading;
    }
}
