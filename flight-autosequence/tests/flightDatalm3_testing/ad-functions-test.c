#include "ad-functions-test.h"

// Note: Ensure your implementation of fix_reading, min, mean, buffer_lt, buffer_gt, 
// and buffer_eq are defined in your project or included here!
const float G = 9.8f;                           // gravity
const float MOLAR_MASS_AIR = 0.02896f;          // kg/mol
const float R_GAS_CONST = 8.315f;               // gas constant
const float GAMMA = 1.4f;

const float DROGUE_DEPLOY_ALTITUDE = 1524.0f;     // m
const float MAIN_DEPLOY_ALTITUDE = 300.0f;        // m
float GROUND_ALTITUDE = 700.0f;                   // m

const float ACCEL_MIN = -100.0f; // m/s^2, anything less than this is likely bad data (but we allow for high positive accel during burn)
const float ACCEL_MAX = 100.0f;  // m/s^2, anything above this is likely bad data

const float ALT_MIN = 0.0f;  // meters (allow for some negative drift if starting at 0)
const float ALT_MAX = 15000.0f; // meters 

const float TEMP_C_MIN = -50.0f; // Celsius
const float TEMP_C_MAX = 50.0f;  // Celsius
const float TEMP_K_MIN = TEMP_C_MIN + 273.15f; // Kelvin
const float TEMP_K_MAX = TEMP_C_MAX + 273.15f; // Kelvin

const float PILOT_TERM_VEL = 6.0f; // m/s, terminal velocity with drogue chute deployed (for fallback timer calculations)
const float DROGUE_TERM_VEL = 5.5f;   // m/s, terminal velocity with main chute deployed (for fallback timer calculations)

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

    if (detector->avg_size >= AD_CAPACITY) {
        detector->slope = linreg_slope(detector->average, detector->avg_index, AD_CAPACITY);
    }
}


int detect_event(Detector * detector, FlightPhase phase) {
    if (phase == ST_WAIT_MECO)
        return buffer_lt(detector->average, detector->avg_size, 0);

    if (phase == ST_WAIT_APOGEE)
        // With altitude, a negative slope (< 0) correctly indicates the rocket is falling!
        return detector->slope < 0.0f; // pressure slope should be positive at apogee since pressure increases as we descend

    if (phase == ST_WAIT_DROGUE)                  
        // We look for altitude dropping BELOW the drogue deployment height
        return buffer_lt(detector->average, detector->avg_size, DROGUE_DEPLOY_ALTITUDE); 

    if (phase == ST_WAIT_MAIN)                     
        // We look for altitude dropping BELOW the main deployment height
        return buffer_lt(detector->average, detector->avg_size, MAIN_DEPLOY_ALTITUDE);  

    if (phase == ST_WAIT_GROUND)
        // Altitude slope hits 0 (no longer changing)
        return buffer_lt(detector->average, detector->avg_size, 20.0f);

    return 0;
}


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
    *one_k_time += (uint32_t)(t_1k * 1000);
}


void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt, int size) {
    int N = size;

    float t0 = time_arr[N / 2];
    float h0 = altitude_arr[N / 2];

    float sum_t  = 0.0f, sum_t2 = 0.0f, sum_t3 = 0.0f, sum_t4 = 0.0f;
    float sum_h  = 0.0f, sum_ht = 0.0f, sum_ht2 = 0.0f;

    for (int i = 0; i < N; ++i) {
        float t  = time_arr[i] - t0;
        float h  = altitude_arr[i] - h0;
        float t2 = t * t;
        float t3 = t2 * t;
        float t4 = t2 * t2;

        sum_t   += t;
        sum_t2  += t2;
        sum_t3  += t3;
        sum_t4  += t4;
        sum_h   += h;
        sum_ht  += h * t;
        sum_ht2 += h * t2;
    }

    // Matrix M:
    // | sum_t4  sum_t3  sum_t2 |
    // | sum_t3  sum_t2  sum_t  |
    // | sum_t2  sum_t   N      |

    float M00 = sum_t4, M01 = sum_t3, M02 = sum_t2;
    float M10 = sum_t3, M11 = sum_t2, M12 = sum_t;
    float M20 = sum_t2, M21 = sum_t,  M22 = (float)N;

    float R0 = sum_ht2, R1 = sum_ht, R2 = sum_h;

    // Determinant of M via cofactor expansion
    float det = M00 * (M11 * M22 - M12 * M21)
              - M01 * (M10 * M22 - M12 * M20)
              + M02 * (M10 * M21 - M11 * M20);

    if (fabsf(det) < 1e-2f) {
        *inst_accel = 0.0f;
        *vel        = 0.0f;
        *alt        = h0;
        return;
    }

    // Cramer's rule: replace each column with RHS vector and compute det

    // Solve for a: replace col 0
    float det_a = R0  * (M11 * M22 - M12 * M21)
                - M01 * (R1  * M22 - M12 * R2 )
                + M02 * (R1  * M21 - M11 * R2 );

    // Solve for b: replace col 1
    float det_b = M00 * (R1  * M22 - M12 * R2 )
                - R0  * (M10 * M22 - M12 * M20)
                + M02 * (M10 * R2  - R1  * M20);

    // Solve for c: replace col 2
    float det_c = M00 * (M11 * R2  - R1  * M21)
                - M01 * (M10 * R2  - R1  * M20)
                + R0  * (M10 * M21 - M11 * M20);

    float a = det_a / det;
    float b = det_b / det;
    float c = det_c / det;

    *inst_accel = 2.0f * a;
    *vel        = b;
    *alt        = c + h0;  // un-shift back to original altitude frame
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
            if (reading <= ACCEL_MIN || reading >= ACCEL_MAX)
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

float linreg_slope(float *buf, int start, int n) {
    float sum_y = 0, sum_xy = 0;
    for (int i = 0; i < n; i++) {
        float y = buf[(start + i) % n];
        sum_y  += y;
        sum_xy += i * y;
    }

    float sum_i  = (n * (n - 1)) / 2.0f;
    float sum_i2 = (n * (n - 1) * (2*n - 1)) / 6.0f;
    float denom  = n * sum_i2 - sum_i * sum_i;

    return (n * sum_xy - sum_i * sum_y) / denom;
}
