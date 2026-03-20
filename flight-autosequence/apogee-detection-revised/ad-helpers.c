/*
 * ad-helpers.c
 *
 *  Created on: Mar 12, 2026
 *      Author: felix
 */
#include "ad-helpers.h"

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
        case IMU_DTR:
            if (reading <= ACCEL_MIN)
                return mean(size, buf);
            return reading;

        case BARO_DTR:
            if (reading <= BARO_MIN || reading >= BARO_MAX)
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
