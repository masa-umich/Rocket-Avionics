/*
 * ad-helpers.c
 *
 *  Created on: Mar 12, 2026
 *      Author: felix
 */
#include "ad-helpers.h"

int vector_sign(float x, float y, float z) {
    if (x + y + z > 0)
        return 1;
    return -1;
}

void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt){

    int N = ALTITUDE_BUFFER_SIZE;

    float sum_t = 0.0f;
    float sum_t2 = 0.0f;

    float sum_h = 0.0f;
    float sum_ht = 0.0f;
    float sum_ht2 = 0.0f;
    float sum_t4 = 0.0f;

    float t0 = time_arr[N/2];
    for (int i = 0; i < N; ++i) {
        time_arr[i] -= t0; // Center time data

        sum_t += time_arr[i];
        sum_t2 += time_arr[i] * time_arr[i];
        sum_h += altitude_arr[i];
        sum_ht += altitude_arr[i] * time_arr[i];
        sum_ht2 += altitude_arr[i] * time_arr[i] * time_arr[i];
        sum_t4 += time_arr[i] * time_arr[i] * time_arr[i] * time_arr[i];
    }

    float c = sum_h/N;
    float b = sum_ht/ sum_t2;
    float a = (sum_ht2 - c * sum_t2) / sum_t4;

    *inst_accel = 2.0f * a;
    *vel = b;
    *alt = c;
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
    for (int i = 0; i < AD_CAPACITY; ++i)
    {
        if (buf[i] < search_value)
            ++count;
    }
    if (count >= (AD_CAPACITY * 0.8))
        return 1;
    return 0;
}

int buffer_gt(float* buf, int size, float search_value) {
    if (size < AD_CAPACITY)
        return 0;

    int count = 0;
    for (int i = 0; i < AD_CAPACITY; ++i)
    {
        if (buf[i] > search_value)
            ++count;
    }
    if (count >= (AD_CAPACITY * 0.8))
        return 1;
    return 0;
}

int buffer_eq(float* buf, int size, float search_value) {
    if (size < AD_CAPACITY)
        return 0;

    int count = 0;
    for (int i = 0; i < AD_CAPACITY; ++i)
    {
        if (fabsf(buf[i] - search_value) < 1e-1f) // consider equal if within 0.1
            ++count;
    }
    if (count >= (AD_CAPACITY * 0.8))
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
