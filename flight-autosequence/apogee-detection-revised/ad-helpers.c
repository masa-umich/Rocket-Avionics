/*
 * ad-helpers.c
 *
 *  Created on: Mar 12, 2026
 *      Author: felix
 */
#include "ad-helpers.h"


// fits a quadratic curve to the given altitude and time arrays 
// and uses the coefficients to compute instantaneous acceleration, velocity, and altitude 
// at the time of lockout (last element in the arrays)
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

    if (fabsf(det) < 1e-4f) {
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


// returns the mean of a given array
float mean(int size, float *arr) {
    if (size == 0)
        return 0.0f;
    float sum = 0.0f;
    for(int i = 0; i < size; ++i)
        sum += arr[i];

    return (sum / (float)size);
}


// returns the min of two integers
int min(int x, int y) {
    if(x < y)
        return x;
    else return y;
}


// returns the max of two integers
int max(int x, int y) {
    if(x > y)
        return x;
    else return y;
}


// returns 1 if at least 80% of the values in the buffer
// are less than the search value, else returns 0
int buffer_lt(float* buf, int size, int cap, float search_value) {
    if (size < cap)
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


// returns 1 if at least 80% of the values in the buffer
// are greater than the search value, else returns 0
int buffer_gt(float* buf, int size, int cap, float search_value) {
    if (size < cap)
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


// returns 1 if at least 80% of the values in the buffer
// are nearly equal to the search value, else returns 0
int buffer_eq(float* buf, int size, int cap, float search_value) {
    if (size < cap)
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


// returns the slope of the buffer using linear regression
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
