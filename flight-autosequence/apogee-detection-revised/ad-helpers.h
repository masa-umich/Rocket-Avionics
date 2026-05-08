#ifndef AD_HELPERS_H
#define AD_HELPERS_H

#include <math.h>

// returns the mean of a given array
float mean(int size, float *arr);


// returns the min of two integers
int min(int x, int y);


// returns the max of two integers
int max(int x, int y);


// returns 1 if at least 80% of the values in the buffer
// are less than the search value, else returns 0
int buffer_lt(float* buf, int size, int cap, float search_value);


// returns 1 if at least 80% of the values in the buffer
// are greater than the search value, else returns 0
int buffer_gt(float* buf, int size, int cap, float search_value);


// returns 1 if at least 80% of the values in the buffer
// are nearly equal to the search value, else returns 0
int buffer_eq(float* buf, int size, int cap, float search_value);


// fits a quadratic curve to the given altitude and time arrays 
// and uses the coefficients to compute instantaneous acceleration, velocity, and altitude 
// at the time of lockout (last element in the arrays)
void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt, int size);


// returns the slope of the buffer using linear regression
float linreg_slope(float* buf, int start, int n);

#endif
