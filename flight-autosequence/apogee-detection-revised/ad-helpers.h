#ifndef AD_HELPERS_H
#define AD_HELPERS_H

#include "ad-functions.h"
#include <math.h>

float mean(int size, float *arr);

int min(int x, int y);

int max(int x, int y);

int buffer_lt(float* buf, int size, float search_value);

int buffer_gt(float* buf, int size, float search_value);

int buffer_eq(float* buf, int size, float search_value);


float fix_reading(float reading, float* buf, int size, DetectorType type);

void quadr_curve_fit(float* altitude_arr, float* time_arr, float* inst_accel, float* vel, float* alt, int size);

float linreg_slope(float* buf, int start, int n);

#endif
