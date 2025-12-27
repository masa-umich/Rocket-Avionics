#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "apogee-detection/apogee-functions.h"
#include "flight-autosequence/autosequence-script.h"

typedef struct {
    float accel_angle_hpf[2];   // pitch, yaw
    float gyro_angle_lpf[2];     
    float prev_gyro[2];
    float alpha;

} ComplementaryFilter;

void cf_init(ComplementaryFilter * cf, float tau);

void cf_update(ComplementaryFilter * cf, float accel_x, float accel_y, float accel_z,
               float gyro_pitch, float gyro_yaw, float dt,
               float * pitch, float * roll);

void accel_to_angle(float accel_x, float accel_y, float accel_z,
                      float* pitch, float* yaw);




#endif