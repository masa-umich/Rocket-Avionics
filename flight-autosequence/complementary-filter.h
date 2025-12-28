#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "apogee-detection/apogee-functions.h"
#include "flight-autosequence/autosequence-script.h"
#include "board-drivers/LSM6DSO32XTR/inc/LSM6DSO32XTR.h"

typedef struct {
    float accel_angle_lpf[3];
    float gyro_angle_hpf[2];
    float prev_gyro[2];
    float alpha;

} ComplementaryFilter;

void cf_init(ComplementaryFilter * cf, float tau);

void cf_update(ComplementaryFilter * cf, float * pitch, float * roll);

void accel_to_angle(float accel_x, float accel_y, float accel_z,
                      float* pitch, float* yaw);


#endif