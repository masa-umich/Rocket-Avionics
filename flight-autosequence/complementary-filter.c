
#include "complementary-filter.h"

extern const uint32_t period;
const float PI = 3.14159265f;

void cf_init(ComplementaryFilter* cf, float tau){
    cf->accel_angle_hpf[0] = 0.0f;
    cf->accel_angle_hpf[1] = 0.0f;
    cf->gyro_angle_lpf[0] = 0.0f;
    cf->gyro_angle_lpf[1] = 0.0f;
    cf->prev_gyro[0] = 0.0f;
    cf->prev_gyro[1] = 0.0f;

    float dt = period / 1000.0f; // convert to seconds
    cf->alpha = tau / (tau + dt); 
}

void cf_update(ComplementaryFilter* cf, float accel_x, float accel_y, float accel_z,
               float gyro_pitch, float gyro_yaw, float dt,
               float* pitch, float* roll){

    float accel_pitch, accel_yaw;
    accel_to_angle(accel_x, accel_y, accel_z, &accel_pitch, &accel_yaw);

    // High-pass filter on accelerometer angles
    cf->accel_angle_hpf[0] = cf->alpha * (cf->accel_angle_hpf[0] + accel_pitch - cf->prev_gyro[0] * dt);
    cf->accel_angle_hpf[1] = cf->alpha * (cf->accel_angle_hpf[1] + accel_yaw - cf->prev_gyro[1] * dt);

    // Low-pass filter on gyroscope angles
    cf->gyro_angle_lpf[0] = (1.0f - cf->alpha) * (cf->gyro_angle_lpf[0] + gyro_pitch * dt) + cf->alpha * cf->gyro_angle_lpf[0];
    cf->gyro_angle_lpf[1] = (1.0f - cf->alpha) * (cf->gyro_angle_lpf[1] + gyro_yaw * dt) + cf->alpha * cf->gyro_angle_lpf[1];

    // Combine filters
    *pitch = cf->accel_angle_hpf[0] + cf->gyro_angle_lpf[0];
    *roll  = cf->accel_angle_hpf[1] + cf->gyro_angle_lpf[1];

    // Store previous gyro readings
    cf->prev_gyro[0] = gyro_pitch;
    cf->prev_gyro[1] = gyro_yaw;
}

void accel_to_angle(float accel_x, float accel_y, float accel_z,
                      float* pitch, float* yaw){
    *pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * (180.0f / PI);
    *yaw   = atan2f(accel_y, accel_z) * (180.0f / PI);
}