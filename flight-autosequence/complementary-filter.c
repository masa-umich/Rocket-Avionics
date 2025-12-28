
#include "complementary-filter.h"

extern const uint32_t period;
const float PI = 3.14159265f;
const float dt = period / 1000.0f; // convert to seconds

void cf_init(ComplementaryFilter* cf, float tau){
    cf->accel_angle_hpf[0] = 0.0f;
    cf->accel_angle_hpf[1] = 0.0f;
    cf->accel_angle_hpf[2] = 0.0f;

    cf->gyro_angle_lpf[0] = 0.0f;
    cf->gyro_angle_lpf[1] = 0.0f;
    
    cf->prev_gyro.XL_x = 0.0f;
    cf->prev_gyro.XL_y = 0.0f;

    cf->alpha = tau / (tau + dt); 
}

void cf_update(ComplementaryFilter* cf, float* pitch, float* roll){
    // Average accelerometer and gyroscope data from both IMUs
    float accel_x = (sensors_h.imu1.XL_x + sensors_h.imu2.XL_x) / 2.0f;
    float accel_y = (sensors_h.imu1.XL_y + sensors_h.imu2.XL_y) / 2.0f;
    float accel_z = (sensors_h.imu1.XL_z + sensors_h.imu2.XL_z) / 2.0f;
    float gyro_x = (sensors_h.imu1.W_X + sensors_h.imu2.W_X) / 2.0f;
    float gyro_y = (sensors_h.imu1.W_Y + sensors_h.imu2.W_Y) / 2.0f;
    // may not want to take average for data depending on noise characteristics
    
    // Extract accelerometer and gyroscope data
    float accel_pitch, accel_yaw;
    accel_to_angle(accel_x, accel_y, accel_z, &accel_pitch, &accel_yaw);
    
    
    // Low-pass filter for accelerometer angles
    cf->accel_angle_lpf[0] += (1 - cf->alpha) * (accel_pitch - cf->accel_angle_lpf[0]);
    cf->accel_angle_lpf[1] += (1 - cf->alpha) * (accel_yaw - cf->accel_angle_lpf[1]);

    // High-pass filter for gyroscope angles
    float gyro_angle_x = cf->prev_gyro[0] + gyro_x * dt;
    float gyro_angle_y = cf->prev_gyro[1] + gyro_y * dt;
    cf->gyro_angle_hpf[0] = cf->alpha * (gyro_angle_x + cf->gyro_angle_hpf[0] - cf->prev_gyro[0]);
    cf->gyro_angle_hpf[1] = cf->alpha * (gyro_angle_y + cf->gyro_angle_hpf[1] - cf->prev_gyro[1]);

    // Combine the two filtered angles
    *pitch = cf->accel_angle_lpf[0] + cf->gyro_angle_hpf[0];
    *roll  = cf->accel_angle_lpf[1] + cf->gyro_angle_hpf[1];

    // Update previous gyroscope readings
    cf->prev_gyro[0] = cf->gyro_angle_hpf[0];
    cf->prev_gyro[1] = cf->gyro_angle_hpf[1];

}

void accel_to_angle(float accel_x, float accel_y, float accel_z,
                      float* pitch, float* yaw){
    *pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * (180.0f / PI);
    *yaw   = atan2f(accel_y, accel_z) * (180.0f / PI);
}