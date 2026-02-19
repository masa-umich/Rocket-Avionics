

#include "complementary-filter.h"

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

void cf_update(ComplementaryFilter* cf, float* pitch, float* yaw, 
                      float accel_x, float accel_y, float accel_z,
                      float gyro_x, float gyro_y){
    
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
    *yaw   = cf->accel_angle_lpf[1] + cf->gyro_angle_hpf[1];

    // Update previous gyroscope readings
    cf->prev_gyro[0] = cf->gyro_angle_hpf[0];
    cf->prev_gyro[1] = cf->gyro_angle_hpf[1];

}

void accel_to_angle(float accel_x, float accel_y, float accel_z,
                      float* pitch, float* yaw){
    *pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * (180.0f / PI);
    *yaw   = atan2f(accel_y, accel_z) * (180.0f / PI);
}



// ORIENTATION FILTER UPDATE - NOT USED YET
                //float pitch, yaw;
                //cf_update(&imu_cf, &pitch, &yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y);
                /*
                for (int i = 6; i > 0; --i) {
                    pitch_buf[i] = pitch_buf[i - 1];
                    yaw_buf[i] = yaw_buf[i - 1];
                }
                pitch_buf[0] = pitch;
                yaw_buf[0] = yaw;
                */