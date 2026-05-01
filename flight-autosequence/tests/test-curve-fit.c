#include <stdio.h>

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

    if (denominator == 0.0f || sum_t2 == 0.0f) {
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

void test_simulate(){
    const int N = 11;
    float time_arr[11];
    float altitude_arr[11];
    
    // Ground truth values at the exact middle of our 11-sample window
    float true_alt = 3000.0f;     // m
    float true_vel = 300.0f;      // m/s (approx Mach 0.88)
    float true_accel = -20.0f;    // m/s^2 (experiencing drag + gravity)
    
    // Generate simulated flight data (50ms intervals)
    float start_time = 12.0f;     // Arbitrary flight time (T+12 seconds)
    float sample_rate = 0.05f;    // 50 ms
    
    printf("Simulated Rocket Sensor Data:\n");
    printf("Time (s) | Altitude (m)\n");
    printf("------------------------\n");
    
    for (int i = 0; i < N; i++) {
        time_arr[i] = start_time + (i * sample_rate); 
        
        // Calculate physics relative to the middle reading
        float dt = time_arr[i] - (start_time + (N/2) * sample_rate); 
        
        // Standard Kinematic Equation: h = h0 + v*t + 0.5*a*t^2
        altitude_arr[i] = true_alt + (true_vel * dt) + (0.5f * true_accel * dt * dt);
        
        printf("%8.2f | %10.4f\n", time_arr[i], altitude_arr[i]);
    }
    
    // Outputs
    float calc_accel = 0.0f;
    float calc_vel = 0.0f;
    float calc_alt = 0.0f;
    
    // Run the filter
    quadr_curve_fit(altitude_arr, time_arr, &calc_accel, &calc_vel, &calc_alt, N);
    
    // Verify
    printf("\n--- Test Results ---\n");
    printf("          | Expected   | Calculated\n");
    printf("-----------------------------------\n");
    printf("Altitude  | %7.2f m  | %9.2f m\n", true_alt, calc_alt);
    printf("Velocity  | %7.2f m/s| %9.2f m/s\n", true_vel, calc_vel);
    printf("Accel     | %7.2f m/s2| %8.2f m/s2\n", true_accel, calc_accel);
}

void test_against_curve(){
    // Static arrays from the exact quadratic function h(t) = 2t^2 + 3t + 10
    float time_arr[11] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
    float altitude_arr[11] = {10.0f, 15.0f, 24.0f, 37.0f, 54.0f, 75.0f, 100.0f, 129.0f, 162.0f, 199.0f, 240.0f};
    
    // Output variables
    float calc_accel = 0.0f;
    float calc_vel = 0.0f;
    float calc_alt = 0.0f;
    
    // Run the curve fit
    quadr_curve_fit(altitude_arr, time_arr, &calc_accel, &calc_vel, &calc_alt, 11);
    
    // Print the results
    printf("--- Static Data Test Results ---\n");
    printf("          | Expected | Calculated\n");
    printf("---------------------------------\n");
    printf("Altitude  |    75.00 | %8.2f\n", calc_alt);
    printf("Velocity  |    23.00 | %8.2f\n", calc_vel);
    printf("Accel     |     4.00 | %8.2f\n", calc_accel);
}

int main() {
    //test_simulate();
    test_against_curve();
}