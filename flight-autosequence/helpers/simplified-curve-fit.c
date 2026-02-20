#ifndef SIMPLIFIED_CURVE_FIT_C
#define SIMPLIFIED_CURVE_FIT_C

const extern int ALTITUDE_BUFFER_SIZE;

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

#endif