#include <assert.h> 
#include "ad-helpers.h"
#include <stdio.h>

#define TOLERANCE 1e-1f // tolerance for floating point comparisons in tests

void almost_equal(float a, float b) {
    
}

// MEAN
void test_mean() {
    float arr1[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    almost_equal(mean(5, arr1), 3.0f);

    float arr2[] = {10.0f, 20.0f, 30.0f};
    almost_equal(mean(3, arr2), 20.0f);

    float arr3[] = {5.0f};
    almost_equal(mean(1, arr3), 5.0f);

    float arr4[] = {0.0f, 0.0f, 0.0f};
    almost_equal(mean(3, arr4), 0.0f);

    float arr5[] = {};
    almost_equal(mean(0, arr5), 0.0f);
}

void test_min() {
    assert(min(3, 5) == 3);
    assert(min(-1, 1) == -1);
    assert(min(0, 0) == 0);
    assert(min(10, 2) == 2);
}

void test_max() {
    assert(max(3, 5) == 5);
    assert(max(-1, 1) == 1);
    assert(max(0, 0) == 0);
    assert(max(10, 2) == 10);
}

void test_buffer_lt() {
    float buf1[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    assert(buffer_lt(buf1, 5, 5, 4.5f) == 1);
    assert(buffer_lt(buf1, 5, 5, 3.0f) == 0);

    float buf2[] = {10.0f, 20.0f, 30.0f};
    assert(buffer_lt(buf2, 3, 3, 31.0f) == 1);
    assert(buffer_lt(buf2, 3, 3, 25.0f) == 0);
    assert(buffer_lt(buf2, 3, 3, 15.0f) == 0);

    float buf3[] = {0.8f, 0.9f, 1.0f, 1.1f, 1.2f};
    assert(buffer_lt(buf3, 5, 5, 1.5f) == 1);

    float buf4[] = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 20.f};
    assert(buffer_lt(buf4, 11, 11, 1.8f) == 0);
    assert(buffer_lt(buf4, 11, 11, 1.81f) == 1);
}

void test_buffer_gt() {
    float buf1[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    assert(buffer_gt(buf1, 5, 5, 1.5f) == 1);
    assert(buffer_gt(buf1, 5, 5, 3.0f) == 0);

    float buf2[] = {10.0f, 20.0f, 30.0f};
    assert(buffer_gt(buf2, 3, 3, 5.0f) == 1);
    assert(buffer_gt(buf2, 3, 3, 15.0f) == 0);
    assert(buffer_gt(buf2, 3, 3, 25.0f) == 0);

    float buf3[] = {0.8f, 0.9f, 1.0f, 1.1f, 1.2f};
    assert(buffer_gt(buf3, 5, 5, 0.5f) == 1);

    float buf4[] = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 20.f};
    assert(buffer_gt(buf4, 11, 11, 1.2f) == 0);
    assert(buffer_gt(buf4, 11, 11, 1.19f) == 1);
    assert(buffer_gt(buf4, 5, 11, 1.21f) == 0);
}

void test_buffer_eq() {
    float buf1[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    assert(buffer_eq(buf1, 5, 5, 1.0f) == 1);
    assert(buffer_eq(buf1, 5, 5, 0.9f) == 0);

    float buf2[] = {10.0f, 10.0f, 10.0f};
    assert(buffer_eq(buf2, 3, 3, 10.0f) == 1);
    assert(buffer_eq(buf2, 3, 3, 9.9f) == 0);

    float buf3[] = {0.8f, 0.8f, 0.8f, 0.8f, 0.8f};
    assert(buffer_eq(buf3, 5, 5, 0.8f) == 1);

    float buf4[] = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f};
    assert(buffer_eq(buf4, 5, 5, 1.2f) == 0);
}

void test_linreg_slope() {
    // linear tests
    float buf1[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    almost_equal(linreg_slope(buf1, 0, 5), 1.0f);

    float buf2[] = {10.0f, 20.0f, 30.0f};
    almost_equal(linreg_slope(buf2, 0, 3), 10.0f);

    float buf3[] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    almost_equal(linreg_slope(buf3, 0, 5), 0.0f);

    float buf4[] = {1.0f, 2.0f, 1.5f, 3.0f, 2.5f};
    almost_equal(linreg_slope(buf4, 0, 5), 0.4f);

    float buf5[] = {10.5, 8.2, 11.1, 9.4, 10.8, 7.9, 12.0, 9.1, 10.2, 8.5, 11.5, 9.8, 10.1, 8.9, 10.0};
    almost_equal(linreg_slope(buf5, 0, 15), 0.0f);

    
    // nonlinear tests
}

void test_quadr_curve_fit() {
    float accel, vel, alt;

    // -- basic sanity --

    // pure quadratic: h(t) = -0.5t² + 5t + 10, eval at t=2 → accel=-1.0, vel=3.0, alt=18.0
    float h1[] = {10.0f, 14.5f, 18.0f, 20.5f, 22.0f};
    float t1[] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    quadr_curve_fit(h1, t1, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(1.0f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(3.0f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(18.0f)) < TOLERANCE);

    // linear: h(t) = 2t + 5, eval at t=2 → accel=0, vel=2.0, alt=9.0
    float h2[] = {5.0f, 7.0f, 9.0f, 11.0f, 13.0f};
    float t2[] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    quadr_curve_fit(h2, t2, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(2.0f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(9.0f)) < TOLERANCE);

    // constant: h(t) = 42, eval at t=2 → accel=0, vel=0, alt=42
    float h3[] = {42.0f, 42.0f, 42.0f, 42.0f, 42.0f};
    float t3[] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    quadr_curve_fit(h3, t3, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(42.0f)) < TOLERANCE);

    // degenerate (all same time) → fallback: accel=0, vel=0, alt=h_mid
    float h4[] = {10.0f, 11.0f, 12.0f, 13.0f, 14.0f};
    float t4[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    quadr_curve_fit(h4, t4, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(12.0f)) < TOLERANCE);

    // -- rocket flight phases (g=9.81 m/s², 1 Hz sampling) --

    // powered ascent: h(t) = -4.905t² + 80t + 500, eval at t=2 → vel=60.38, alt=640.38
    float h5[] = {500.0f, 575.095f, 640.38f, 695.855f, 741.52f};
    float t5[] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    quadr_curve_fit(h5, t5, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(-9.81f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(60.38f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(640.38f)) < TOLERANCE);

    // slow ascent near apogee: h(t) = -4.905t² + 9.81t + 845.095, eval at t=1 → vel=0, alt=850
    // (apogee exactly at midpoint t=1, window centred on peak)
    float h6[] = {845.095f, 850.0f, 845.095f};
    float t6[] = {0.0f, 1.0f, 2.0f};
    quadr_curve_fit(h6, t6, &accel, &vel, &alt, 3);
    assert(fabs(fabs(accel) - fabs(-9.81f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(0.0f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(850.0f)) < TOLERANCE);

    // descent: h(t) = -4.905t² - 20t + 700, eval at t=2 → vel=-39.62, alt=640.38
    float h7[] = {700.0f, 675.095f, 640.38f, 595.855f, 541.52f};
    float t7[] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    quadr_curve_fit(h7, t7, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(-9.81f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(-39.62f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(640.38f)) < TOLERANCE);

    // -- rocket flight (high frequency: dt=0.1s) --

    // fast ascent, tight window: h(t) = -4.905t² + 60t + 300, eval at t=0.2 → vel=58.038, alt=311.804
    float h8[] = {300.0f, 305.951f, 311.804f, 317.559f, 323.215f};
    float t8[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f};
    quadr_curve_fit(h8, t8, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(-9.81f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(58.038f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(311.804f)) < TOLERANCE);

    // descent, tight window: h(t) = -4.905t² - 15t + 400, eval at t=0.2 → vel=-16.962, alt=382.038
    float h9[] = {400.0f, 398.451f, 396.804f, 395.059f, 393.215f};  
    float t9[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f};
    quadr_curve_fit(h9, t9, &accel, &vel, &alt, 5);
    assert(fabs(fabs(accel) - fabs(-9.81f)) < TOLERANCE);
    assert(fabs(fabs(vel) - fabs(-16.962f)) < TOLERANCE);
    assert(fabs(fabs(alt) - fabs(396.804f)) < TOLERANCE);
}

void run_tests() {
    test_mean();
    test_min();
    test_max();
    test_buffer_lt();
    test_buffer_gt();
    test_buffer_eq();
    test_linreg_slope();
    test_quadr_curve_fit();
}

int main() {
    run_tests();
    return 0;
}
