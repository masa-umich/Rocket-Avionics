#include <assert.h> 
#include "ad-functions.h"
#include <stdio.h>

int almost_equal(float a, float b) {
    return fabsf(a - b) < 1e-1f; // tolerance of 0.1 for floating point comparisons
}

void test_insert_and_clear() {
    Detector detector;
    clear(&detector);

    // Test that all values are reset to 0
    for (int i = 0; i < AD_CAPACITY; ++i) {
        assert(detector.readings_1[i] == 0.0f);
        assert(detector.readings_2[i] == 0.0f);
        assert(detector.average[i] == 0.0f);
    }
    assert(detector.index_1 == 0);
    assert(detector.size_1 == 0);
    assert(detector.index_2 == 0);
    assert(detector.size_2 == 0);
    assert(detector.avg_index == 0);
    assert(detector.avg_size == 0);
    assert(detector.slope == 0.0f);

    // Test insert with valid readings
    insert(&detector, 1.0f, 2.0f, ST_WAIT_DROGUE, IMU_DTR);
    assert(detector.readings_1[0] == 1.0f);
    assert(detector.readings_2[0] == 2.0f);
    assert(detector.size_1 == 1);
    assert(detector.size_2 == 1);

    // Test insert with out-of-bounds reading (should be imputed)
    insert(&detector, -300.0f, 250.0f, ST_WAIT_DROGUE, IMU_DTR); // out of bounds for accelerometer
    assert(almost_equal(detector.readings_1[1], 1.0f)); // imputed to mean of buffer (which is just the first reading)
    assert(almost_equal(detector.readings_2[1], 2.0f)); // imputed to mean of buffer (which is just the first reading)
}


void test_compute_pressure() {
    // Test at sea level
    float pressure = compute_pressure(0.0f);
    printf("Pressure at sea level: %f hPa\n", pressure);
    assert(fabsf(pressure - P_GROUND) < 1.0f);

    // Test at 1000m
    pressure = compute_pressure(1000.0f);
    printf("Pressure at 1000m: %f hPa\n", pressure);
    assert(fabsf(pressure - 898.76f) < 1.0f);

    // Test at 5000m
    pressure = compute_pressure(5000.0f);
    printf("Pressure at 5000m: %f hPa\n", pressure);
    assert(fabsf(pressure - 540.19f) < 1.0f);

    // Test at 10000m
    pressure = compute_pressure(10000.0f);
    printf("Pressure at 10000m: %f hPa\n", pressure);
    assert(fabsf(pressure - 264.36f) < 1.0f);
}


void test_compute_height() {
    // Test at sea level
    float height = compute_height(P_GROUND);
    printf("Height at sea level pressure: %f m\n", height);
    assert(fabsf(height - 0.0f) < 1.0f);

    // Test at 1000m
    height = compute_height(898.76f);
    printf("Height at 1000m pressure: %f m\n", height);
    assert(fabsf(height - 1000.0f) < 10.0f);

    // Test at 5000m
    height = compute_height(540.19f);
    printf("Height at 5000m pressure: %f m\n", height);
    assert(fabsf(height - 5000.0f) < 10.0f);

    // Test at 10000m
    height = compute_height(264.36f);
    printf("Height at 10000m pressure: %f m\n", height);
    assert(fabsf(height - 10000.0f) < 10.0f);
}


void test_fix_reading() {
    Detector detector;
    clear(&detector);

    // Test with valid reading
    float corrected = fix_reading(1.0f, detector.readings_1, detector.size_1, IMU_DTR);
    assert(corrected == 1.0f);

    // Test with out-of-bounds reading (should be imputed to mean of buffer, which is 0 since buffer is empty)
    corrected = fix_reading(-300.0f, detector.readings_1, detector.size_1, IMU_DTR);
    assert(corrected == 0.0f);

    // Insert a valid reading and test that out-of-bounds reading is imputed to that value
    insert(&detector, 2.0f, 0.0f, ST_WAIT_DROGUE, IMU_DTR);
    corrected = fix_reading(-300.0f, detector.readings_1, detector.size_1, IMU_DTR);
    assert(corrected == 2.0f);
}

void run_tests() {
    test_insert_and_clear();
    test_compute_pressure();
    test_compute_height();
    test_fix_reading();
}

int main() {
    run_tests();
    return 0;
}