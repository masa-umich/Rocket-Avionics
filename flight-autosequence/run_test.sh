#!/bin/bash
set -e  # stop immediately if any command fails

#echo "Running Python flight simulation to generate CSV data..."
#python3 tests/system_tests/AI_generated_testing/simulate_flight.py

echo "Compiling C simulation..."
gcc tests/system_tests/matlab_testing/test_autosequence_mat.c \
    tests/system_tests/matlab_testing/sim_hardware.c \
    apogee-detection-revised/ad-functions.c \
    apogee-detection-revised/ad-helpers.c \
    -I apogee-detection-revised \
    -o tests/system_tests/matlab_testing/flight_sim \
    -lm

echo "Running C simulation..."
tests/system_tests/matlab_testing/./flight_sim