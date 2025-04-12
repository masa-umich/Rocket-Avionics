#ifndef NOISE_H
#define NOISE_H

#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cmath>

class Environment 
{
private:
    double P_sea_level = 1;
    long double P_11k = 0.23015758656029023; //atmospheres
    double lapse_rate = 0.0065; //Celsius per meter
    double T_sea_level = 293; //Kelvin (20 Celsius)
    double T_iso = 216.65; //temp is roughly constant in isothermic region
    double g = 9.8; //acceleration due to gravity
    double M = 0.02896; //Molar mass of air kg/mol
    double R = 8.315; //gas constant

public:

    // this takes the raw height reading from MASTRAN and computes the actual 
    // pressure in millibar
    double compute_pressure(double height);

    // this takes the raw height reading from MASTRAN and computes the actual
    // temperature reading in celsius
    double compute_temp(double height);
};

class Barometer
{
private:
   double C1 = 40127; // pressure sensitivty
   double C2 = 36924; // pressure offset
   double C3 = 23317; // temperature coefficient of pressure sensitivityS
   double C4 = 23282; // temperature coefficient of pressure offset
   double C5 = 33464; // reference temperature
   double C6 = 28312; // temperature coefficient of the temperature
   // double D1 = 9085466; //digital pressure value
   // double D2 = 8569150; //digital temperature value

public:
   // this is the forward application of the 1st/2nd order pressure & temperature
   // calculations.
   // Since we have used the physics equations in compute_pressure and compute_temp,
   // we already have the result of this calculation.
   // what we want is the D1 and D2 that 'seed' this calculation
   // to calculate this - see guess_and_check function.
   std::pair<double, double> forward_calculation(double D1, double D2);

   //aim for 0.00001
   bool almost_equal(double a, double b, double relative_tolerance = 0.001);

   // returns D1 and D2 reading
   std::pair<int, int> guess_and_check(double P, double TEMP);

   //  Example:
   //  Time(s): 5.5
   //  MASTRAN height(m): 372.024
   //  Computed pressure(millibar): 970.102
   //  Computed temp(Celsius): 17.4318
   //  Noisy pressure (millibar) Baro-1: 970.128 | Baro-2: 970.102
   //  Noisy temp (Celsius) Baro-1: 17.2874 | Baro-2: 17.4982
   //  Computed noisy D1 (digital pressure value) Baro-1: 8961150 | Baro-2: 8958940
   //  Computed noisy D2 (digital temp val): Baro-1: 8486925 | Baro-2: 8493175
   //  Final compensated pressure from driver: Baro-1: 971.098 | Baro-2: 971.073
   //  Final compensated temp from driver: Baro-1: 17.3047 | Baro-2: 17.5157
   // view_data() prints the above report for each timestep across the rocket flight
   void view_data();
};

class Timer
{
   std::chrono::time_point<std::chrono::system_clock> cur;
   std::chrono::duration<double> elap;

public:
   Timer() : cur(), elap(std::chrono::duration<double>::zero()) {}

   void start()
   {
      cur = std::chrono::system_clock::now();
   }

   void stop()
   {
      elap += (std::chrono::system_clock::now() - cur);
   }

   void reset()
   {
      elap = std::chrono::duration<double>::zero();
   }

   double seconds()
   {
      return elap.count();
   }
};

double gaussian_random(double mean, double stddev);


#endif