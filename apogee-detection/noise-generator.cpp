#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cmath>

using namespace std;

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
    double compute_pressure(double height)
    {
        double P;
        double exponent;

        if (height < 11000)
        {
            exponent = (g * M) / (R * lapse_rate);
            P = pow(P_sea_level * (1 - ((lapse_rate * height) / T_sea_level)), exponent);
        }

        else 
        {
            exponent = - ((g * M) / (R * T_iso)) * (height - 11000);
            P = P_11k * exp(exponent);
        }

        P = P * 1.01325 * 1000; //convert from atmospheres to millibar 
        
        //baro displays readings 100x bigger
        return P * 100;
    }

    // this takes the raw height reading from MASTRAN and computes the actual
    // temperature reading in celsius
    double compute_temp(double height)
    {
        double TEMP;
        if (height < 11000)
            TEMP = T_sea_level - (lapse_rate * height) - 273.15;
        else    
            TEMP = T_iso - 273.15;
        
        //baro displays readings 100x bigger
        return TEMP * 100;
    }
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
   std::pair<double, double> forward_calculation(double D1, double D2)
   {
      double dT = D2 - (C5 * pow(2, 8));
      double TEMP = 2000 + ((dT * C6) / pow(2, 23));
      double OFF = (C2 * pow(2, 16)) + ((C4 * dT) / pow(2, 7));
      double SENS = (C1 * pow(2, 15)) + ((C3 * dT) / pow(2, 8));

      if (TEMP < 2000) // is temp less than 20 C?
      {
         double T2 = pow(dT, 2) / pow(2, 31);
         double OFF2 = (5 * pow(TEMP - 2000, 2)) / 2;
         double SENS2 = (5 * pow(TEMP - 2000, 2)) / pow(2, 2);

         if (TEMP < 1500) // is temp less than -15 C?
         {
            OFF2 = OFF2 + (7 * pow(TEMP + 1500, 2));
            SENS2 = SENS2 + ((11 * pow(TEMP + 1500, 2)) / 2);

            TEMP = TEMP - T2;
            OFF = OFF - OFF2;
            SENS = SENS - SENS2;
         }
      }

      double P = (D1 * (SENS / pow(2, 21)) - OFF) / pow(2, 15);
      return {P, TEMP};
   }

   //aim for 0.00001
   bool almost_equal(double a, double b, double relative_tolerance = 0.001)
   {
      if (abs(a - b) <= (max(abs(a), abs(b)) * relative_tolerance))
         return true;
      else
         return false;
   }

   std::pair<int, int> guess_and_check(double P, double TEMP)
   {
      std::pair<double, double> P_T;
      double D1_guess = 9'000'000;
      double D2_guess = 9'000'000;
      double D1_dummy = -1;
      double D1, D2;
      bool found_D2 = false;
      bool found_D1 = false;

      // we first send in a dummy_D1, becuase D1 has no affect on determining TEMP,
      // but D2 does, so this first while loop does guess and check, trying different
      // values for D2 until the forward calculation matches our known temperature
      int loop_counter = 0;
      while (!found_D2)
      {
         P_T = forward_calculation(D1_dummy, D2_guess);

         if (almost_equal(TEMP, P_T.second))
         {
            D2 = D2_guess;
            found_D2 = true;
         }

         else if (TEMP < P_T.second)
            D2_guess -= 5;

         else if (TEMP > P_T.second)
            D2_guess += 5;

        ++loop_counter;
        if(loop_counter > 2000000)
        {
            cout << "Missed one\n\n";
            break;
        }
      }

      // Since we've exited the first loop we now know our D2 value, although not needed
      // we will pass it to the forward_calculation function.
      // this time we don't have a D1_dummy, but make an actual guess -> trying different
      // values of D1 until the forward_calculation matches our known pressure
      loop_counter = 0;
      while (!found_D1)
      {
         P_T = forward_calculation(D1_guess, D2);

         if (almost_equal(P, P_T.first))
         {
            D1 = D1_guess;
            found_D1 = true;
         }
         else if (P < P_T.first)
            D1_guess -= 5;
         else if (P > P_T.first)
            D1_guess += 5;

         ++loop_counter;
         if (loop_counter > 2000000)
         {
             cout << "Missed one\n\n";
             break;
         }
      }

      return {D1, D2};
   }
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

double gaussian_random(double mean, double stddev)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(gen);
}

int main()
{

   Environment env;
   Barometer baro_1;
   Barometer baro_2;

   std::pair<int, int> baro1_D1_D2;
   std::pair<int, int> baro2_D1_D2;
   
   std::ifstream inFS("height_data.txt");
   std::vector<long double> heights;
   string height;

   while (inFS >> height) 
      heights.push_back(std::stold(height));
   
   inFS.close();

   Timer time;
   time.start();

   for (size_t i = 0; i < heights.size(); i += 10)
   {
      double current_height = heights[i];

      double P_true = env.compute_pressure(current_height);
      double T_true = env.compute_temp(current_height);

      // inject noise
      double P_noisy_1 = P_true + gaussian_random(0, 1.5);
      double T_noisy_1 = T_true + gaussian_random(0, 8);

      double P_noisy_2 = P_true + gaussian_random(0, 1.5);
      double T_noisy_2 = T_true + gaussian_random(0, 8);

      // time.stop();
      // double current_time = time.seconds();

      baro1_D1_D2 = baro_1.guess_and_check(P_noisy_1, T_noisy_1);
      baro2_D1_D2 = baro_2.guess_and_check(P_noisy_2, T_noisy_2);

      cout << "Time(s): " << static_cast<double>(i) / 100 << "\n";
      cout << "MASTRAN height(m): " << current_height << "\n";
      cout << "Computed pressure(millibar): " << P_true / 100 << "\n";
      cout << "Computed temp(Celsius): " << T_true / 100 << "\n";

      cout << "Noisy pressure (millibar) Baro-1: " << P_noisy_1 / 100 << " | ";
      cout << "Baro-2: " << P_noisy_2 / 100 << "\n";

      cout << "Noisy temp (Celsius) Baro-1: " << T_noisy_1 / 100 << " | ";
      cout << "Baro-2: " << T_noisy_2 / 100 << "\n";

      cout << "Computed noisy D1 (digital pressure value) Baro-1: " << baro1_D1_D2.first << " | ";
      cout << "Baro-2: " << baro2_D1_D2.first << "\n";

      cout << "Computed noisy D2 (digital temp val): Baro-1: " << baro1_D1_D2.second << " | ";
      cout << "Baro-2: " << baro2_D1_D2.second << "\n";

      auto [baro1_final_P, baro1_final_TEMP] = baro_1.forward_calculation(baro1_D1_D2.first, baro1_D1_D2.second);
      auto [baro2_final_P, baro2_final_TEMP] = baro_2.forward_calculation(baro2_D1_D2.first, baro2_D1_D2.second);

      cout << "Final compensated pressure from driver: Baro-1: " << baro1_final_P / 100 << " | ";
      cout << "Baro-2: " << baro2_final_P / 100 << "\n";

      cout << "Final compensated temp from driver: Baro-2: " << baro1_final_TEMP / 100 << " | ";
      cout << "Baro-2: " << baro2_final_TEMP / 100 ;

      cout << "\n\n";

      // time.start();
   }
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
   
}