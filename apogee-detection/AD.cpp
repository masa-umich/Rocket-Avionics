#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cmath>

#include <numeric>
#include <iterator>
#include <iomanip>

#include "noise_generator.hpp"

using namespace std;

struct Detector
{
    double slope[10];
    int slope_i = 0;
    int slope_size = 0;

    static const int CAPACITY = 10;
    double readings_1[CAPACITY]; 
    int index_1 = 0;
    int size_1 = 0;

    double readings_2[CAPACITY];
    int index_2 = 0;
    int size_2 = 0;

    double previous[5];
    int prev_index = 0;
    int prev_size = 0;

    double mean(int size, double * arr)
    {
        if (size == 0)
            return 0;
        double sum = std::accumulate(arr, arr + size, 0);
        return sum / size;
    }

    void insert(double baro1, double baro2)
    {
        if(std::isnan(baro1))
            baro1 = mean(size_1, readings_1);
        readings_1[index_1] = baro1;
        index_1++;
        index_1 = index_1 % CAPACITY;
        size_1++;


        if(std::isnan(baro2))
            baro2 = mean(size_2, readings_2);
        readings_2[index_2] = baro2;
        index_2++;
        index_2 = index_2 % CAPACITY;
        size_2++;

        previous[prev_index] = (mean(size_1, readings_1) + mean(size_2, readings_2)) / 2;
        prev_index++;
        prev_index = prev_index % 5;
        prev_size++;

        if(prev_size > 1)
        {
            slope[slope_i] = previous[(prev_index + 5 - 1) % 5 ] - previous[(prev_index + 5 - 2) % 5];
            slope_size++;
            slope_i = (slope_i + 1) % 10;
        }
    }

    bool detect_apog()
    {
        // if(prev_size < 5)
        if (slope_size < 10)
            return false;
        else
        { // BAD and fragile approach, frequently fails to detect anything
            // if (previous[prev_index] > previous[(prev_index + 5 - 1) % 5] &&
            // previous[(prev_index + 5 - 1) % 5] > previous[(prev_index + 5 - 2) % 5] &&
            // previous[(prev_index + 5 - 2) % 5] > previous[(prev_index + 5 - 3) % 5] &&
            // previous[(prev_index + 5 - 3) % 5] > previous[(prev_index + 5 - 4) % 5])

            // if (slope[0] > 0 && slope[1] > 0 && slope[2] > 
            //     0 && slope[3] > 0 && slope[4] && slope[5] > 
            //     0 && slope[6] > 0 && slope[7] > 0 && slope[8] > 
            //     0 && slope[9] > 0)

            int count = 0;
            for(int i = 0; i < 10; ++i)
            {
                if(slope[i] > 0)
                    ++count;
            }

            if(count >= 9)
                return true;
        }
        return false;
    }
    
};


void launch()
{
    Environment env;
    Barometer baro_1;
    Barometer baro_2;
    Detector detect;
    //    bool apogee;

    std::pair<int, int> baro1_D1_D2;
    std::pair<int, int> baro2_D1_D2;

    std::ifstream inFS("height_data.txt");
    std::vector<long double> heights;
    string height;

    while (inFS >> height)
        heights.push_back(std::stold(height));

    inFS.close();

    for (size_t i = 0; i < heights.size(); i += 5)
    {
        double current_height = heights[i];

        //13647
        if (current_height > 18000)
        {
            double P_true = env.compute_pressure(current_height);
            double T_true = env.compute_temp(current_height);

            // inject noise
            double P_noisy_1 = P_true + gaussian_random(0, 1.5);
            double T_noisy_1 = T_true + gaussian_random(0, 8);

            double P_noisy_2 = P_true + gaussian_random(0, 1.5);
            double T_noisy_2 = T_true + gaussian_random(0, 8);

            baro1_D1_D2 = baro_1.guess_and_check(P_noisy_1, T_noisy_1);
            baro2_D1_D2 = baro_2.guess_and_check(P_noisy_2, T_noisy_2);

            auto [baro1_final_P, baro1_final_TEMP] = baro_1.forward_calculation(baro1_D1_D2.first / 100, baro1_D1_D2.second / 100);
            auto [baro2_final_P, baro2_final_TEMP] = baro_2.forward_calculation(baro2_D1_D2.first / 100, baro2_D1_D2.second / 100);

            cout << setw(10) << left << current_height;
            cout << setw(10) << right << static_cast<double>(i) / 100 << "\n";

            if (detect.size_1 < 10 && detect.size_2 < 10)
            {
                detect.insert(baro1_final_P, baro2_final_P);
            }
            else
            {
                detect.insert(baro1_final_P, baro2_final_P);
                if (detect.detect_apog())
                {
                    cout << "Apogee Detected at time: " << static_cast<double>(i) / 100 << "\n";
                }
            }
        }
    }
}

int main()
{
    launch();
}

